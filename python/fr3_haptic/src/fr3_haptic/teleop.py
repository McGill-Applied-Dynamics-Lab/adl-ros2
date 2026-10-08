r"""Haptic teleoperation of the real FR3 with ``haptic_teleop`` (adl-python).

The real-arm counterpart of adl-python ``examples/04_i3_newton_fr3_coupling.py``: the Inverse3
drives the FR3 along one interface axis, rendered with any ``haptic_teleop`` method.

Coupling methods (``zoh``, ``linear``, ``tdpa-zoh``, ``tdpa-linear``): the follower-side spring
is ``osc_controller`` on franka-pc at 1 kHz, its interface-axis gains set to the coupling
``kv``, ``dv`` by this script, and the leader state is streamed as its target. Proxy methods
(``proxy-rim``, ``proxy-fixed-mass``): the proxy runs here at the haptic rate and is streamed
as the target with a feedforward force; the controller gains come from ``--osc-params``.

Setup order matters for safety. Target streaming is enabled *before* the controller switch,
so ``Robot``'s periodic republishing never sends an old target to the new controller; the
hold pose (free axes, orientation) and the device origin come from the controller's own first
``ee_state`` sample, so the first target is where the robot already is.

**Forces are off unless ``--force``.** Run without it first: the arm should follow the handle
along the interface axis. The force fades in over ``--force-ramp-s``; the felt stiffness at the
handle is ``kv · scale · force_gain`` (printed at start).

    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml
    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --method linear --force
    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --plant-hz 1000 --save

Needs the colcon overlay sourced (``arm_client``), franka-server running with
``osc_controller``, and Haply's Inlet service.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import asdict

import numpy as np
from arm_client.control.parameters_client import ParametersClient
from arm_client.robot import Robot
from experiment_logger import ExperimentLogger, LoggingConfig
from haptic_teleop import (
    LoopRateMonitor,
    PassivityObserver,
    RateTicker,
    TickLog,
    gc_paused,
    jitter_report,
    run_loop,
)
from haptic_teleop.config import FixedMassConfig, LinearConfig, RenderingConfigs, SafetyConfig, TDPAConfig
from haptic_teleop.devices import Inverse3Device
from pyrim import InterfaceFrame
from utilities import apply_yaml_config

from .adapters import RobotModelAdapter
from .config import ModelConfig
from .plant import FR3Plant, FR3System, StalenessWatchdog
from .recorder import RobotStateRecorder
from .session import PROXY_METHODS, SessionConfig, TeleopSession, haptic_columns

TAG = "[fr3_teleop]"
SWITCH_INTERVAL_S = 1e-4  # GIL switch interval while the loops run [s]; Python's default is 5e-3
METHODS = ("zoh", "linear", "tdpa-zoh", "tdpa-linear", "proxy-rim", "proxy-fixed-mass")
AXES = "xyz"


def parse_axes(spec: str) -> tuple[tuple[int, ...], tuple[float, ...]]:
    """``"y,-x,z"`` → ``axes=(1, 0, 2), signs=(1, -1, 1)``: one signed device axis per world axis."""
    axes, signs = [], []
    for token in spec.split(","):
        token = token.strip().lower()
        name = token.lstrip("+-")
        if name not in AXES:
            raise argparse.ArgumentTypeError(f"--axes: {token!r} is not a device axis (x, y, z, optionally signed)")
        axes.append(AXES.index(name))
        signs.append(-1.0 if token.startswith("-") else 1.0)
    if len(axes) != 3 or len(set(axes)) != 3:
        raise argparse.ArgumentTypeError(f"--axes needs each of x, y, z once, got {spec!r}")
    return tuple(axes), tuple(signs)


def build_parser() -> argparse.ArgumentParser:
    """The CLI. Every flag can also be set from ``--conf`` YAML (by its ``dest``)."""
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    g = p.add_argument_group("method")
    g.add_argument("--method", choices=METHODS, default="zoh")
    g.add_argument("--kv", type=float, default=500.0, help="coupling stiffness, world [N/m]")
    g.add_argument("--dv", type=float, default=70.0, help="coupling damping, world [N·s/m]")
    g.add_argument("--linear-max-extrapolation", type=float, default=1.0, help="linear: cap, in plant periods")
    g.add_argument("--tdpa-max-damping", type=float, default=10.0, help="TDPA damping cap [N·s/m]; 0 = unbounded")
    g.add_argument("--proxy-mass", type=float, default=1.0, help="proxy-fixed-mass: mass [kg]")
    g.add_argument("--contact-surface", type=float, default=None, help="proxy methods: wall on the interface axis [m]")

    g = p.add_argument_group("rates")
    g.add_argument("--haptic-hz", type=float, default=1000.0, help="haptic loop rate [Hz]")
    g.add_argument("--plant-hz", type=float, default=50.0, help="target streaming / model update rate [Hz]")
    g.add_argument(
        "--sample-hz", type=float, default=None, help="plant sample rate [Hz]; default = plant-hz, 0 = every controller tick"
    )

    g = p.add_argument_group("interface")
    g.add_argument("--interface-axis", choices=tuple(AXES), default="z", help="robot base axis the interface moves along")
    g.add_argument("--interface-min", type=float, default=None, help="lowest commanded interface position [m]")
    g.add_argument("--interface-max", type=float, default=None, help="highest commanded interface position [m]")
    g.add_argument(
        "--interface-range",
        type=float,
        default=0.1,
        help="commanded interface position stays within ± this of the start [m]; 0 disables",
    )
    g.add_argument("--free-axis-stiffness", type=float, default=0.0, help="handle spring on the free axes [world N/m]")
    g.add_argument("--free-axis-damping", type=float, default=0.0, help="handle damper on the free axes [world N·s/m]")

    g = p.add_argument_group("inverse3")
    g.add_argument("--uri", default="ws://localhost:10001", help="Inlet service address")
    g.add_argument("--axes", type=parse_axes, default="y,-x,z", help="signed device axis per world axis")
    g.add_argument("--scale", type=float, default=1.0, help="world metres per device metre")
    g.add_argument("--force", action="store_true", help="render the force (default: off)")
    g.add_argument("--force-gain", type=float, default=0.1, help="handle N per world N")
    g.add_argument("--max-i3-force", type=float, default=3.0, help="clamp at the handle [N]")
    g.add_argument("--vel-filter-hz", type=float, default=8.0, help="leader velocity low-pass [Hz]; 0 disables")
    g.add_argument("--no-write-through", action="store_true", help="send the force on the next read instead")
    g.add_argument("--force-ramp-s", type=float, default=1.0, help="force fade-in at the start [s]")

    g = p.add_argument_group("safety")
    g.add_argument("--energy-budget-mj", type=float, default=0.0, help="guard: energy budget [mJ]; 0 disables")
    g.add_argument("--saturation-abort-ms", type=float, default=0.0, help="guard: saturation [ms]; 0 disables")
    g.add_argument("--chatter-hz", type=float, default=25.0, help="guard: force reversal rate [Hz]; 0 disables")
    g.add_argument("--chatter-deadband-n", type=float, default=1.0, help="guard: reversal deadband [N]")
    g.add_argument("--guard-cooldown-s", type=float, default=3.0, help="force off after a guard trip [s]; 0 ends the run")
    g.add_argument("--plant-max-age-ms", type=float, default=50.0, help="watchdog: largest plant sample age [ms]")
    g.add_argument("--watchdog-ramp-ms", type=float, default=100.0, help="watchdog: force ramp [ms]")
    g.add_argument("--feedforward-cap", type=float, default=15.0, help="proxy methods: feedforward clamp [N]")

    g = p.add_argument_group("robot")
    g.add_argument("--namespace", default="fr3")
    g.add_argument("--controller", default="osc_controller")
    g.add_argument("--osc-params", default=None, help="controller parameter YAML loaded before the coupling gains")
    g.add_argument("--home", action="store_true", help="move to the home configuration first")
    g.add_argument("--ee-state-topic", default="/fr3/osc/ee_state")
    g.add_argument("--task-wrench-topic", default="/fr3/osc/task_wrench")
    g.add_argument("--tool-tip-offset", type=float, nargs=3, default=None, help="proxy-rim: model interface point [m]")

    g = p.add_argument_group("run")
    g.add_argument("--seconds", type=float, default=60.0, help="run length [s]")
    g.add_argument("--save", action="store_true", help="write an MCAP run")
    g.add_argument("--output-dir", default="data/fr3_haptic", help="where --save writes runs")
    g.add_argument("--notes", default="", help="free text stored with the run")
    g.add_argument(
        "--robot-log-hz",
        type=float,
        default=50.0,
        help="record robot state, commanded torques, task error/wrench and EE state at this rate [Hz]; 0 disables",
    )
    return p


def interface_limits(
    absolute: tuple[float, float] | None, start: float, half_range: float
) -> tuple[float, float] | None:
    """Commanded interface bounds: ``absolute`` (``--interface-min/max``) intersected with
    ``start ± half_range`` (``--interface-range``; 0 disables it).

    Raises:
        ValueError: if the start lies outside the absolute bounds.
    """
    low, high = absolute if absolute is not None else (-np.inf, np.inf)
    if not low <= start <= high:
        raise ValueError(f"the robot starts at {start:.3f} m, outside --interface-min/max ({low}, {high})")
    if half_range > 0:
        low, high = max(low, start - half_range), min(high, start + half_range)
    if np.isinf(low) and np.isinf(high):
        return None
    return low, high


def controller_gains(args: argparse.Namespace) -> list[tuple[str, object]]:
    """``osc_controller`` parameters this run sets, after ``--osc-params``.

    Coupling methods: the interface axis becomes the coupling spring (``kv``, ``dv``, in N/m,
    so decoupling off), the twist feedforward carries the leader velocity, no wrench
    feedforward. Proxy methods: wrench feedforward on; gains are left to ``--osc-params``.
    """
    axis = args.interface_axis
    if args.method in PROXY_METHODS:
        return [("feedforward.wrench", True), ("feedforward.twist", True)]
    return [
        ("control.inertia_decoupling", False),
        (f"gains.k_pos_{axis}", float(args.kv)),
        (f"gains.d_pos_{axis}", float(args.dv)),
        ("feedforward.twist", True),
        ("feedforward.accel", False),
        ("feedforward.wrench", False),
    ]


def main(argv: list[str] | None = None) -> None:
    args = apply_yaml_config(build_parser(), argv)
    axes, signs = args.axes
    signs = np.asarray(signs)
    direction = np.zeros(3)
    direction[AXES.index(args.interface_axis)] = 1.0
    frame = InterfaceFrame.from_direction(direction)
    dim = frame.dim
    sample_hz = args.plant_hz if args.sample_hz is None else args.sample_hz
    limits = None
    if args.interface_min is not None or args.interface_max is not None:
        limits = (
            -np.inf if args.interface_min is None else args.interface_min,
            np.inf if args.interface_max is None else args.interface_max,
        )

    # -- robot and controller --------------------------------------------------------------
    robot = Robot(namespace=args.namespace)
    robot.wait_until_ready()
    if args.home:
        robot.home()
    robot.set_target_streaming(True)  # before the switch: no stale target reaches the controller
    plant = FR3Plant(
        robot,
        frame,
        ee_state_topic=args.ee_state_topic,
        task_wrench_topic=args.task_wrench_topic,
        sample_period_s=0.0 if sample_hz <= 0 else 1.0 / sample_hz,
        interface_limits=limits,
        record=True,
    )
    robot.controller_switcher_client.switch_controller(args.controller)
    params = ParametersClient(robot.node, target_node=args.controller)
    params.wait_until_ready()
    if args.osc_params is not None:
        params.load_param_config(args.osc_params)
    gains = controller_gains(args)
    params.set_parameters(gains)

    first = plant.wait_for_sample()
    hold = first.ee_pose
    plant.set_hold_pose(hold)
    plant.set_interface_limits(interface_limits(limits, float(first.x_i[0]), args.interface_range))

    # -- interface point (proxy-rim: the model's) ------------------------------------------
    system = None
    tool_correction = np.zeros(dim)
    if args.method == "proxy-rim":
        model_cfg = ModelConfig()
        if args.tool_tip_offset is not None:
            model_cfg.tool_tip_offset = list(args.tool_tip_offset)
        system = FR3System(RobotModelAdapter(node=robot.node, robot=robot, model_cfg=model_cfg, frame=frame))
        system.update()
        tool_correction = first.x_i - system.model.x_i
    leader_origin = hold.position - frame.lift(tool_correction)

    # -- method, device, safety ------------------------------------------------------------
    rendering = RenderingConfigs(
        linear=LinearConfig(max_extrapolation=args.linear_max_extrapolation),
        fixed_mass=FixedMassConfig(mass=args.proxy_mass),
        tdpa=TDPAConfig(max_damping=args.tdpa_max_damping),
    )
    method = rendering.build(
        args.method,
        dim,
        dt=1.0 / args.haptic_hz,
        stiffness=args.kv,
        damping=args.dv,
        contact_surface=-np.inf if args.contact_surface is None else args.contact_surface,
        contact_axis=0,
        basis=frame.basis,
    )
    device = Inverse3Device(
        origin=tuple(leader_origin),
        scale=args.scale,
        axes=axes,
        signs=tuple(signs),
        dim=3,
        force_gain=args.force_gain,
        force_enabled=args.force,
        max_force=args.max_i3_force if args.force else 0.0,
        write_through=not args.no_write_through,
        velocity_filter_hz=args.vel_filter_hz,
        uri=args.uri,
    )
    safety = SafetyConfig(
        energy_budget_mj=args.energy_budget_mj,
        saturation_abort_ms=args.saturation_abort_ms,
        chatter_hz=args.chatter_hz,
        chatter_deadband_n=args.chatter_deadband_n,
        guard_cooldown_s=args.guard_cooldown_s,
    )
    cfg = SessionConfig(
        method=args.method,
        haptic_hz=args.haptic_hz,
        plant_hz=args.plant_hz,
        max_force_world=args.max_i3_force / args.force_gain,
        force_ramp_s=args.force_ramp_s,
        guard_cooldown_s=args.guard_cooldown_s,
        free_axis_stiffness=args.free_axis_stiffness,
        free_axis_damping=args.free_axis_damping,
        feedforward_cap=args.feedforward_cap,
        tool_correction=tool_correction,
    )
    n_ticks = round(args.seconds * args.haptic_hz)
    log = TickLog(n_ticks, haptic_columns(dim))
    session = TeleopSession(
        cfg,
        device=device,
        plant=plant,
        method=method,
        frame=frame,
        leader_origin=leader_origin,
        observer=PassivityObserver(),
        guard=safety.build(max_i3_force=args.max_i3_force),
        watchdog=StalenessWatchdog(args.plant_max_age_ms * 1e-3, args.watchdog_ramp_ms * 1e-3, latch=True),
        device_axes=axes,
        device_signs=signs,
        device_scale=args.scale,
        log=log,
        system=system,
    )

    np.set_printoptions(precision=4, suppress=True)
    felt_k = args.kv * args.scale * args.force_gain
    print(f"{TAG} {args.method} along {args.interface_axis}; hold pose {hold.position} m, leader origin {leader_origin} m")
    print(f"{TAG} rates: haptic {args.haptic_hz:g} Hz, plant {args.plant_hz:g} Hz, samples {sample_hz:g} Hz (0 = every tick)")
    print(f"{TAG} coupling kv = {args.kv:g} N/m, dv = {args.dv:g} N·s/m (world) → felt {felt_k:g} N/m")
    print(f"{TAG} controller {args.controller}: {gains}")
    print(f"{TAG} force {'ON, clamped at %.1f N' % args.max_i3_force if args.force else 'OFF (--force to enable)'}")

    # -- run -------------------------------------------------------------------------------
    haptic_monitor = LoopRateMonitor(args.haptic_hz)
    result: dict[str, np.ndarray] = {"jitter": np.zeros(0)}

    def haptic_step(tick: int, t_s: float) -> None:
        haptic_monitor.tick()
        session.haptic_step(tick, t_s)
        if tick % round(args.haptic_hz) == 0:
            x_l, _ = session.leader
            latest = plant.sample
            x_i = latest[1].x_i if latest is not None else np.full(dim, np.nan)
            print(
                f"{TAG} t = {t_s:5.1f} s  leader - plant = {np.round((x_l - x_i) * 1e3, 1)} mm  "
                f"plant age {plant.sample_age_s() * 1e3:5.1f} ms  haptic {haptic_monitor.snapshot().measured_hz:.0f} Hz"
            )

    def haptic() -> None:
        try:
            result["jitter"] = run_loop(
                args.haptic_hz,
                haptic_step,
                stop_fn=lambda: session.stopped,
                n_ticks=n_ticks,
                ticker=RateTicker(args.haptic_hz),
            )
        finally:
            device.write(np.zeros(3))
            session.request_stop("haptic loop ended")

    sys.setswitchinterval(SWITCH_INTERVAL_S)
    thread = threading.Thread(target=haptic, name="haptic", daemon=True)
    recorder = None
    if args.robot_log_hz > 0:
        recorder = RobotStateRecorder(plant.node, args.robot_log_hz, seconds=args.seconds + 10.0)
    t0 = time.monotonic()
    try:
        with device, gc_paused():
            device.settle_and_zero()  # the handle starts exactly on the leader origin
            t0 = time.monotonic()
            thread.start()
            try:
                run_loop(
                    args.plant_hz,
                    session.plant_step,
                    stop_fn=lambda: session.stopped,
                    n_ticks=round(args.seconds * args.plant_hz) + 1,
                )
            except KeyboardInterrupt:
                print(f"\n{TAG} interrupted")
            finally:
                session.request_stop(session.shared.stop_reason or "plant loop ended")
                thread.join(timeout=5.0)
                device.write(np.zeros(3))
    finally:
        session.shutdown_robot()
        if recorder is not None:
            recorder.close()
        plant.close()

    # -- report and log --------------------------------------------------------------------
    log.finish(session.shared.ticks)
    print(f"{TAG} stopped: {session.shared.stop_reason}")
    print(f"{TAG} {haptic_monitor.totals().summary('haptic ')}")
    if len(result["jitter"]):
        print(f"{TAG} {jitter_report(result['jitter'])}")
    print(f"{TAG} guard trips: {session.shared.guard_trips}, watchdog trips: {session.watchdog.trips}")
    print(f"{TAG} plant samples: {len(plant.history)}")
    if recorder is not None:
        print(f"{TAG} robot log ({args.robot_log_hz:g} Hz): {recorder.summary()}")

    metadata = {
        "args": {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(args).items()},
        "controller_gains": gains,
        "session": {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(cfg).items()},
        "hold_position": hold.position.tolist(),
        "hold_orientation_xyzw": hold.orientation.as_quat().tolist(),
        "leader_origin": leader_origin.tolist(),
        "stop_reason": session.shared.stop_reason,
    }
    logger_config = (
        LoggingConfig.to_file(args.output_dir, notes=args.notes or f"fr3 {args.method} kv={args.kv} dv={args.dv}")
        if args.save
        else LoggingConfig.in_memory()
    )
    with ExperimentLogger(logger_config, metadata=metadata) as logger:
        log.to_logger(logger, "haptic", 1.0 / args.haptic_hz)
        # Plant samples on the local clock relative to the run start (approximately the haptic t = 0)
        for s in plant.history:
            logger.log_sample(
                "plant",
                {"x_i": s.x_i, "v_i": s.v_i, "lam": s.lam, "t_s": s.t_s, "ee_position": s.ee_position, "task_force": s.task_force},
                timestamp_s=s.rx_s - t0,
            )
        for t_s, x, v, f_ff in session.shared.commands:
            logger.log_sample("command", {"x": x, "v": v, "f_ff": f_ff}, timestamp_s=t_s)
        # Robot state, commanded torques, task error/wrench, EE state; same clock as "plant"
        if recorder is not None:
            recorder.to_logger(logger, t0)
    robot.shutdown()


if __name__ == "__main__":
    main()
