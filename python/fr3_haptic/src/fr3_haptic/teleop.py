r"""Haptic teleoperation of the real FR3 with ``haptic_teleop`` (adl-python).

The real-arm counterpart of adl-python ``examples/04_i3_newton_fr3_coupling.py``: the Inverse3
drives the FR3 along one interface axis, rendered with any ``haptic_teleop`` method.

Coupling methods (``zoh``, ``linear``, ``tdpa-zoh``, ``tdpa-linear``): the follower-side spring
is ``osc_controller`` on franka-pc at 1 kHz, its interface-axis gains set to the coupling
``kv``, ``dv`` by this script, and the leader state is streamed as its target. Proxy methods
(``proxy-rim``, ``proxy-fixed-mass``): the proxy runs here at the haptic rate and is streamed
as the target with a feedforward force; the controller gains come from ``--osc-params``.

The haptic loop (device, rendering method, guard, watchdog) runs in its own process
(``haptic_teleop.HapticProcess``) that imports no ROS: nothing this process does — `Robot`'s
1 kHz subscriptions, the robot-state recorder — can take its interpreter lock. This process
runs the ROS side: the command loop (haptic -> robot) and the RIM model loop (robot -> haptic);
they exchange the latest values through shared memory.

Setup order matters for safety. Target streaming is enabled *before* the controller switch,
so ``Robot``'s periodic republishing never sends an old target to the new controller; the
hold pose (free axes, orientation) and the device origin come from the controller's own first
``ee_state`` sample, so the first target is where the robot already is.

**Forces are off unless ``--force``.** Run without it first: the arm should follow the handle
along the interface axis. The force fades in over ``--force-ramp-s``; the felt stiffness at the
handle is ``kv · scale · force_gain`` (printed at start).

    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml
    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --method linear --force
    pixi run -e humble fr3_teleop --conf python/fr3_haptic/configs/teleop.yaml --model-update-hz 1000 --save

Needs the colcon overlay sourced (``arm_client``), franka-server running with
``osc_controller``, and Haply's Inlet service.
"""

from __future__ import annotations

import argparse
import gc
import threading
import time
from dataclasses import asdict

import numpy as np
from arm_client.control.parameters_client import ParametersClient
from arm_client.robot import Robot
from experiment_logger import ExperimentLogger, LoggingConfig
from haptic_teleop import (
    HapticProcess,
    HapticProcessSpec,
    HapticResult,
    HapticStateMailbox,
    HapticStepConfig,
    ModelMailbox,
    PlantMailbox,
    StatusMailbox,
    run_loop,
)
from haptic_teleop.config import FixedMassConfig, LinearConfig, RenderingConfigs, SafetyConfig, TDPAConfig
from haptic_teleop.delay import DelayConfig
from pyrim import InterfaceFrame
from utilities import apply_yaml_config

from .adapters import RobotModelAdapter
from .config import ModelConfig
from .links import Links, LinksConfig
from .plant import FR3Plant, FR3System
from .recorder import RobotStateRecorder
from .session import PROXY_METHODS, CommandLoop, CommandLoopConfig, RimModelLoop

TAG = "[fr3_teleop]"
CONTROLLER_HZ = 1000.0  # osc_controller publishes ee_state / task_wrench every tick
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
    g.add_argument(
        "--model-update-hz",
        type=float,
        default=50.0,
        help="robot -> haptic: robot states passed to the haptic loop [Hz] (the controller measures at 1 kHz); "
        "0 = every controller tick",
    )
    g.add_argument(
        "--rim-update-hz",
        type=float,
        default=None,
        help="robot -> haptic: RIM dynamics model updates [Hz] (proxy-rim); default and maximum = --model-update-hz",
    )
    g.add_argument(
        "--command-hz",
        type=float,
        default=None,
        help="haptic -> robot: targets sent to osc_controller [Hz]; default = --model-update-hz",
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
    g.add_argument("--haptic-cpu", type=int, default=None, help="pin the haptic process to this CPU")
    g.add_argument(
        "--haptic-rt-priority", type=int, default=None, help="SCHED_FIFO priority of the haptic process (needs the privilege)"
    )
    g.add_argument("--feedforward-cap", type=float, default=15.0, help="proxy methods: feedforward clamp [N]")

    g = p.add_argument_group("delays (simulated, per link; 0 = none)")
    g.add_argument("--feedback-delay-ms", type=float, default=0.0, help="robot -> haptic: base delay [ms]")
    g.add_argument("--feedback-jitter-ms", type=float, default=0.0, help="robot -> haptic: jitter [ms]")
    g.add_argument("--command-delay-ms", type=float, default=0.0, help="haptic -> robot: base delay [ms]")
    g.add_argument("--command-jitter-ms", type=float, default=0.0, help="haptic -> robot: jitter [ms]")
    g.add_argument(
        "--jitter-dist",
        choices=("uniform", "normal"),
        default="uniform",
        help="jitter distribution: uniform (base +- jitter) or normal (sigma = jitter), clipped at 0",
    )
    g.add_argument("--delay-seed", type=int, default=None, help="random seed for the delays; default: drawn and logged")

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


def resolve_rates(args: argparse.Namespace) -> tuple[float, float, float]:
    """``(model_update_hz, rim_update_hz, command_hz)`` with the defaults applied.

    ``--model-update-hz 0`` means every controller tick (``CONTROLLER_HZ``). ``--rim-update-hz``
    and ``--command-hz`` default to the model update rate; the RIM model cannot update faster
    than the state it is computed from.

    Raises:
        ValueError: for a negative rate, or a RIM rate above the model update rate.
    """
    if args.model_update_hz < 0:
        raise ValueError(f"--model-update-hz must be >= 0, got {args.model_update_hz}")
    model_update_hz = CONTROLLER_HZ if args.model_update_hz == 0 else float(args.model_update_hz)
    rim_update_hz = model_update_hz if args.rim_update_hz is None else float(args.rim_update_hz)
    command_hz = model_update_hz if args.command_hz is None else float(args.command_hz)
    if rim_update_hz <= 0 or command_hz <= 0:
        raise ValueError(f"--rim-update-hz and --command-hz must be > 0, got {rim_update_hz}, {command_hz}")
    if rim_update_hz > model_update_hz:
        raise ValueError(f"--rim-update-hz ({rim_update_hz:g}) cannot exceed --model-update-hz ({model_update_hz:g})")
    return model_update_hz, rim_update_hz, command_hz


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


def links_config(args: argparse.Namespace) -> LinksConfig:
    """Both links' delays from the flags. A missing seed is drawn here, so it can be logged and
    the run reproduced; the command link uses ``seed + 1``."""
    seed = args.delay_seed if args.delay_seed is not None else int(np.random.SeedSequence().entropy % 2**31)
    return LinksConfig(
        feedback=DelayConfig(
            base_ms=args.feedback_delay_ms, jitter_ms=args.feedback_jitter_ms, jitter_dist=args.jitter_dist, seed=seed
        ),
        command=DelayConfig(
            base_ms=args.command_delay_ms, jitter_ms=args.command_jitter_ms, jitter_dist=args.jitter_dist, seed=seed + 1
        ),
    )


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
    model_update_hz, rim_update_hz, command_hz = resolve_rates(args)
    links_cfg = links_config(args)
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
    plant_box = PlantMailbox(dim)  # plant samples for the haptic process, written by the callbacks
    boxes = [plant_box]
    links = Links(links_cfg)  # simulated delays; a zero-delay link is bypassed
    plant = FR3Plant(
        robot,
        frame,
        ee_state_topic=args.ee_state_topic,
        task_wrench_topic=args.task_wrench_topic,
        sample_period_s=0.0 if args.model_update_hz <= 0 else 1.0 / args.model_update_hz,
        interface_limits=limits,
        record=True,
        mailbox=links.plant_sink(plant_box),
    )
    links.start()  # delivers the delayed samples (and later targets); no thread without a delay
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

    # -- haptic process: method, device and safety live there ------------------------------
    rendering = RenderingConfigs(
        linear=LinearConfig(max_extrapolation=args.linear_max_extrapolation),
        fixed_mass=FixedMassConfig(mass=args.proxy_mass),
        tdpa=TDPAConfig(max_damping=args.tdpa_max_damping),
    )
    safety = SafetyConfig(
        energy_budget_mj=args.energy_budget_mj,
        saturation_abort_ms=args.saturation_abort_ms,
        chatter_hz=args.chatter_hz,
        chatter_deadband_n=args.chatter_deadband_n,
        guard_cooldown_s=args.guard_cooldown_s,
    )
    haptic_state, status = HapticStateMailbox(dim), StatusMailbox()
    boxes += [haptic_state, status]
    model_box = None
    if system is not None:
        model_box = ModelMailbox(system.model.n, dim)
        boxes.append(model_box)
        model_box.write(system.model)
    spec = HapticProcessSpec(
        basis=frame.basis,
        leader_origin=leader_origin,
        method=args.method,
        rendering=rendering,
        stiffness=args.kv,
        damping=args.dv,
        contact_surface=-np.inf if args.contact_surface is None else args.contact_surface,
        step=HapticStepConfig(
            haptic_hz=args.haptic_hz,
            max_force_world=args.max_i3_force / args.force_gain,
            force_ramp_s=args.force_ramp_s,
            guard_cooldown_s=args.guard_cooldown_s,
            free_axis_stiffness=args.free_axis_stiffness,
            free_axis_damping=args.free_axis_damping,
        ),
        safety=safety,
        max_i3_force=args.max_i3_force,
        watchdog_max_age_s=args.plant_max_age_ms * 1e-3,
        watchdog_ramp_s=args.watchdog_ramp_ms * 1e-3,
        device_kwargs={
            "scale": args.scale,
            "axes": axes,
            "signs": tuple(signs),
            "dim": 3,
            "force_gain": args.force_gain,
            "force_enabled": args.force,
            "max_force": args.max_i3_force if args.force else 0.0,
            "write_through": not args.no_write_through,
            "velocity_filter_hz": args.vel_filter_hz,
            "uri": args.uri,
        },
        seconds=args.seconds,
        plant=plant_box,
        model=model_box,
        state_out=haptic_state,
        status_out=status,
        cpu=args.haptic_cpu,
        rt_priority=args.haptic_rt_priority,
    )
    loop = CommandLoop(
        CommandLoopConfig(
            method=args.method, command_hz=command_hz, feedforward_cap=args.feedforward_cap, tool_correction=tool_correction
        ),
        plant=links.command_plant(plant),
        haptic_state=haptic_state,
        status=status,
        dim=dim,
    )
    rim_loop = RimModelLoop(system, links.model_sink(model_box)) if system is not None else None

    np.set_printoptions(precision=4, suppress=True)
    felt_k = args.kv * args.scale * args.force_gain
    print(f"{TAG} {args.method} along {args.interface_axis}; hold pose {hold.position} m, leader origin {leader_origin} m")
    print(
        f"{TAG} rates: haptic {args.haptic_hz:g} Hz (own process) | robot -> haptic: model update {model_update_hz:g} Hz"
        + (f", RIM {rim_update_hz:g} Hz" if rim_loop is not None else "")
        + f" | haptic -> robot: commands {command_hz:g} Hz"
    )
    for name, d in (("feedback (robot -> haptic)", links_cfg.feedback), ("command (haptic -> robot)", links_cfg.command)):
        if d.enabled:
            print(f"{TAG} delay {name}: {d.base_ms:g} ms + {d.jitter_dist} jitter {d.jitter_ms:g} ms (seed {d.seed})")
    gap_ms = links_cfg.worst_delivery_gap_s(model_update_hz) * 1e3
    if links_cfg.feedback.enabled and gap_ms > 0.8 * args.plant_max_age_ms:
        print(
            f"{TAG} WARNING: feedback deliveries can be ~{gap_ms:.0f} ms apart (update period + delay spread); "
            f"the watchdog trips at {args.plant_max_age_ms:g} ms. Raise --plant-max-age-ms or lower the jitter."
        )
    print(f"{TAG} coupling kv = {args.kv:g} N/m, dv = {args.dv:g} N·s/m (world) → felt {felt_k:g} N/m")
    print(f"{TAG} controller {args.controller}: {gains}")
    print(f"{TAG} force {'ON, clamped at %.1f N' % args.max_i3_force if args.force else 'OFF (--force to enable)'}")

    # -- run -------------------------------------------------------------------------------
    recorder = None
    if args.robot_log_hz > 0:
        recorder = RobotStateRecorder(plant.node, args.robot_log_hz, seconds=args.seconds + 10.0)
    haptic = HapticProcess(spec)
    result = HapticResult(error="not started")
    t0 = time.monotonic()
    try:
        # Collector off for the run (on again below). A generation-2 collection of this heap
        # (rclpy, arm_client, JAX) takes ~75 ms and stalls every thread here, so no plant sample
        # reaches the haptic process and its watchdog (50 ms) trips: during the run at high
        # rates, and at once if this collect() ran after haptic.start(). So: collect *before*
        # the haptic loop (and its watchdog) starts. Disable only: gc.freeze() (what gc_paused
        # does) stalls rclpy's executors.
        gc.collect()
        gc.disable()
        haptic.start()  # the child opens and zeroes the device, then its loop starts
        t0 = time.monotonic()
        running = lambda: not loop.stopped and haptic.running  # noqa: E731
        if rim_loop is not None:  # robot -> haptic: the RIM model, on its own thread and rate
            rim_thread = threading.Thread(
                target=run_loop,
                args=(rim_update_hz, rim_loop.step),
                kwargs={"stop_fn": lambda: not running(), "n_ticks": round((args.seconds + 5.0) * rim_update_hz)},
                name="rim_model",
                daemon=True,
            )
            rim_thread.start()
        try:
            # haptic -> robot: the targets, on the main thread
            run_loop(
                command_hz, loop.step, stop_fn=lambda: not running(), n_ticks=round((args.seconds + 5.0) * command_hz)
            )
        except KeyboardInterrupt:
            print(f"\n{TAG} interrupted")
    finally:
        loop.request_stop(loop.shared.stop_reason or "run ended")  # also ends the RIM thread
        gc.enable()
        haptic.stop()
        result = haptic.join()
        loop.shutdown_robot()  # not delayed; drops targets still in flight
        links.stop()
        if recorder is not None:
            recorder.close()
        plant.close()

    # -- report and log --------------------------------------------------------------------
    stop_reason = loop.shared.stop_reason or result.stop_reason
    print(f"{TAG} stopped: {stop_reason}")
    if result.error:
        print(f"{TAG} haptic process error:\n{result.error}")
    for line in (result.rate_summary, result.jitter_summary, f"gc: {result.gc_summary}", f"scheduling: {result.realtime}"):
        if line:
            print(f"{TAG} {line}")
    print(f"{TAG} guard trips: {result.guard_trips}, watchdog trips: {result.watchdog_trips}")
    print(f"{TAG} plant samples: {sum(s.rx_s >= t0 for s in plant.history)} during the run")
    if links_cfg.enabled:
        print(f"{TAG} links: {links.summary()}")
    if recorder is not None:
        print(f"{TAG} robot log ({args.robot_log_hz:g} Hz): {recorder.summary()}")

    metadata = {
        "args": {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(args).items()},
        "controller_gains": gains,
        "command_loop": {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(loop.cfg).items()},
        "delays": {
            "feedback": vars(links_cfg.feedback) if links_cfg.feedback.enabled else None,
            "command": vars(links_cfg.command) if links_cfg.command.enabled else None,
        },
        "rates": {
            "haptic_hz": args.haptic_hz,
            "model_update_hz": model_update_hz,
            "rim_update_hz": rim_update_hz if rim_loop is not None else None,
            "command_hz": command_hz,
            "controller_hz": CONTROLLER_HZ,
        },
        "haptic_step": asdict(spec.step),
        "hold_position": hold.position.tolist(),
        "hold_orientation_xyzw": hold.orientation.as_quat().tolist(),
        "leader_origin": leader_origin.tolist(),
        "stop_reason": stop_reason,
        "haptic_scheduling": result.realtime,
    }
    logger_config = (
        LoggingConfig.to_file(args.output_dir, notes=args.notes or f"fr3 {args.method} kv={args.kv} dv={args.dv}")
        if args.save
        else LoggingConfig.in_memory()
    )
    try:
        with ExperimentLogger(logger_config, metadata=metadata) as logger:
            if result.log is not None:
                result.log.to_logger(logger, "haptic", 1.0 / args.haptic_hz)
            # Plant samples on the local clock relative to the run start (approximately the haptic
            # t = 0). Samples from the setup, before it, are not part of the run.
            for s in plant.history:
                if s.rx_s < t0:
                    continue
                logger.log_sample(
                    "plant",
                    {"x_i": s.x_i, "v_i": s.v_i, "lam": s.lam, "t_s": s.t_s, "ee_position": s.ee_position, "task_force": s.task_force},
                    timestamp_s=s.rx_s - t0,
                )
            for t_s, x, v, f_ff in loop.shared.commands:
                logger.log_sample("command", {"x": x, "v": v, "f_ff": f_ff}, timestamp_s=t_s)
            # Robot state, commanded torques, task error/wrench, EE state; same clock as "plant"
            if recorder is not None:
                recorder.to_logger(logger, t0)
            # Every delivery on a delayed link: when it was sent, how long it took
            for rx_s, delivered_s, delay_s, t_s in links.records.feedback:
                if delivered_s >= t0:
                    logger.log_sample("feedback_link", {"delay_ms": delay_s * 1e3, "sent_s": rx_s - t0, "t_s": t_s}, timestamp_s=delivered_s - t0)
            for sent_s, delivered_s, delay_s in links.records.command:
                if delivered_s >= t0:
                    logger.log_sample("command_link", {"delay_ms": delay_s * 1e3, "sent_s": sent_s - t0}, timestamp_s=delivered_s - t0)
    finally:
        for box in boxes:
            box.close()
            box.unlink()
        robot.shutdown()


if __name__ == "__main__":
    main()
