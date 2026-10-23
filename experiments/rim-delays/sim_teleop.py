r"""fr3_teleop against a simulated FR3 (`sim_plant.SimFR3`): same CLI, same haptic process, links and
command loop, same logged streams, plus a ``sim`` stream with the ground truth (contact force).

Use it to rehearse a condition before the robot, to test the runner and the analysis, and as the
simulated counterpart of every hardware result. ``--device scripted`` (default here) needs no
hardware at all; ``--device inverse3`` puts your hand on a simulated arm.

    pixi run -e humble python experiments/rim-delays/sim_teleop.py --method proxy-rim \
        --interface-axis xyz --contact-surface 0.1 --script press --feedback-delay-ms 425 \
        --command-delay-ms 425 --seconds 30 --save --run-name try_rim_850
"""

from __future__ import annotations

import gc
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from experiment_logger import ExperimentLogger, LoggingConfig
from fr3_haptic.links import Links, LinksConfig
from fr3_haptic.port import RobotPort
from fr3_haptic.session import PROXY_METHODS, CommandLoop, CommandLoopConfig, RimModelLoop
from fr3_haptic.teleop import (
    TAG,
    TWO_PORT,
    absolute_limits,
    build_parser,
    interface_basis,
    links_config,
    log_run_streams,
    resolve_rates,
    script_config,
    scripted_device,
)
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
from pyrim import InterfaceFrame
from utilities import apply_yaml_config

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_plant import TRUTH_COLS, SimConfig, SimFR3, osc_gains  # noqa: E402


def build_sim_parser():
    p = build_parser()
    p.set_defaults(device="scripted", live_hz=0.0, robot_log_hz=0.0, output_dir=str(Path("~/data/franka/rim-delays/sim").expanduser()))
    g = p.add_argument_group("simulated plant")
    g.add_argument("--table-z", type=float, default=None, help="table height [m]; default --contact-surface, else 0.1")
    g.add_argument("--table-stiffness", type=float, default=3.0e4, help="[N/m]")
    g.add_argument("--table-damping", type=float, default=300.0, help="[N·s/m]")
    g.add_argument("--table-friction", type=float, default=0.2, help="Coulomb coefficient")
    g.add_argument("--physics-hz", type=float, default=2000.0, help="[Hz]")
    return p


def main(argv: list[str] | None = None) -> None:
    args = apply_yaml_config(build_sim_parser(), argv)
    axes, signs = args.axes
    basis, contact_axis = interface_basis(args.interface_axis)
    frame = InterfaceFrame(basis)
    dim = frame.dim
    model_update_hz, rim_update_hz, command_hz = resolve_rates(args)
    links_cfg = links_config(args)
    if absolute_limits(args.interface_min, args.interface_max, dim) is not None:
        print(f"{TAG} sim: --interface-min/max are not simulated")
    table_z = args.table_z if args.table_z is not None else (args.contact_surface if args.contact_surface is not None else 0.1)
    if args.contact_surface is None and args.method in PROXY_METHODS:
        args.contact_surface = table_z  # the proxies need the wall; in sim it is known exactly
    script = script_config(args)
    start = tuple(args.start_position) if args.start_position is not None else (0.4, 0.0, 0.2)

    coupling = None if args.method in PROXY_METHODS else (args.kv, args.dv)
    sim_cfg = SimConfig(
        start=start, table_z=table_z, table_stiffness=args.table_stiffness, table_damping=args.table_damping,
        friction=args.table_friction, physics_hz=args.physics_hz,
    )
    plant_box = PlantMailbox(dim)
    boxes = [plant_box]
    links = Links(links_cfg)
    gains = osc_gains(args.interface_axis, coupling, sim_cfg.mass)
    port = RobotPort(max_damping=args.tdpa_robot_max_damping) if args.method in TWO_PORT else None
    sim = SimFR3(sim_cfg, frame, gains, sink=links.plant_sink(plant_box), sample_hz=model_update_hz, port=port)
    # as fr3_teleop: a generation-2 collection stalls every thread here past the watchdog
    gc.collect()
    gc.disable()
    sim.start()
    links.start()
    leader_origin = np.asarray(start, dtype=float)

    device_factory, device_extra, _ = scripted_device(args, leader_origin)

    haptic_state, status = HapticStateMailbox(dim), StatusMailbox()
    boxes += [haptic_state, status]
    model_box = None
    rim_loop = None
    if args.method == "proxy-rim":
        sim.update()
        model_box = ModelMailbox(sim.model.n, dim)
        boxes.append(model_box)
        model_box.write(sim.model)
        rim_loop = RimModelLoop(sim, links.model_sink(model_box), record=args.save)

    spec = HapticProcessSpec(
        basis=frame.basis,
        leader_origin=leader_origin,
        method=args.method,
        rendering=RenderingConfigs(
            linear=LinearConfig(max_extrapolation=args.linear_max_extrapolation),
            fixed_mass=FixedMassConfig(mass=args.proxy_mass),
            tdpa=TDPAConfig(max_damping=args.tdpa_max_damping),
        ),
        stiffness=args.kv,
        damping=args.dv,
        contact_surface=-np.inf if args.contact_surface is None else args.contact_surface,
        contact_axis=contact_axis,
        step=HapticStepConfig(
            haptic_hz=args.haptic_hz,
            max_force_world=args.max_i3_force / args.force_gain,
            force_ramp_s=args.force_ramp_s,
            guard_cooldown_s=args.guard_cooldown_s,
            free_axis_stiffness=args.free_axis_stiffness,
            free_axis_damping=args.free_axis_damping,
        ),
        safety=SafetyConfig(
            energy_budget_mj=args.energy_budget_mj,
            saturation_abort_ms=args.saturation_abort_ms,
            chatter_hz=args.chatter_hz,
            chatter_deadband_n=args.chatter_deadband_n,
            guard_cooldown_s=args.guard_cooldown_s,
        ),
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
            "max_force": args.max_i3_force if (args.force or script is not None) else 0.0,
            "write_through": not args.no_write_through,
            "velocity_filter_hz": args.vel_filter_hz,
            "uri": args.uri,
            **device_extra,
        },
        **({"device_factory": device_factory} if device_factory is not None else {}),
        seconds=args.seconds,
        plant=plant_box,
        model=model_box,
        state_out=haptic_state,
        status_out=status,
        cpu=args.haptic_cpu,
        rt_priority=args.haptic_rt_priority,
    )
    loop = CommandLoop(
        CommandLoopConfig(method=args.method, command_hz=command_hz, feedforward_cap=args.feedforward_cap),
        plant=links.command_plant(sim),
        haptic_state=haptic_state,
        status=status,
        dim=dim,
    )
    print(
        f"{TAG} SIM {args.method} on {args.interface_axis}, table z = {table_z:g} m, "
        f"delays: feedback {LinksConfig.label(links_cfg.feedback)}, command {LinksConfig.label(links_cfg.command)}"
    )

    haptic = HapticProcess(spec)
    result = HapticResult(error="not started")
    t0 = time.monotonic()
    try:
        drain = links_cfg.drain_s()
        if drain > 0:
            time.sleep(drain)
        haptic.start()
        t0 = time.monotonic()
        running = lambda: not loop.stopped and haptic.running  # noqa: E731
        if rim_loop is not None:
            threading.Thread(
                target=run_loop,
                args=(rim_update_hz, rim_loop.step),
                kwargs={"stop_fn": lambda: not running(), "n_ticks": round((args.seconds + 5.0) * rim_update_hz)},
                name="rim_model",
                daemon=True,
            ).start()
        run_loop(command_hz, loop.step, stop_fn=lambda: not running(), n_ticks=round((args.seconds + 5.0) * command_hz))
    except KeyboardInterrupt:
        loop.request_stop("interrupted")
    finally:
        loop.request_stop(loop.shared.stop_reason or "run ended")
        gc.enable()
        haptic.stop()
        result = haptic.join()
        loop.shutdown_robot()
        links.stop()
        sim.close()

    stop_reason = loop.shared.stop_reason or result.stop_reason
    print(f"{TAG} stopped: {stop_reason}; guard trips {result.guard_trips}, watchdog trips {result.watchdog_trips}")
    if result.error:
        print(f"{TAG} haptic process error:\n{result.error}")
    if result.rate_summary:
        print(f"{TAG} {result.rate_summary}")

    metadata = {
        "sim": True,
        "sim_config": asdict(sim_cfg),
        "args": {k: (list(v) if isinstance(v, tuple) else v) for k, v in vars(args).items()},
        "controller_gains": {"K": gains[0].tolist(), "D": gains[1].tolist()},
        "delays": {
            "feedback": LinksConfig.describe(links_cfg.feedback),
            "command": LinksConfig.describe(links_cfg.command),
        },
        "rates": {"haptic_hz": args.haptic_hz, "model_update_hz": model_update_hz,
                  "rim_update_hz": rim_update_hz if rim_loop else None, "command_hz": command_hz},
        "script": asdict(script) if script is not None else None,
        "hand": asdict(device_extra["hand"]) if device_extra.get("hand") is not None else None,
        "interface_basis": basis.tolist(),
        "contact_axis": contact_axis,
        "contact_surface": args.contact_surface,
        "table_z": table_z,
        "hold_position": list(start),
        "leader_origin": leader_origin.tolist(),
        "stop_reason": stop_reason,
        "t0_mono_s": t0,  # time.monotonic() at the run start: the haptic log's mono_s minus this is run time
    }
    config = (
        LoggingConfig.to_file(args.output_dir, run_name=args.run_name, notes=args.notes or f"sim {args.method}")
        if args.save
        else LoggingConfig.in_memory()
    )
    try:
        with ExperimentLogger(config, metadata=metadata) as logger:
            if result.log is not None:
                result.log.to_logger(logger, "haptic", 1.0 / args.haptic_hz)
            log_run_streams(
                logger,
                t0,
                commands=loop.shared.commands,
                targets=sim.targets,
                links=links,
                model_history=rim_loop.history if rim_loop is not None else (),
            )
            truth = sim.truth
            for row in truth[truth[:, 0] >= t0]:
                logger.log_sample("sim", dict(zip(TRUTH_COLS[1:], row[1:])), timestamp_s=row[0] - t0)
            if port is not None and port.history is not None:
                for t_s, e_in, e_out, remote_in, alpha in port.history:
                    if t_s >= t0:
                        logger.log_sample("robot_port", {"energy_in": e_in, "energy_out": e_out,
                                                         "remote_energy_in": remote_in, "damping": alpha},
                                          timestamp_s=t_s - t0)
    finally:
        for box in boxes:
            box.close()
            box.unlink()


if __name__ == "__main__":
    main()
