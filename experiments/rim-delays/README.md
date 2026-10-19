# RIM under communication delay

Demonstrate, without the user study, that the RIM inertial proxy renders hard contact
transparently and stably under space-relevant delays where ZOH and TDPA do not. Real FR3 +
Inverse3 pipeline (`fr3_teleop`), 3-DoF translation, table contact on z.

| Factor | Levels |
|---|---|
| Method | `proxy-rim` (ours), `zoh`, `tdpa-zoh` |
| Round-trip delay | 0, 50 ms (local surface link), 850 ms (orbit-to-ground relay), 2.6 s (Earth–Moon); split evenly, `feedback = command = rtt / 2` |
| Task | `press` (approach, press 1 cm into the table, hold, retract; ×3), `circle` (two 5 cm circles on the table) |
| Operator | programmed: the script, held by a simulated hand (`--hand`); you on the Inverse3 for the demo/video |

The programmed operator makes every condition see the same intent. With `--hand` the loop is
closed through a mass-spring-damper grip, so a late force lets the hand overshoot into the table and
a non-passive rendering can go unstable — the effects the abstract claims, measured repeatably.

## Files

| File | Role |
|---|---|
| `conditions.yaml` | The design: methods, delays, tasks, repeats, flags common to every run |
| `run_conditions.py` | Runs every cell (shuffled within each repeat, resumable) on `--backend sim` or `robot` |
| `sim_teleop.py`, `sim_plant.py` | `fr3_teleop` against a simulated FR3 (point mass with the FR3 task inertia, OSC-like PD, stiff table) — same haptic process, links and command loop |
| `analyze.py` | Metrics per run, summary table, figures (`<session>/analysis/`) |

Runs are saved under `~/data/franka/rim-delays/<backend>/<session>/` (`output_dir` in `conditions.yaml`).

The device side lives in `fr3_haptic`: `--interface-axis xyz`, `--device scripted`, `--script press|circle`,
`--hand`, `--run-name` (`python/fr3_haptic/src/fr3_haptic/scripted.py`).

## Commands

```bash
# Simulation (no hardware): the whole design, then the analysis
pixi run -e humble python experiments/rim-delays/run_conditions.py --backend sim --session sim-v1
pixi run -e humble python experiments/rim-delays/analyze.py ~/data/franka/rim-delays/sim/sim-v1

# One condition by hand, in sim
pixi run -e humble python experiments/rim-delays/sim_teleop.py --conf python/fr3_haptic/configs/teleop.yaml \
    --method proxy-rim --interface-axis xyz --contact-surface 0.1 --device scripted --script press --hand \
    --feedback-delay-ms 425 --command-delay-ms 425 --seconds 40

# Robot session (asks before every run; --only-* to do a subset)
pixi run -e humble python experiments/rim-delays/run_conditions.py --backend robot --session day1 --only-rtt 0
pixi run -e humble python experiments/rim-delays/analyze.py ~/data/franka/rim-delays/robot/day1
```

## Hardware bring-up (in this order; stop at the first surprise)

1. **Table height.** Jog the EE (with the tool used for the runs) onto the table, read
   `robot.end_effector_pose.position[2]`; put it in `conditions.yaml` `common.contact_surface`.
   For RIM it must be at the *model* interface point (`--tool-tip-offset` if a tool is mounted).
2. **3-DoF free space, no delay, no force**, Inverse3, `--method zoh --interface-axis xyz`: the arm
   follows the handle in x, y, z; the x/y hold gains from `osc_teleop.yaml` are now coupling gains.
3. Same with `--method proxy-rim`, still no `--force`, no contact: robot follows the proxy.
4. **Scripted, no hand, no delay** (`--device scripted --script press`) for each method: the arm
   presses 1 cm into the table; check the peak table force in the plot before anything else.
5. **Scripted with `--hand`**, no delay, each method. Then 50 ms, 850 ms, 2.6 s — ZOH first at each
   delay with the guards on (`--energy-budget-mj`, `--chatter-hz`); the hand model can drive a
   non-passive method unstable, on the real arm too.
6. `circle`, same ladder.
7. You on the Inverse3 with `--force`: the demo and the video, same conditions.

`teleop.yaml` sets 250 ms delays: the runner always passes both delays explicitly; by hand, do too.

## What a result here can and cannot claim

- The table is known to RIM a priori (`--contact-surface`); it is not detected from the robot.
  Say so. The model (`M_eff`, `z_i`, `f_eff`) is what crosses the delayed link.
- `tdpa-zoh` is a one-port passivity observer/controller at the device, not the two-port,
  delay-aware TDPA (Artigas et al.). It is a fair stability baseline, not the strongest one.
- The programmed hand is a grip model, not a person: rankings should transpose, absolute
  numbers (overshoot, peak force) scale with its stiffness and mass. Report its parameters.
- Simulated delays are constant (no jitter) unless `--feedback-jitter-ms`/`--command-jitter-ms`.
