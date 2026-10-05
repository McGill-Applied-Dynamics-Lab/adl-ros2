# fr3_haptic — plan

Run `haptic_teleop` (adl-python) on the real FR3. `fr3_haptic` replaces `rim_teleop`
and supplies the part adl-python deliberately leaves to the harness: the ROS2 plant
adapter, the robot command path and robot-side safety.

**Goal:** a real-arm method comparison study first (ZOH, linear, TDPA, RIM, fixed-mass),
then harden one method into a daily-use teleop.

## Decisions

| Topic | Decision |
|---|---|
| Follower coupling (ZOH / linear / TDPA) | Server side: `osc_controller` at 1 kHz with `inertia_decoupling: false`, z gains = coupling `K`, `D`; x/y stiff; orientation + posture held. Same role as `PointCoupling` in adl-python example 04. |
| RIM / fixed-mass | Proxy in the haptic process (`RIMRendering`). Proxy position → `target_pose`, `−λ` → `target_wrench`. |
| `rim_teleop` | Rebuilt on `haptic_teleop` as `fr3_haptic`. Keep the Pinocchio model adapter, safety gates, tool-tip correction. Drop the duplicated `pyrim` fork, logger, loops. |
| DoF | 1-DoF z first (table contact, all five methods comparable). 3-DoF translation later. |
| Process | One Python process (pixi `humble`, Python 3.12): 1 kHz haptic thread talking to the Inverse3 directly (`Inverse3Sync`), a plant thread publishing targets at a configurable rate, an rclpy executor writing robot samples into a mailbox. |
| Timing | Both PCs use `systemd-timesyncd` (NTP, ~ms). Do not compare stamps across machines: header stamps are used for ordering and staleness only; latency is measured client-side as round trip. |
| adl-python | Git submodule at `external/adl-python` (branch `dev`), installed editable. |

## Architecture

The overview diagram is on the docs site: `docs/franka-client/haptic-teleop.md`. Every link in detail:

| From → to | What | Rate | Transport |
|---|---|---|---|
| Inverse3 ⇄ haptic thread | handle position, velocity / force | 1 kHz | Inlet websocket (localhost), on the haptic thread |
| haptic thread → plant loop | leader `(x_l, v_l)` on the interface axis | written 1 kHz, read at `plant_hz` | mailbox `session.leader` |
| plant loop → `osc_controller` | target pose + twist (+ feedforward wrench for proxy methods) | `plant_hz` | DDS, client → franka-pc |
| `osc_controller` → fr3_plant node | EE pose + twist, task force (same tick) | 1 kHz | DDS, franka-pc → client |
| fr3_plant node → haptic thread | `FR3PlantSample (x_i, v_i, λ, t_s)` | `sample_hz` (≤ 1 kHz) | mailbox `plant.sample` |
| plant loop → haptic thread | `DynModel` (proxy-rim) | `plant_hz` | mailbox `session.model` |
| Robot node → plant loop | `q`, `dq` for the Pinocchio model | broadcaster rate | `Robot` properties |
| `osc_controller` ⇄ FR3 | joint torques / robot state | 1 kHz | libfranka FCI |

Timing notes:
- The only jitter-critical loop on the client is the haptic thread, and it never goes through ROS.
- The four client threads share one GIL; whether the executors stretch haptic ticks is unmeasured
  (Phase 3 step 0, or offline with a fake device). If they do, split the haptic thread into its own
  process with no rclpy and exchange the three mailboxes through shared memory.
- Both ROS links cross the network: DDS adds ~0.1–0.5 ms, small next to the plant period.

`λ` sign: `task_wrench` is the force applied to the robot, `K(x_l − x_i) + D(v_l − v_i)`
along the axis. `PlantState.lam` is the operator-should-feel sign, i.e. its negative.

## Phases

### Phase 0 — wiring
- [x] Worktree `adl-ros2.worktrees/fr3-haptic`, branch `feat/fr3-haptic`.
- [x] adl-python submodule at `external/adl-python`.
- [x] pixi: `pyrim`, `haptic_teleop`, `experiment_logger`, `utilities` from the submodule; remove `python/pyrim`.
- [x] Rename `python/rim_teleop` → `python/fr3_haptic` (legacy orchestrator ported to adl-python `pyrim`; 15/15 tests pass).
- [x] adl-python: declare `websockets` in `haptic_teleop` (submodule branch `feat/fr3-haptic`, not pushed); adl-python tests pass in the `humble` env (Python 3.12, 312 tests).
- [ ] `CLAUDE.md`: update the RIM section for `fr3_haptic` (deferred to Phase 2; `main` has uncommitted edits there).

### Phase 1 — FR3 plant adapter (`fr3_haptic/plant.py`)
- [x] `FR3Plant` state side: `/fr3/osc/ee_state` + `/fr3/osc/task_wrench` paired by identical stamp into
  `FR3PlantSample` (satisfies `haptic_teleop.PlantState`); mailbox `(updates, sample)`; `x_i`, `v_i` projected on
  the interface frame; `lam = −project(task force)`; `t_s` = controller stamp; `rx_s` = local receive time.
  Optional `sample_period_s` decimation to emulate a slower plant.
- [x] `FR3Plant` command side: `aim(x_l, v_l)` → `target_pose` + `target_twist` (free axes and orientation from
  `hold_pose`); `command(x, v, f_ff)` adds `target_wrench`; `freeze()`; optional interface limits.
- [x] `FR3System(pyrim.SystemInterface)` over `RobotModelAdapter`.
- [x] `Robot.set_target_streaming()` + `Robot.publish_target()` (arm_client): pause the republish timers and
  publish immediately; streamed targets are stored, so leaving streaming does not resume older targets.
- [x] `StalenessWatchdog`: force gain ramps to 0 when the sample age exceeds `max_age_s` (latching by default).
- [x] franka-server `c63df93`: `osc_controller` publishes `ee_state` (`nav_msgs/Odometry`, same tick as `task_wrench`).
- [ ] Hardware check (Phase 3 step 0): `ros2 topic hz /fr3/osc/ee_state` ≈ 1 kHz, pairing rate on the client,
  executor load with two 1 kHz subscriptions in Python.

Moved to Phase 2: set the `osc_controller` interface gains from the same config that builds the rendering
method (so the server-side `K`, `D` cannot drift from what ZOH assumes), with decoupling off and wide limits.
`FR3System` vs `FR3Plant` interface point: Pinocchio EE / tool tip vs libfranka `kEndEffector` — must match.

### Phase 2 — harness (`fr3_haptic/session.py`, `fr3_haptic/teleop.py`, `fr3_teleop` entry point)
- [x] `TeleopSession`: haptic tick (device → method → force × start ramp × watchdog gain, guard cooldown,
  optional free-axis handle spring, passivity observer + guard, `TickLog`) and plant tick (model update for
  `proxy-rim`, `aim` for coupling methods, `command(proxy + tool_correction, v, −f clamped)` for proxy methods,
  freeze + end of run on a watchdog trip). Hardware injected; tested with fakes.
- [x] `fr3_teleop` CLI (+ `--conf configs/teleop.yaml`): safe setup order — streaming on *before* the controller
  switch; hold pose and device origin from the controller's own first `ee_state`; controller gains set from the
  same `kv`, `dv` that build the method (coupling methods), on top of `configs/osc_teleop.yaml`; forces off
  unless `--force`. Logs `haptic` (every tick), `plant` (every sample), `command` streams + run metadata.
- [x] `proxy-rim`: leader origin shifted onto the model interface point; target shifted back by
  `tool_correction = x_i(ee_state) − x_i(model)`.
- [x] Measured (local DDS, isolated domain): rclpy `MultiThreadedExecutor` delivers ~14 Hz of a 1 kHz topic;
  `FR3Plant` now spins its own node on a `SingleThreadedExecutor` → 1000 Hz paired, gaps p99 1.1 ms.
- [ ] Deadman (Inverse3 has no button; keyboard or foot pedal?).
- [ ] `CLAUDE.md` RIM section → `fr3_haptic` (deferred: `main` has uncommitted edits there).

### Phase 3 — bring-up and system ID (hardware)
0. Without `--force`: `ros2 topic hz /fr3/osc/ee_state`; the arm follows the handle; check the printed
   plant age and haptic rate; GIL load of haptic loop + plant spin + `Robot` executor in one process.
1. Loop jitter, end-to-end `T_eff` (leader → target → measured), handle `b` → `K_max = 2b/T_eff`.
2. ZOH, low `K`, free space. 3. Linear, TDPA. 4. RIM, fixed-mass against the table.

### Phase 4 — study
- Port `experiments/virtual_coupling/exp10_method_comparison` to the real arm.

### Phase 5 — daily-use teleop
- 3-DoF translation, clutch / indexing, orientation hold, `Robot` API integration.

## Dependencies on the controller work (`franka-server`, branch `feat/osc-controller`)

- `osc_controller` per-axis gains in N/m (`inertia_decoupling: false`) — exists.
- **To add:** `/fr3/osc/ee_state` — measured EE pose + twist published from the same tick and stamp as
  `task_wrench`, so one `PlantSample` comes from one instant.
- `limits.max_position_error`, force limits and `delta_tau_max` change the coupling the methods assume:
  set them wide for teleop and log them with every run.

## Port notes (Phase 0)

API differences between the former local `pyrim` fork and adl-python `pyrim`, and how they were handled:

- `RIMIntegrator(vel_filter_alpha=...)` removed (adl-python filters leader velocity in the device). The legacy
  orchestrator and bench now apply `fr3_haptic.filters.LowPassFilter` before `add_leader_state` — same behavior.
- `RIMIntegrator.contact_surface` setter removed. Assigning it silently created a new attribute and disabled the
  wall; the bench now builds the integrator once the wall is known. Watch for this when porting other callers.
- `DynModel.tau_ext` → `f_ext`, and the convention is now "gravity belongs in `f_ext`". `RobotModelAdapter` keeps
  `c = nle − g` and `f_ext = None`, i.e. it models the gravity-compensated (libfranka) robot. See open questions.
- The integrator step now subtracts `z_i`. `RobotModelAdapter` produces `z_i` through `RIMCalculator`, so RIM
  output on the robot can differ slightly from runs with the old fork.
- `FixedMassCalculator` is gone from `pyrim`; kept locally in `fr3_haptic/fixed_mass.py` for the legacy
  orchestrator. It re-seeds the proxy from the plant at every model update, whereas
  `haptic_teleop.FixedMassProxyRendering` seeds once from the leader — a different ablation condition.

## Open questions

- `Robot` itself spins on a `MultiThreadedExecutor`: its own subscriptions (joint states, pose, wrench) may be
  far below their publish rate too. Worth measuring for the rest of `arm_client`.

- Fixed-mass condition for the study: adl-python's (seed once from the leader) or the legacy one (re-seed from the plant)?
- `DynModel` for the real arm: gravity-compensated plant (`f_ext` = measured external torque only), or the
  open-/closed-loop split described in adl-python `NEWTON_PLAN.md`?

- Where the Inverse3 is plugged in for experiments (client PC assumed) and the network path to franka-pc.
- PTP between the two PCs, if cross-machine latency must be measured from stamps.
