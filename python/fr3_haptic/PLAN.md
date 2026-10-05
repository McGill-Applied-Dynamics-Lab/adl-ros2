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

## Data flow

```
Inverse3 ──read──► haptic thread (1 kHz) ── method.add_leader_state / step / haptic_force ──► Inverse3
                        │  leader (x, v)                   ▲ PlantSample (x_i, v_i, λ, t_s)
                        ▼                                  │ DynModel (RIM)
                  plant thread (rate R) ── target_pose / target_twist [/ target_wrench] ──► osc_controller (1 kHz, franka-pc)
                        ▲                                                                   │
                  rclpy mailbox ◄──────────── /fr3/osc/ee_state + /fr3/osc/task_wrench ◄────┘
```

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

### Phase 1 — FR3 plant adapter
- `FR3Plant` state side: mailbox `(updates, PlantSample)`; `x_i`, `v_i` from EE state projected on the
  interface axis (`pyrim.InterfaceFrame`); `λ` from `task_wrench`; `t_s` from the header stamp.
- `FR3Plant` command side: `aim(x_l, v_l)` → `target_pose` + `target_twist` (other axes at home, orientation fixed);
  `command(x_proxy, f_ff)` adds `target_wrench` for RIM / fixed-mass.
- `FR3System(pyrim.SystemInterface)` wrapping `RobotModelAdapter` → `DynModel`.
- Direct publishers: `Robot` needs a streaming mode that disables its 100 Hz / 50 Hz republish timers.
- Staleness watchdog: stale plant sample → ramp haptic force to zero, freeze the robot target.

### Phase 2 — harness
- One entry point mirroring adl-python `examples/04_i3_newton_fr3_coupling.py`: `Inverse3Device.settle_and_zero`
  (origin at TCP), `RenderingConfigs.build`, `gc_paused`, force ramp, `SafetyMonitor`, deadman.
- Logging: `TickLog` (haptic) + plant streams via `experiment_logger`. YAML config via `--conf`.

### Phase 3 — bring-up and system ID (hardware)
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

- Fixed-mass condition for the study: adl-python's (seed once from the leader) or the legacy one (re-seed from the plant)?
- `DynModel` for the real arm: gravity-compensated plant (`f_ext` = measured external torque only), or the
  open-/closed-loop split described in adl-python `NEWTON_PLAN.md`?

- Where the Inverse3 is plugged in for experiments (client PC assumed) and the network path to franka-pc.
- PTP between the two PCs, if cross-machine latency must be measured from stamps.
