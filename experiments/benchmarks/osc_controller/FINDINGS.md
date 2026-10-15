# osc_controller: findings (2026-10-06 to 10-08)

State of `osc_controller` (franka-server) on the FR3 after the first hardware tests, and where to
start for the open problem: **joint friction**. Measurements come from the benchmarks in this
folder (see [README.md](README.md)); runs still on disk are under `results/tracking/`.

Working preset: `configs/controllers/osc/default.yaml`, `control.inertia_decoupling: true`,
`k_pos 650`, `k_rot 3000/3000/5000`, `damping_ratio_pos/rot 0.7`, `d_* = -1` (from the ratio).

## Summary

| Problem | Status | Cause | Fix |
|---|---|---|---|
| 50-60 Hz buzz, mostly joint 2 | **solved** | limit cycle sustained by the torque rate limiter (1 Nm/tick) when damping is critical | damping ratio 0.7 (`gains.damping_ratio_*`, franka-server `feat/osc-damping-ratio`) |
| Orientation / position offset that flips with the direction of motion | open | Coulomb joint friction against a finite task stiffness | friction compensation (proposed below) |
| 1-3 Hz wobble during slow motions | open | stick-slip of a joint whose velocity hovers around zero (j2 in a vertical plunge) | same |
| Yaw error up to ~150 mrad on the figure eight | open | stick-slip of joint 7, very low yaw stiffness (tiny wrist inertia) | same |
| Steady-state error after steps (3-7 mm) | open | static friction | same, or a clamped integral term |

## Solved: 50-60 Hz limit cycle on the torque rate limiter

- Symptom: audible buzz; 45-66 Hz component on joints 1-4, largest on j2 (~±5 Nm commanded).
- Evidence: commanded torque and measured joint velocity on j2 fully coherent at ~66 Hz
  (coherence 1.00, phase ~180°: the damping term). Whenever it ran, j2's commanded torque was at
  `limits.delta_tau_max` (1 Nm/tick, the FR3 1000 Nm/s limit) on 53-91 % of ticks; whenever it did
  not, 0 %. Bistable: off (0.03 Nm) or on (~3 Nm), triggered by a transient (step, fast motion),
  sustained after the arm stops — a plain hold test never shows it.
- Raising `delta_tau_max` is not an option (hardware limit). Filtering joint velocity
  (`filter.joint_velocity 0.5`) made it worse (added lag).
- Fix: damping below critical. Figure eight at k_pos 500 / k_rot 1250/1250/2500: j2 chatter
  1.04 → 0.27 Nm, ticks at the limit 3.2 % → 0.01 %, tracking unchanged. Plunge at
  k_rot 3000/3000/5000 with `damping_ratio_rot` 1.0 → 0.7: j2 chatter 1.98 → 0.19 Nm, at limit
  25 % → 0.02 % (`results/tracking/20261008_125649_plunge` vs `..._130049_plunge_zr07`).
- Diagnostic to keep an eye on: `tau_rate_limited_pct` / `dtau_p99_Nm` in every benchmark, and
  `settled_*` chatter in `step_response.py`. More than ~1 % of ticks at the limit is a warning.

## Open: joint friction

### Evidence

**Vertical plunge** (20 cm down and up in 10 s from home, 63 mm/s peak;
`tracking.py --shape plunge --depth 0.2 --period 10 --cycles 1`, run `20261008_130049_plunge_zr07`):

- Position error 7.2 mm RMS, almost all in **x** (the plunge is in z); orientation error 8.2 mrad
  RMS, almost all **pitch**. x and pitch errors are anti-correlated (−0.90 to −0.95): one
  coupled "wrist swing" direction.
- Both sit at a constant level that **flips sign when the motion reverses** (pitch ≈ −10 mrad
  going down, +10 mrad going up; x ≈ ∓8 mm) and does not scale with speed → Coulomb friction, not
  lag or viscous damping.
- A 1-3 Hz wobble rides on it (peak ~1.2 Hz, 1.0 mrad RMS in pitch). **Joint 2 stick-slips**: its
  mean velocity is ~3 mrad/s (it barely takes part in a vertical plunge), but it oscillates by
  ~20 mrad/s around zero, crosses zero 170-285 times in 7 s, and is nearly stopped (|dq| < 0.01)
  25 % of the time. Correlation of band-passed pitch error with j2 velocity 0.4-0.5, j4 0.5-0.6;
  j4 and j6 do the motion and move steadily. (`wobble.py` reproduces these numbers.)

**Figure eight** (xy, ±20 cm × ±8 cm, 8 s period, 200 mm/s peak; run `20261008_121759`):

- Yaw error ±140-150 mrad, near zero at the centre crossings, largest at the ends; x/y errors
  ≤ 20 mrad.
- **Joint 7 stick-slips**: its velocity is flat at zero for 1.5-2.5 s, then breaks loose to
  0.4 rad/s, while its commanded torque plateaus at ≈ 0.7 Nm → static friction of j7 ≈ 0.7 Nm.
- Position error 9.7 mm RMS, lag ~60 ms (twist feedforward only; acceleration feedforward unused).

### Why the gains cannot fix it

Task-space inertia Λ at home (`fr3_hand_tcp`, base-aligned; Pinocchio on
`franka_rim/models/fr3_franka_hand.urdf`):

| | x | y | z | roll | pitch | yaw |
|---|---|---|---|---|---|---|
| Λ_ii | 11.4 kg | 4.6 kg | 5.4 kg | 0.17 kg m² | 0.53 kg m² | **0.002 kg m²** |

With `inertia_decoupling: true` the physical stiffness is Λ·k: at k_rot 2000/2000/2500 that is
~340 / 1060 / **5** Nm/rad, so 0.7 Nm of j7 friction alone needs ~140 mrad of yaw error. Yaw Λ
also varies by ~100× with the configuration (0.002-0.2 nearby).

Stiffness cannot buy this back: K and Λ fix the bandwidth ω = √(K/Λ) (= √k in decoupled units),
independent of the parametrisation (decoupled or not). 50 Nm/rad of yaw needs ω ≈ 160 rad/s
(25 Hz), close to where the rate-limiter limit cycle lives, and the extra damping that comes with
it is what triggers that cycle. Error only shrinks proportionally; stick-slip stays. Measured:
raising k_rot to 3000/3000/5000 did not change the plunge errors.

### Proposed fix: friction compensation in osc_controller

Per-joint feedforward added to the commanded torque:

    tau_f,i = f_c,i * tanh(qd_d,i / v_s)            (optionally + f_v,i * qd_d,i)

- Use the **desired** joint velocity `qd_d = J^+ v_target` (target twist, already streamed with
  `feedforward.twist`), not the measured one: the measured velocity is noisy around zero, which is
  exactly where stick-slip happens, and a sign() on it chatters. Fall back to a small, heavily
  smoothed measured term only if there is no target twist.
- Parameters: `friction.enabled`, `friction.coulomb` (7 values, Nm), `friction.v_smooth` (rad/s,
  ~0.01-0.02), optional `friction.viscous`. Runtime-settable, defaults off.
- Keep it inside the rate limit: it adds torque steps at velocity reversals; the tanh width sets
  how fast. Check `tau_rate_limited_pct` after enabling it.
- Starting values: j7 ≈ 0.7 Nm (measured breakaway); others unknown, typically 0.3-1 Nm on the
  FR3. Identify them rather than guess (below).

Identification: slow constant-velocity sweeps of one joint at a time under the JTC (or a joint
impedance controller), forward and back; Coulomb friction ≈ half the difference of the measured
joint torque between the two directions (gravity cancels); repeat at 2-3 speeds for the viscous
part. `FrankaRobotState` already gives tau_J and dq at 1 kHz.

Alternative / complement: a clamped, slow integral term on the task error. Removes the static
offsets (steady-state step error too) but has to unwind at each reversal (lag), and does nothing
for stick-slip. Friction feedforward first.

### Validation plan

Same gains, friction compensation off vs on:

```bash
cd experiments/benchmarks/osc_controller
python tracking.py --shape plunge --depth 0.2 --period 10 --cycles 1 --config default --tag plunge_nofric
python tracking.py --shape plunge --depth 0.2 --period 10 --cycles 1 --config default --set friction.enabled=true --tag plunge_fric
python tracking.py --config default --tag eight_nofric
python tracking.py --config default --set friction.enabled=true --tag eight_fric
python compare.py tracking --last 4
python plots.py --compare results/tracking/*_plunge_nofric results/tracking/*_plunge_fric
```

Targets: plunge x/pitch offset (7 mm / 8 mrad RMS today), j2 time near zero velocity and the
1-3 Hz wobble (`python wobble.py results/tracking/<run>/data.npz`: band-pass 1-6 Hz, correlate with
each joint's velocity), figure-eight yaw (±150 mrad today), j7 dwell at zero velocity. Rate-limit
share must stay < 1 %.

## Other open items

- **Communication-constraint violations** crashed franka-server ~4 times during these tests.
  Not investigated. Suspect first: `log.enabled: true` + `log.controller_parameters: true` in the
  preset print Kp/Kd/Λ from the 1 kHz update loop (throttled 1 s, but formatting and logging in
  the RT thread). Try with logging off; then look at franka-pc's RT setup.
- **Target stream gaps**: the 500 Hz client stream usually stays < 11 ms, one run had a 30 ms gap.
  GC is paused while streaming; source not identified.
- **Steady-state error after steps** (3-7 mm at k_pos 400-650): friction, see above.
- **Lag on the figure eight** (~60 ms): acceleration feedforward (`/fr3/target_accel`) unused.
- **Gain bounds**: `gains.k_rot_*` is bounded at 5000 in `osc_controller.yaml`.
- **End-effector frame**: the controller tracks libfranka `kEndEffector`; `/fr3/current_pose`
  matches it to 0.01 mm / 0.01° with the current hand setup (checked 2026-10-06), but nothing
  enforces it.

## Code involved

- franka-server `feat/osc-damping-ratio` (`0d18f49`): `gains.damping_ratio_pos/rot`.
- adl-ros2 `feat/osc-controller`: this benchmark suite; `Robot.stream_cartesian_traj` and
  `examples/07b_follow_trajectory_osc.py` (client-side trajectory streaming for osc_controller);
  `Robot` single-threaded executor, `wait_for_future`, target re-seeding on controller switch.
