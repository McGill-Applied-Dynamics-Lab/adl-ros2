# Delayed RIM Teleoperation — Talk Experiment Plan

Oct 23, 2026 · @Charles

## Goal and message

The experiments must show one claim: with a local model, transparency is set by the fidelity of the model, not by the delay. Position–position (P-P) degrades with the round-trip delay T; the model-based methods do not, and the better the robot model, the smaller the residual.

The talk angle is the impact of the robot model. No user study: scripted-leader runs carry the quantitative claims, and device-in-the-loop runs with the Inverse3 cover stability, which depends on the hand holding the handle.

TDPA is left out. One sentence on the slide covers it: TDPA stabilizes P-P but cannot make contact arrive earlier, because the information is not yet on the haptic side.

## Methods and conditions

Four methods, read as a ladder: M1 shows the delay problem, and M3 → M4 → M5 add model fidelity one step at a time.

| Method | Local model on the haptic side | Role in the talk |
| --- | --- | --- |
| M1 — P-P (ZOH) | none | reference: shows the delay problem |
| M3 — God-object | table geometry only | model-based, no dynamics |
| M4 — Fixed-mass proxy | geometry + constant mass m | a plausible mass, not the robot |
| M5 — RIM (arm) | geometry + the arm's reduced dynamics (Λ, z, f\_eff) | the proposed method |

M4 is tuned to its best case: m equals the effective mass along the initial motion direction at the start configuration, nᵀΛ₀n. Any gap to M5 then comes from Λ changing, not from a poorly chosen m.

**M5's inertia: orientation held.** The OSC holds the end effector's orientation, and at the experiments' frequencies (below its rotational modes, ~3.6 Hz and up) the arm moves as if the orientation were fixed: on the E2 circle the free-rotation Λ was ~7× too light along x (radial gap +7.4 N, +1.2 N with the held reduction). M5 therefore reduces with the orientation held (`rim_hold_orientation: true` in `teleop.yaml`, `--no-rim-hold-orientation` for the free reduction), and M4's m is taken from the same held Λ₀ (`tools/lambda_map.py`, `Lambda_held`) so the comparison stays fair. The frequency-dependent version (a 6-D proxy held by the OSC's own orientation gains, free at impacts and held in slow pushes) is future work (M6, FORMULATION.md).

Delay conditions (round trip T, split evenly T\_c = T\_f = T/2): 0, 5g, 850 ms, 2.6 s. Model-based methods are expected to be flat in T, so their free-motion runs need only T = 0 and 2.6 s; M1 needs all four.

## Metrics

The headline metric is the unfelt force e\_F: what the robot's controller exerted when the operator's action took effect, minus what the operator felt. It needs no force sensor, since F\_osc is the OSC's published task\_wrench.

```latex
e_F(t) = f(t) + F_{osc}(t + T_c)
```

It is not circular for M4 and M5, even though they send f\_ff = −f: for them, e\_F reduces exactly to the OSC's PD correction, the force the robot needed beyond what the operator was shown. In static contact F\_osc ≈ −λ\_c, so the same metric measures force matching against the table.

| Method | Expected e\_F in free motion |
| --- | --- |
| M1 | ≈ −K v T, grows with delay |
| M3 | ≈ Λ a: force the robot needed that was never felt |
| M4 | ≈ (Λ − m) a: right on average, wrong by direction and configuration |
| M5 | model error only: joint friction, held terms |

Contact metrics (E3, E6): force-onset latency after the leader crosses z = s; leader penetration at onset; felt-force overshoot and oscillation; robot-side peak table force; e\_F during the hold.

Stability indicators (E5, E6): bounce count, settling time, and RMS force in a band above about 8 Hz.

Energy balance, computed offline from the logs for every method. A negative value means the system generated energy; it gives one method-agnostic number for how active a run was.

```latex
E(t) = -\int_0^t \left( f \cdot v_l + F_{osc} \cdot v \right) d\tau
```

## Scripted-leader experiments

A scripted leader gives every method the identical trajectory, so differences come from the method alone. It cannot show coupled instability; that is what E5 and E6 are for.

### E1 — Free-motion transparency

Shows that model-based transparency is flat in delay and improves with model fidelity.

- **Protocol:** sinusoids of about 5 cm amplitude, along the heaviest and the lightest eigenvectors of Λ at the start configuration. Report the eigenvalue ratio on the slide. M4's m is retuned per direction (nᵀΛ₀n along each), its best case, so E2 and E4 carry M4 vs M5.
- **Conditions:** M1, M3, M4, M5; frequencies 0.5, 1 and 2 Hz; delays as in Methods and conditions.
- **Plots:** RMS e\_F vs T, and RMS e\_F vs frequency.
- **Expected:** M1 grows with T; M3 grows as ω²; M5 stays low. With m retuned per direction, M4 should match M5 in the two directions. On hardware, M5's residual is likely dominated by joint friction at low speed: keep it, it is the honest floor of the method.

### E2 — Horizontal circle (anisotropy)

Shows that a fixed mass cannot follow an inertia that varies with the direction of motion.

- **Protocol:** a horizontal circle at constant speed: radius 10 cm, period 2 s (v ≈ 0.31 m/s, a ≈ 0.99 m/s²), centered at (0.5, 0, 0.25) m. The centripetal acceleration rotates through every direction of the plane.
- **Start position (E2 only):** (0.5, 0, 0.3) m, not the common start: centered on (0.4, 0, 0.16) the near side folds the arm until joint 4 is at −3.03 rad, where libfranka's position-dependent velocity limit stops even a 3 s circle (joint 4 at ~100 % of its limit; a 1.5 s circle aborted with `joint_velocity_violation`). At (0.5, 0, 0.25) the 2 s circle uses ~52 % of the limit. Compute Λ at this start for M4's mass.
- **Conditions:** M4 and M5 (M3 for reference), at T = 0 and 2.6 s.
- **Plots:** e\_F against circle phase; the angle between force and acceleration, for f and for −F\_osc.
- **Expected:** M4's e\_F oscillates at twice the circle frequency while M5 stays flat. With off-diagonal coupling, the robot's force is not parallel to its acceleration; M5 reproduces that sideways component, M4 cannot.

### E3 — Contact onset over approach velocity

Shows when the model matters at contact: model-based vs P-P, and the inertial transient switching on with speed.

- **Protocol:** a vertical approach at constant velocity into the table, a hold at fixed leader depth, then retraction.
- **Conditions:** all methods; approach speeds of 1, 5, 10 and 20 cm/s at T = 850 ms; all four delays at one speed; three repetitions each.
- **Plots:** contact metrics vs approach speed; one time series at 850 ms (leader z, felt force, and robot −F\_osc shifted by T\_c) for M1 vs M5.
- **Expected:** at low speed all methods converge. As speed rises, M1's onset kick grows with v T, and the model-based transients scale with momentum (Λ v for M5, m v for M4, none for M3). Static force is the same across M3–M5 (K × depth), so E3 carries model-based vs P-P and E1/E2 carry which model.

### E4 — Reach sweep (configuration change)

Shows that a mass tuned at one configuration is wrong elsewhere.&#32;

- **Protocol:** a slow scripted line from near the base outward, with a small sinusoidal dither superimposed so inertial force is present throughout.
- **Conditions:** M4 (tuned at the start of the line) and M5, at T = 0 and 2.6 s.
- **Plots:** e\_F and the eigenvalues of Λ against position along the path.
- **Expected:** M4 is right at its tuning point and degrades away from it; M5 does not. Approaching the extended-arm singularity, the radial effective mass grows and M5 makes the arm feel locked, which is true of the robot. Stop well short of the singularity on hardware.

## Device-in-the-loop experiments (Inverse3)

Stability is a property of the coupled system, and the hand is part of it. These runs use the real Inverse3 and are presented as a single-operator characterization with repeated trials, not a user study.

### E5 — Maximum stable coupling stiffness

Shows hard-contact rendering: the stiffest coupling each method sustains, as a function of delay. This is essentially a Z-width measurement (Colgate and Brown).

- **Protocol:** increase K in steps; at each step, tap and hold against the table. Record K\_max as the last stable step.
- **Conditions:** M1 and M5 (plus one of M3 or M4 as a check); all four delays; two grips: held, and released (the handle dropped onto the wall, the worst case).
- **Instability criterion, fixed before running:** RMS force above about 8 Hz exceeding a threshold, or a sustained oscillation after release.
- **Plot:** K\_max vs T, one line per method and grip.
- **Expected:** a flat line for the model-based methods, set by the Inverse3 and the 1 kHz loop, and a falling line for M1, since P-P under delay needs damping that grows with K T. M3–M5 should give nearly the same K\_max: in static contact each reduces to the device against the coupling spring K. That is the expected result and is worth stating; this experiment separates P-P from model-based, not the models from each other.

### E6 — Contact taps and holds, with video

Shows contact quality when the handle is held, and produces the talk video.

- **Protocol:** the operator holds the handle and performs taps and holds on the table, a fixed number per condition.
- **Conditions:** all methods, all four delays.
- **Metrics:** the E3 contact metrics plus bounce count, settling time, high-frequency force energy and the energy balance E.
- **Video:** record every run side by side (device, robot, force overlay). One clip per method at 2.6 s is enough for the talk, and it addresses the reviewer's video request for the paper.

## Protocol details

- **Time alignment:** pair each haptic tick with the robot tick at which its target was actually applied, using packet send timestamps, not the nominal T\_c. This matters under variable delay and for every e\_F computation.
- **Logging, per run:** x\_l, v\_l, f (haptic side, 1 kHz); x, v, F\_osc as task\_wrench, x\_t, v\_t, f\_ff (robot side, 1 kHz); the model terms Λ, z, f\_eff at each update; packet send and receive timestamps on both links; the method, delay and parameters.
- **Fixed parameters, reported on the slides:** K, D, K\_r, D\_r, the mapping scale and force gain, f\_m, and M4's m with the configuration it was tuned at.
- **Start configuration:** the same for every run; compute Λ there before E1 and E2 to choose the two directions.
- **Ground truth for contact:** on hardware, contact force comes from F\_osc (or the FR3's external-wrench estimate), not from a sensor. Label it as such on the slides.
- **Safety:** the FR3's collision reflex thresholds, not the 80 N OSC limit, will abort the fast approaches in E3 first; check them before the velocity sweep. Stay well clear of the singularity in E4.

## Parameter selection

Every parameter is set by a stated rule on pilot runs that are not reported, then frozen before data collection. Parameters no method owns are shared by all methods; a method's own parameter (M4's m, M1's coupling under delay) is set to that method's best case by rule.

Order, since each step depends on the one before:

1. Mapping: position scale α and force gain β.
2. OSC gains K\_r, D\_r.
3. E5 at T = 0 (about half an hour), giving the model-based K\_max.
4. Coupling gains K, D.
5. A pilot E1 or E2 run, to choose f\_m.
6. The 5G delay model.
7. Freeze, and fill the table below.

### Rules

- **OSC stiffness K\_r (M3–M5).** An error δ in the assumed table position produces an unrendered robot force of about K\_r δ, so K\_r is capped by the force accepted: 10 N at δ = 2 mm gives K\_r ≤ 5000 N/m. Check that the FR3 OSC holds it stably. M1 is forced to K\_r = K, D\_r = D by its architecture.
- **OSC damping D\_r.** With ζ = D\_r / (2√(K\_r λ)), the heaviest direction of Λ is the least damped. Set D\_r so ζ ≥ 0.7 at the largest eigenvalue over the workspace, then check that the lightest direction is not so overdamped that velocity noise is audible. Verify with step responses at 2–3 configurations.
- **Coupling stiffness K.** 70% of the released-handle K\_max of a model-based method at T = 0, from E5. It is delay-independent for M3–M5.
- **Coupling damping D.** The coupling mode's reduced mass is μ = m\_d λ / (m\_d + λ) ≈ m\_d, the handle mass in robot units (m\_dev / αβ). Aim for ζ ≈ 0.7, capped by velocity-noise buzz.
- **M1 in device runs (E5, E6).** The common K where stable; otherwise 70% of M1's own K\_max(T) from E5, reported. Scripted runs (E1–E4) close no loop through the leader, so every method uses the common K there.
- **Feedback rate f\_m.** Set per signal, since M1 receives only the robot state and M5 only the model. Log Λ at 1 kHz in a pilot run and choose, for the model, the lowest f\_m at which Λ changes by only a few percent per update and holding it adds little to e\_F; for the state, the lowest f\_m whose hold ripple K\|v\|/f\_m stays under 1 N. Pilot (2026-10-26): 25 Hz for the model, 500 Hz for the state.
- **5G delay model.** Replay a measured trace if one can be found; otherwise fit a shifted heavy-tailed distribution (log-normal or gamma) to one cited study. Make the delay correlated (AR(1) or trace replay) with nondecreasing arrivals, not independent per packet: at 1 kHz, independent jitter of tens of ms reorders constantly and latest-value delivery drops most packets. Seed it, so every method sees the identical delay sequence on both links.

Reported 5G figures vary widely: an RTT of about 6.5 ms in a field trial ([source](https://www.netmode.ntua.gr/wp-content/uploads/2023/02/IEEE_CSCN_2022.pdf)), an average RTT of 47 ms for teleoperated driving ([source](https://arxiv.org/html/2503.14186v1)), and median latency 32.4 ms with 30.8 ms jitter on a private 5G link for telesurgery ([source](https://www.scielo.br/j/clin/a/yB969c5FvZM6FTQfCg77cwd/?lang=en)).

### Frozen values

| Parameter | Value | Units | Rule | Notes |
| --- | --- | --- | --- | --- |
| α, β (mapping) | 1.0, 0.5 | —, N/N | workspace and force fit | `teleop.yaml` `scale`, `force_gain`; handle clamp `max_i3_force` 10 N = 20 N at the robot |
| K\_r | 2000 (x, y, z) | N/m | K\_r δ ≤ accepted force | M3–M5; M1 uses K. `osc_teleop.yaml` `gains.k_pos_*`; 4 N at δ = 2 mm. Lowered from 3000: a 40–46 Hz oscillation on the torque-rate limit stopped the server (E1 god-object, 2026-10-26). Orientation k\_rot 85, 265, 10 N·m/rad |
| D\_r | auto: 2 ζ √(K\_r Λ\_ii), ζ = 0.7 | N·s/m | ζ ≥ 0.7 at λ\_max | M3–M5; M1 uses D. `gains.d_pos_* = -1`, `damping_ratio_pos` 0.7; per axis on the controller's live Λ\_ii, not a fixed value |
| K | 1000 | N/m | 70% of K\_max at T = 0 | `teleop.yaml` `kv` |
| D | 70 | N·s/m | ζ ≈ 0.7 on m\_d, buzz-capped | `teleop.yaml` `dv` |
| K for M1 (per T) | 1000 (common K) | N/m | common K, else 70% of M1's K\_max(T) | device runs only; `K_M1` in `e6_taps.py` is empty |
| m (M4) | E1 14.46 (heavy) / 4.79 (light); E2 4.73 (+y); E3 11.73 (z, start (0.4, 0, 0.12)); E5, E6 9.08 (z); E4 13.41 (along the line) | kg | nᵀΛ₀n | held Λ₀ (`rim_hold_orientation`), IK from home to each start; computed by the scripts per run |
| f\_t | 1000 | Hz | = f\_h | `teleop.yaml` `command_hz` |
| f\_m, robot state (M1) | 500 | Hz | ZOH ripple K\|v\|/f\_m ≤ 1 N | pilot (fm\_circle2): 0.72 N at p95 speed, 1 ms mean hold |
| f\_m, RIM model (M5) | 25 | Hz | Λ change ≤ 3 % per update, holding ≤ 0.5 N | pilot (fm\_circle2): 2.1 % p95, 0.12 N RMS |
| 5G delay model | replay, half per link, 10 ms per sample | ms | trace or fitted, correlated, seeded | lab trace (5G-data.csv, condition 5G, 21k samples): RTT median 40, p95 74, p99 81 ms; seeds 156 / 157; FORMULATION.md §1 |
| 5G packet loss | not modelled | % | same source | the trace has latencies only |

## Priorities and risks

Freeze the parameters first (Parameter selection, which starts with E5 at T = 0), then run in this order; each step leaves a usable slide if time runs out after it.

- [ ] E1 (2 directions × 2 frequencies) and E2: scripted, cheap, and they carry the model-fidelity message
- [ ] E3 with the velocity sweep: scripted
- [ ] E5 on the Inverse3: M1 vs M5, both grips
- [ ] E4
- [ ] E6, recorded as video at the same time

Risks to plan for:

- **Small inertial forces:** if Λ a is only a few newtons at the chosen frequencies, model differences disappear in noise. Check the magnitude at 2 Hz on the first run and raise the frequency or amplitude if needed, within the OSC and device limits.
- **Friction floor:** on hardware, joint friction may dominate M5's residual and set M5's floor in E1. E2 and E4 are less sensitive, because they depend on how the inertia changes, not on its size.
- **Identical K\_max across M3–M5:** expected, not a failure; frame E5 as model-based vs P-P.
- **Frictionless models:** the operator feels no tangential friction in M3–M5 while the real robot does. It does not affect E1–E5, but be ready for the question.
