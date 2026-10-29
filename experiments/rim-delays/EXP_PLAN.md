# Delayed RIM Teleoperation — Talk Experiment Plan

Oct 23, 2026 · @Charles · merged on Oct 27 with the online plan of Oct 23 (its lab-session checklist dropped)

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

**M5's inertia: orientation held.** The OSC holds the end effector's orientation, and at the experiments' frequencies (below its rotational modes: 3.6, 4.7 and 11.4 Hz with the tool point imposed at the start, 2.6 Hz lowest at the E6 contact; FORMULATION.md §3) the arm moves as if the orientation were fixed: on the E2 circle the free-rotation Λ was ~7× too light along x (radial gap +7.4 N, +1.2 N with the held reduction). M5 therefore reduces with the orientation held (`rim_hold_orientation: true` in `teleop.yaml`, `--no-rim-hold-orientation` for the free reduction), and M4's m is taken from the same held Λ₀ (`tools/lambda_map.py`, `Lambda_held`) so the comparison stays fair. The frequency-dependent version (a 6-D proxy held by the OSC's own orientation gains, free at impacts and held in slow pushes) is future work (M6, FORMULATION.md).

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

Contact metrics (E3, E6): time to the hold force (from the leader's crossing of z = s until the felt force is within ±20 % of its hold value for good — the one defined for every method: M1 under delay drags more before contact than it feels in it, so its onset and overshoot are not defined); force-onset latency after the crossing; leader penetration at onset; felt-force overshoot and oscillation; robot-side peak table force; e\_F during the hold; whether osc\_controller's contact force limiter fired.

Stability indicators (E5, E6): bounce count, settling time, and RMS force in a band above about 8 Hz.

Energy balance, computed offline from the logs for every method. A negative value means the system generated energy; it gives one method-agnostic number for how active a run was.

```latex
E(t) = -\int_0^t \left( f \cdot v_l + F_{osc} \cdot v \right) d\tau
```

**Phase-separated masses (E1, E4).** Inertia and friction are separated by phase on a periodic motion. Along the motion direction n, take the first harmonic at the drive frequency ω (least squares on cos, sin, an offset and a linear trend for a slow motion underneath) of a position, X, and a force, F. For a mass, F = m a = −m ω² x, in phase opposition with x; Coulomb and viscous friction are in phase with the velocity, 90° away, so they drop out of the in-phase part:

```latex
m = -\frac{\mathrm{Re}(F/X)}{\omega^2}, \qquad F_c \approx \frac{\pi}{4}\,\mathrm{Im}(F/X)\,|X|
```

Applied to three pairs (`exp_metrics.e4`, `mass_by_s`, per position bin along E4's line; E1's notebook fits the same over its three frequencies):

| Mass | Force | Position | Clock | Meaning |
| --- | --- | --- | --- | --- |
| m\_robot | F\_osc (task\_wrench) | robot EE (ee\_state) | robot, window shifted by the measured lag | the arm's apparent mass under its controller |
| m\_felt | −f | proxy x\_p | haptic | the mass the method renders |
| m\_err | e\_F | proxy x\_p | haptic | the mass the operator was not shown (≈ m\_robot − m\_felt when the robot tracks the proxy) |

Also logged: robot/proxy tracking (amplitude ratio at ω; near 1 means the methods are compared on the same motion) and the model's nᵀΛn (M5).

Caveats. (1) With one frequency the in-phase part is m − k/ω², k any stiffness-like force in phase with the position (null-space posture, position-dependent gravity-model error); E1's three-frequency fit gave k ≈ 30–60 N/m, which at E4's 0.7 Hz (ω² ≈ 19) would make m\_robot read 1.5–3 kg low — two frequencies separate m and k. (2) F\_osc is the commanded force, equal to the applied one while nothing saturates (torque, torque rate, the contact force limiter); joint friction then belongs to the arm, in Im(F/X). (3) It is an apparent mass at ω: it includes the orientation hold and anything the model leaves out (rotor inertia). (4) Drive below the proxy's coupling resonance, √(K/m)/2π: near it the proxy swings wider than the leader and the robot cannot follow (E4's first design, 1.5 Hz on a ~1.4 Hz resonance).

## Scripted-leader experiments

A scripted leader gives every method the identical trajectory, so differences come from the method alone. It cannot show coupled instability; that is what E5 and E6 are for.

### E1 — Free-motion transparency

Shows that model-based transparency is flat in delay and improves with model fidelity.

- **Protocol:** sinusoids of about 5 cm amplitude, along the heaviest and the lightest eigenvectors of Λ at the start configuration. Report the eigenvalue ratio on the slide. M4's m is retuned per direction (nᵀΛ₀n along each), its best case, so E2 and E4 carry M4 vs M5.
- **Conditions:** M1, M3, M4, M5; frequencies 0.25, 0.5 and 1 Hz, inside the held-rotation model's validity; delays as in Methods and conditions.
- **Plots:** RMS e\_F vs T, and RMS e\_F vs frequency. Optional slide: the apparent mass fitted across frequencies, including the earlier 2–4 Hz runs (T1). It moves from Λ\_6,vv toward the free-rotation Λ: a controlled arm's inertia depends on frequency and on its controller.
- **Expected:** M1 grows with T; M3 grows as ω²; M5 stays low. With m retuned per direction, M4 should match M5 in the two directions. On hardware, M5's residual is likely dominated by joint friction at low speed: keep it, it is the honest floor of the method.

### E2 — Horizontal circle (anisotropy)

Shows that a fixed mass cannot follow an inertia that varies with the direction of motion.

- **Protocol:** a horizontal circle at constant speed: radius 10 cm, period 2 s (v ≈ 0.31 m/s, a ≈ 0.99 m/s²), centered at (0.5, 0, 0.25) m. The centripetal acceleration rotates through every direction of the plane.
- **Start position (E2 only):** (0.5, 0, 0.3) m, not the common start: centered on (0.4, 0, 0.16) the near side folds the arm until joint 4 is at −3.03 rad, where libfranka's position-dependent velocity limit stops even a 3 s circle (joint 4 at ~100 % of its limit; a 1.5 s circle aborted with `joint_velocity_violation`). At (0.5, 0, 0.25) the 2 s circle uses ~52 % of the limit. Compute Λ at this start for M4's mass.
- **Conditions:** M4 and M5 (M3 for reference), at T = 0 and 2.6 s.
- **Plots:** e\_F against circle phase, and its lap-averaged split into a radial part (inertia, where the methods differ) and a tangential part (friction, common to all model-based methods); the angle between force and acceleration, for f and for −F\_osc. The split is approximate (anisotropic Λ leaks inertia tangentially, joint friction is not along −v); the exact separation runs each circle both ways (Appendix).
- **Expected:** M4's e\_F oscillates at twice the circle frequency while M5 stays flat. With off-diagonal coupling, the robot's force is not parallel to its acceleration; M5 reproduces that sideways component, M4 cannot.

### E3 — Contact onset over approach velocity

Shows when the model matters at contact: model-based vs P-P, and the inertial transient switching on with speed.

- **Protocol:** a vertical approach at constant velocity into the table, a hold at fixed leader depth, then retraction.
- **Conditions:** all methods; approach speeds of 1, 5, 10 and 20 cm/s at T = 850 ms; all four delays at one speed; three repetitions each.
- **Steady-approach sweep (M3–M5, added 2026-10-26):** 5, 10 and 20 cm/s at T = 850 ms with a 20 cm pre-height, so the proxies meet the table at the leader's speed (≥ 1 s at constant speed, about 3 decay times of the coupling mode); M4's mass tuned at the contact. In the main sweep the leader reaches its speed only 3 cm above the table, and the M4/M5 proxy is still catching up at contact.
- **Plots:** contact metrics vs approach speed; one time series at 850 ms (leader z, felt force, and robot −F\_osc shifted by T\_c) for M1 vs M5.
- **Expected:** at low speed all methods converge. As speed rises, M1's onset kick grows with v T, and the model-based transients scale with momentum (Λ v for M5, m v for M4, none for M3). Static force is the same across M3–M5 (K × depth), so E3 carries model-based vs P-P and E1/E2 carry which model.

### E4 — Reach sweep (configuration change)

Shows that a mass tuned at one configuration is wrong elsewhere.&#32;

- **Protocol:** a slow scripted line from near the base outward, with a small sinusoidal dither superimposed so inertial force is present throughout.
- **Design (2026-10-26):** x 0.30 → 0.68 m at z = 0.15 m (held Λ along z ~16 → ~7 kg), a 3 cm vertical dither at 0.7 Hz (below the proxy's coupling resonance, 1.3–1.9 Hz along the line), 1 cm/s along the line; masses phase-separated at the dither frequency (friction drops out). The first design (x at z = 0.25 m, 1 cm at 1.5 Hz along the line) sat on that resonance, and Λ along x hardly changes there.
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
- **Run as E5-lite (2026-10-26):** K and D frozen (1000, 70), so only M1's ladders at 850 ms and 2.6 s (down from 1000) and M5 at 2.6 s (up to 2000), 20 s per step. Result: held, M1 was stable at K = 1000 at both delays; M5 stable at 2000 held, unstable at 2000 released (HF RMS 1.27 N, 3 guard trips). No K\_max(T) trend: the hand damps P-P's slow oscillation (~1/(2T), 0.2 Hz at 2.6 s), and the HF criterion (> 8 Hz) cannot see it.
- **Dropped grip (added 2026-10-27):** the worst case, hands off. The operator brings the handle onto the table, lets go within 1 s of touching it and does not touch it again for the rest of a 30 s run (about 5 periods of 1/(2T) at 2.6 s), catching it only at the workspace edge. Criterion, added to the HF one: unstable if, from 2 s after the first touch to the end, the loop threw the free handle more than 10 mm above the table (`rebound_mm`; also logged: `rebounds`, `depth_max_mm`). Ladders: M5 at 2.6 s, K = 1000 (the reference); M1 at 850 ms and 2.6 s, down from 1000 (1000, 500, 250, 125). About 15 min. The first session's `released` runs (dropped and picked up 3 times) stay in `kmax.csv` but are not comparable: the hand was back on the handle between drops. **Velocity filter 50 Hz for the dropped grip** (`--vel-filter-hz 50`, `VEL_FILTER_HZ` in the script; frozen 8 Hz elsewhere): the first dropped run (M5, 2.6 s, 8 Hz, 17:51) rang at ~18 Hz as soon as the handle was let go, before any contact. The 8 Hz filter delays the velocity ~100° at 17 Hz, so D acts on the free handle (~0.06 kg) as a ~560 N/m spring with slightly negative damping: phase margin ~0° with the device's ~2–3 ms latency. It is a device-side mode, independent of the method and the delay, which the hand damps in held runs. **50 Hz did not work:** the M5 check at T = 0 (2026-10-27 19:12) grew at 22–24 Hz while the handle came down, then went to 58 Hz at the 50 N clamp within 3 s (robot at its 80 N OSC limit, watchdog stop). The single-mass model's ~30° margin was optimistic: the Inverse3 has more lag or dynamics above ~20 Hz. Dropped grip skipped.
- **Plots (`e5_kmax.ipynb`):** K\_max vs T per method and grip (▽ where no step of the ladder was stable), and the talk's figure: the dropped handle's height and felt force from its first touch, M1 vs M5 at K = 1000, per delay.
- **Expected (dropped):** M5's handle rests on its local wall at every delay. M1's wall is the delayed robot: it pushes a round trip late, and keeps pushing a round trip after the handle has left, so the loop throws the handle; likely unstable at every K of the ladder at 2.6 s.
- **Expected:** a flat line for the model-based methods, set by the Inverse3 and the 1 kHz loop, and a falling line for M1, since P-P under delay needs damping that grows with K T. M3–M5 should give nearly the same K\_max: in static contact each reduces to the device against the coupling spring K. That is the expected result and is worth stating; this experiment separates P-P from model-based, not the models from each other.
- **Guided stiffness ladders (added 2026-10-27, to run 2026-10-28; replaces the dropped grip, skipped):** the claim is that a local model renders a stiff wall under delay. E6 already shows the wall at K = 1000 N/m (felt force against depth: M3–M5 0.95–1.04 N/mm through zero at every delay; M1 at 850 ms and 2.6 s R² 0.10–0.24 with 22–35 N at zero depth), but that is only βK = 200 N/m at the handle. Protocol: E6's guide, hand held (tap, hold, tap, hold; 4 s holds), from a 10 mm hover (E6: 30 mm; lower so the window zooms and the dip stays visible: 1.25 mm ≈ 14 px at 8000 N/m), at a depth asking ~10 N at every K (guide depth 10 N / K: 10 mm at 1000 N/m, 1.25 mm at 8000), velocity filter at the frozen 8 Hz (as E6, so its K = 1000 step is E6's configuration), up the ladder until unstable. Ladders: M5 at T = 0 and 2.6 s, and M3 at 2.6 s: K = 1000, 2000, 4000, 8000 N/m (200–1600 N/m at the handle); M1 at T = 0 and 2.6 s: K = 1000, 2000 only, since M1's robot stiffness is K (K\_r := K) and the OSC oscillated at 3000 N/m — a limit of P-P itself. 16 steps of ~25 s, ~35–45 min. Verdict: HF RMS after each impact (E6's), else over the contact, < 0.5 N and no guard trip, confirmed by feel; reported with the felt wall (slope, offset, R², `exp_metrics.wall`). Figure: F9 in `analysis/talk_figures.ipynb`.
- **Expected (guided):** M5 and M3 feel the K that is set, up to the same K\_max at T = 0 and 2.6 s (the device's limit). M1 matches at T = 0 up to 2000; at 2.6 s it renders no wall at any K (offset, low R²), whatever the verdict.
- **Result (guided, 2026-10-28; M3 skipped for time):** every step stable (HF RMS 0.10–0.33 N), so K\_max is above the highest step, 8 N/mm (1.6 kN/m at the handle). The result is the wall, not the verdict: M5 renders the set stiffness at 2.6 s as at T = 0 (felt 1.03 / 2.05 / 3.89 / 7.87 N/mm against 1.03 / 2.01 / 3.88 / 7.45, offsets ≤ 1 N, R² 0.81–0.96). P-P renders it at T = 0 (1.01 / 1.93 N/mm) but at 2.6 s has no wall although stable: 0.59 then −0.18 N/mm, R² 0.31 then 0.00, 8 then 22 N at zero depth. Stable is not transparent. Unfelt force during holds: M5 −2.5 to −3 N at every K and delay; P-P at 2.6 s 2.2 → 8.1 N. Figure F9.

### E6 — Contact taps and holds, with video

Shows contact quality when the handle is held, and produces the talk video.

- **Protocol:** the operator holds the handle and follows a guide window (`fr3_teleop --guide`, `fr3_haptic/guide.py`): a reference height scrolling in from the right, over the table, the hand and (M3–M5) the proxy, all local and undelayed. Per run: tap, hold, tap, hold, tap, hold (taps at 8 cm/s with a 0.3 s touch, holds at 5 cm/s with 4 s at 10 mm below the table, ≈ 10 N for every method), hovering 30 mm above the table in between; a 34 s guide, runs of 35–36.5 s (3–4.3 s after the last lift). Taps at 20 cm/s were too fast to follow on the Inverse3 (the hand peaked at 7–8 cm/s, reached 2 and 8 mm of the 10, missed one; 2026-10-27). The guide window must not load the haptic loop: OpenCV on one thread, nice 10, 30 fps (with its 32-thread pool at 60 fps, 11 % of the 1 kHz ticks were over 2 ms). Only z is teleoperated (2026-10-27): the leader's x, y are locked at the start (`--lock-axes xy`), with a light centring spring on the handle (150 N/m, 10 N·s/m world; 30 N/m, 2 N·s/m at the handle); the interface and M5's 3-D model are unchanged, the proxy's x, y held by the coupling to the fixed leader. Unlocked, the hand drifted 14–29 mm in x and 23–78 mm in y and M1 rendered 2–7 N sideways in contact. Every condition gets the same motion; the metrics are split per kind (`tap_*`, `hold_*`), with how closely the hand followed (`track_rms_mm`). Without the guide (pilot, 2026-10-27, M1 at T = 0) the hand pressed 36–61 mm deep, 35–50 N, at the handle clamp. A contact runs from the leader's crossing to when it leaves the table; the hold force is the median over the 0.5 s before the lift, the HF RMS starts 150 ms after the onset (the impact is broadband), and the robot's force is read when the command reached it (T\_c later).
- **Conditions:** all methods, all four delays. M1 uses the frozen K at every delay (E5 not run): an unstable M1 run is a result, kept. M4's m is nᵀΛ₀n along z at the contact, (0.4, 0, 0.083): 12.64 kg.
- **Metrics:** the E3 contact metrics, plus whether the force reached the operator before the lift (M1 under delay) plus bounce count, settling time, high-frequency force energy and the energy balance E.
- **Video:** three clips, not sixteen: M1 at T = 0 (the reference), M1 and M5 at 2.6 s (`VIDEO` in `e6_taps.py`). Film the arm from the side; `tools/contact_video.py` renders the hand, proxy and robot positions next to the clip (`--mode side`) or over it (`--mode overlay`, two clicked calibration points), synced on the robot's first table contact.

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
| α, β (mapping) | 1.0, 0.2 | —, N/N | workspace and force fit; stable on the device | `teleop.yaml` `scale`, `force_gain`; handle clamp `max_i3_force` 10 N = 50 N at the robot. β only matters with the device (E5, E6): handle stiffness K β = 200 N/m, damping D β = 14 N·s/m. At β = 0.5 (35 N·s/m at the handle, through the 8 Hz velocity filter) the Inverse3 and the haptic loop went into a 33 Hz limit cycle; β = 0.2 is stable and comfortable (2026-10-26). The scripted runs (E1–E4) have no handle: they used β = 0.5 for their logged clamp only |
| K\_r | 2000 (x, y, z) | N/m | K\_r δ ≤ accepted force | M3–M5; M1 uses K. `osc_teleop.yaml` `gains.k_pos_*`; 4 N at δ = 2 mm. Lowered from 3000: a 40–46 Hz oscillation on the torque-rate limit stopped the server (E1 god-object, 2026-10-26). Orientation k\_rot 85, 265, 10 N·m/rad |
| D\_r | auto: 2 ζ √(K\_r Λ\_ii), ζ = 0.7 | N·s/m | ζ ≥ 0.7 at λ\_max | M3–M5; M1 uses D. `gains.d_pos_* = -1`, `damping_ratio_pos` 0.7; per axis on the controller's live Λ\_ii, not a fixed value |
| K | 1000 | N/m | 70% of K\_max at T = 0 | `teleop.yaml` `kv` |
| D | 70 | N·s/m | ζ ≈ 0.7 on m\_d, buzz-capped | `teleop.yaml` `dv` |
| K for M1 (per T) | 1000 (common K) | N/m | common K at every delay | device runs only; E5-lite (2026-10-26): stable held at 850 ms and 2.6 s, so no per-delay K. The dropped ladders (2026-10-27) do not change it: E6 holds the handle |
| m (M4) | E1 14.46 (heavy) / 4.79 (light); E2 4.73 (+y); E3 11.73 (z, start (0.4, 0, 0.12)); E5 9.08 (z); E6 12.64 (z, at the contact (0.4, 0, 0.083)); E4 15.95 (z, at the line's start (0.30, 0, 0.15)) | kg | nᵀΛ₀n | held Λ₀ (`rim_hold_orientation`), IK from home to each start; computed by the scripts per run |
| f\_t | 1000 | Hz | = f\_h | `teleop.yaml` `command_hz` |
| f\_m, robot state (M1) | 500 | Hz | ZOH ripple K\|v\|/f\_m ≤ 1 N | pilot (fm\_circle2): 0.72 N at p95 speed, 1 ms mean hold |
| f\_m, RIM model (M5) | 25 | Hz | Λ change ≤ 3 % per update, holding ≤ 0.5 N | pilot (fm\_circle2): 2.1 % p95, 0.12 N RMS |
| 5G delay model | replay, half per link, 10 ms per sample | ms | trace or fitted, correlated, seeded | lab trace (5G-data.csv, condition 5G, 21k samples): RTT median 40, p95 74, p99 81 ms; seeds 156 / 157; FORMULATION.md §1 |
| 5G packet loss | not modelled | % | same source | the trace has latencies only |

## Priorities and risks

Freeze the parameters first (Parameter selection, which starts with E5 at T = 0), then run in this order; each step leaves a usable slide if time runs out after it.

- [x] E1 (2 directions × 3 frequencies, 60 runs) and E2 (8 runs), 2026-10-26
- [x] E3 with the velocity sweep and the steady-approach sweep, 2026-10-26
- [x] E5-lite, held and released (6 steps), 2026-10-26
- [x] E4 (`zdither`, 4 runs), 2026-10-26
- [x] E6 (2026-10-27, 16 runs + video); E5 dropped grip skipped: the free handle needs a faster velocity filter, not worth the time before the talk

### Tomorrow (2026-10-28): the stiffness ladder

- [x] **E5 guided stiffness ladders** (done 2026-10-28, M3 skipped) (~40 min, Inverse3, hand held): `pixi run -e humble python experiments/rim-delays/exp/e5_kmax.py` runs only the guided ladders (the held and released steps are done and skipped). Same grip at every step; flip a verdict by feel if needed.
- [x] Re-run `analysis/talk_figures.ipynb`: F9 draws itself from `kmax.csv`.
- [x] **E5 extension** (2026-10-28): M5 stable to 16 N/mm at T = 0 and 2.6 s (felt 15.40 / 15.80 N/mm, R² 0.94 / 0.99): no device limit found up to 3.2 kN/m at the handle. P-P at T = 0 stable to 4 N/mm (felt 3.89, R² 0.95; the expected failure at 3 did not happen with the hand on). P-P's wall against the delay at 1 / 2 N/mm: R² 0.76 / 0.81 at T = 0, 0.62 / 0.70 at 5G, 0.08 / 0.01 at 850 ms, 0.31 / 0.00 at 2.6 s; offset 1.5–3.3 → 8–22 N.
- [x] **P-P up to its robot limit** (2026-10-28): osc\_controller bounds gains.k\_pos\_* to [0, 5000] N/m and P-P sets K\_r := K, so P-P stops at 5000 (`OSC_K_MAX`; steps above are skipped, not raised: P-P's D\_r = D = 70 gives ζ ~0.1 at 8000). T = 0 and 5G: stable to 5000 with a wall (R² 0.89–0.95), its offset growing with K (1.5 → 7.9 N, the drag). 850 ms: no wall at any K (R² 0.08 / 0.01 / 0.29 / 0.38 at 1 / 2 / 4 / 5 N/mm; the slopes near K at 4–5 are fits to a cloud), unstable at 5000 (HF 1.05 N, 19 N unfelt in holds). F9 marks R² < 0.5 as "no wall" and draws no slope through it.
- [ ] If time: E7, compliant robot (K\_r 2000 / 500 / 250 N/m, fast reaches, M3–M5): needs a K\_r override for M3–M5, not written yet.

### To finish today (2026-10-27): all the data and the figures

Leave with every run on disk and analysed, and the talk figures (FIGURES\_PLAN.md) rendered. Experiments first, while the robot and the Inverse3 are set up; the figures need no hardware.

**Experiments (Inverse3, ~1 h 15 min)**

- [ ] ~~**E5 dropped**~~ skipped (2026-10-27) (~15 min): `exp/e5_kmax.py` runs only the new ladders: M5 at 2.6 s, K = 1000, then M1 at 850 ms and 2.6 s, down from 1000. Hands off after the release; flip the verdict if you had to catch the handle. Film the M1 and M5 runs at 2.6 s, K = 1000: they are the stability clip.
- [x] **E6 guided taps** (~45 min): `exp/e6_taps.py`, 14 runs left (M1 at T = 0 and 5G are done, 16:47–16:49). Check `track_rms_mm` and `missed` after each run; redo a run where the hand missed a contact.
- [x] **E6 video** (with the runs above): the three `VIDEO` conditions (M1 at T = 0, M1 and M5 at 2.6 s), filmed from the side; note each clip's file name against its run.
- [ ] **Backup copy** of `~/data/franka/rim-delays/robot/` (E1–E6) and the video files off the lab PC before leaving.

**Optional, if the robot is still free (~30 min, scripted)**

- [ ] E2 counter-clockwise, to separate inertia from friction exactly (Appendix): M4 and M5 held at T = 0 and 2.6 s, 4 runs. Needs a reverse-direction option for the `circle` script in `fr3_haptic/scripted.py` first. Higher value than the repeats below: it is E2's missing result.
- [ ] Repeats for the error bars: E4 M4 and M5 at 2.6 s (×2), E1 heavy 0.25 Hz at every delay for M1 and at 0 / 2.6 s for M3–M5 (×2).

**Analysis and figures (~2 h, no hardware)**

- [ ] Re-run every notebook top to bottom (E1–E6) on the final sessions, outputs saved.
- [ ] E3: add the steady-sweep session to `RUNS` (the saved outputs have 28 of 37 runs), so the steady plots and time series are not empty.
- [ ] ~~E5: read the dropped runs~~ (skipped): M1's K\_max per delay, and whether M1 was thrown where M5 rested. If M1 was stable dropped, use the fallback sentence in FIGURES\_PLAN.md.
- [ ] E6: fix `%autoreload 2  # …` in `e6_taps.ipynb` (the inline comment is a UsageError), then write the discussion cell (tap/hold metrics per method and delay, M1's force arriving after the lift).
- [x] Render the talk figures F1–F7b with `figures/talk_figures.ipynb` (outputs in `figures/out/`) at slide size into `experiments/rim-delays/figures/`: one message per figure, ≥ 14 pt, the method colours of `plots.py`, as listed in FIGURES\_PLAN.md.
- [ ] `tools/contact_video.py` on the three E6 clips and the E5 stability clip.
- [ ] Commit the scripts, notebooks (with outputs) and figures.

Risks to plan for:

- **Small inertial forces:** if Λ a is only a few newtons at the chosen frequencies, model differences disappear in noise. Check the magnitude at 1 Hz on the first run and raise the frequency or amplitude if needed, within the OSC and device limits.
- **Friction floor:** on hardware, joint friction may dominate M5's residual and set M5's floor in E1. E2 and E4 are less sensitive, because they depend on how the inertia changes, not on its size. First circle: about 10.7 N against the motion at 0.32 m/s, the same for every model-based method, so it is reported as the tangential part of the gap.
- **Identical K\_max across M3–M5:** expected, not a failure; frame E5 as model-based vs P-P.
- **Frictionless models:** the operator feels no tangential friction in M3–M5 while the real robot does. It does not affect E1–E5, but be ready for the question.

## Appendix — Separating model error from friction

The radial/tangential split used in E2 is an approximation; this is the rigorous version and the experiment that makes it exact.

**Why the split is approximate.** On a circle at constant speed, velocity is tangential and acceleration radial, so an inertia error would be purely radial and friction purely tangential, if Λ were isotropic and friction acted exactly along −v. Neither holds. With ΔΛ = diag(δ\_x, δ\_y), centripetal acceleration A and position θ on the circle, the inertia error has components

```latex
\text{radial: } A\big[\tfrac12(\delta_x + \delta_y) + \tfrac12(\delta_x - \delta_y)\cos 2\theta\big],\qquad \text{tangential: } \tfrac12 A\,(\delta_x - \delta_y)\sin 2\theta
```

The tangential leak averages to zero over a lap. Joint Coulomb friction is dissipative but not aligned with −v at the tool, so part of it appears radially (on the robot, ~8.5 N of radial 2f, E2 notebook).

**Energy argument.** The tangential component times the speed is the power of the gap force. Inertia errors do no net work over a closed periodic motion; for a constant symmetric error:

```latex
\oint v^\top \Delta\Lambda\, a\,dt = \Big[\tfrac12 v^\top \Delta\Lambda\, v\Big]_{\text{start}}^{\text{end}} = 0
```

The same holds approximately for slowly varying Λ, and for gravity errors (conservative). So the lap-averaged tangential gap times the speed is the power dissipated by forces the model lacks: on the first circle, 10.7 N over 2.42 m gives 26 J, against a 27.7 J deficit.

**Exact separation: run each circle both ways.** Reversing the direction leaves a(θ) unchanged (centripetal, still inward), and the Coriolis terms are quadratic in v, so everything the model represents is even in velocity; Coulomb and viscous friction are odd. Aligned by position θ, not by time:

```latex
e_{model}(\theta) = \tfrac12\big(\mathrm{gap}_{cw}(\theta) + \mathrm{gap}_{ccw}(\theta)\big),\qquad e_{friction}(\theta) = \tfrac12\big(\mathrm{gap}_{cw}(\theta) - \mathrm{gap}_{ccw}(\theta)\big)
```

This captures friction in any direction, including its radial leak, and attributes the full anisotropic inertia error to the model. Cost: one extra run per condition.

**E1 and E4 (motion on a line).** v and a are collinear, so there is no geometric split; the same principle works through phase. Project the gap onto the acceleration waveform (the even, inertial part) and onto the velocity waveform, 90° out of phase (the odd, friction part): the phase-separated masses of the Metrics section.
