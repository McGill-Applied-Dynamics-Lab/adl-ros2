# Scripts for EXP_PLAN.md

What has to be built to run [EXP_PLAN.md](EXP_PLAN.md), in order. KISS: one script per experiment,
one run per invocation, its parameters set by hand at the top of the file; repeats are reruns.

## Decisions (2026-10-23)

- No conditions grid or built-in repeats: each experiment script runs one condition (method,
  delay, ...) written at its top, then prints its metrics and saves its plot.
- E5: one `fr3_teleop` process per K step.
- 5G: the trace's sampling period is unknown (`time` is a counter) — settled in B4.
- The scripted-grid design (`run_conditions.py`, `conditions.yaml`, `analyze.py`, the old
  `PLAN.md` and `README.md`) is retired under `old/`.
- Video: started by hand, not handled by the scripts for now.

## A. Pipeline changes (`fr3_haptic`, `haptic_teleop`)

| # | Change | For |
|---|---|---|
| A1 | Script profiles: `sine` (direction n, amplitude, frequency, cycles), `hcircle` (free-air horizontal circle at the start height), `approach` (constant velocity through the surface, hold at a fixed leader depth, retract), `line` (A→B slow, plus a dither) | E1–E4 |
| A2 | Proxy x/v in the 1 kHz haptic log; the plant stream recorded at every controller tick, independent of `f_m` | e_F, energy |
| A3 | One clock (`CLOCK_MONOTONIC`) for haptic and robot streams; a sequence number per command, logged at send and delivery on both links, also at zero delay | time alignment |
| A4 | RIM model terms (Λ, z_i, f_eff) logged per update, with send/receive times | E4, f_m pilot |
| A5 | Metadata: OSC gains read back from the controller, start joints q₀, Λ₀ eigenvalues | reported parameters |
| A6 | `--proxy-mass auto`: nᵀΛ₀n along the script's initial direction, logged | M4 best case |
| A7 | `feedforward_cap` raised (Λa ≈ 19 N on E2), saturation logged | E1, E2 |
| A8 | Delay seed fixed per condition; M5's model on the feedback link's delay sequence | identical delays |
| A9 | 5G delay model (after B4) | 5g condition |
| A10 | `--home` then `--start-position` in every preset; abort if q₀ is off the reference | same start |

## B. Tools (`tools/`) and frozen parameters

| Script | Does |
|---|---|
| B1 `lambda_map.py` | Λ at a configuration (live robot or IK of a position): eigenvalues/vectors, nᵀΛn; Λ along a line, distance to the singularity |
| B2 `osc_gains.py` | T1 step responses at 2–3 configurations; ζ = D_r / (2√(K_r λ)) at λ_max and λ_min; the K_r δ cap |
| B3 `pilot_fm.py` | From a 1 kHz proxy-rim pilot: Λ change per update and extra e_F when held, vs f_m |
| B4 `delay_model.py` | The 5G trace: per-recording stats, the replay or fitted model, the sequence a seed gives |
| `params.yaml` | The frozen values every experiment script reads; prints the table for EXP_PLAN.md |

## C. Experiments (`exp/`)

One script per experiment, parameters at the top, including the lists of methods and conditions
(revised 2026-10-25: running a list is optional, `--only-method` / `--only-rtt` narrow it). For
each condition the runner (`exp/common.py`) asks, runs `fr3_teleop` (`--sim`: `sim_teleop.py`),
saves the run as `<YYYYmmdd_HHMMSS>_<exp>_<method>_rtt<T>_<label>` (never overwritten), analyses it
(`analysis/check.md` + figures, `analysis/metrics.json`) and appends it to the session file
`<exp>/sessions/<start>.txt`. `--analyse <run dirs>` re-runs the analysis; `--dry-run` prints the
commands. M4's mass is set per condition to nᵀΛ₀n (Λ₀ from IK at the start, the motion direction).
Notebooks `exp/eN_*.ipynb`: `RUNS` = run dirs, session files or a folder; table + the plan's plots.

| Script | Scripted profile / device | Conditions (defaults) | Metrics (`exp_metrics.py`) |
|---|---|---|---|
| `e1_free_motion.py` | `sine` along Λ₀'s heavy / light eigenvector | 4 methods × {0, 2600} (M1: + 5g, 850) × 3 freqs | RMS e_F along n, f, F_osc |
| `e2_circle.py` | `circle` in the air, start (0.5, 0, 0.3) | M3–M5 × {0, 2600} | e_F radial / tangential, by phase; force angles |
| `e3_approach.py` | `approach` (constant speed through the table) | speeds at 850 ms + delays at 5 cm/s | onset latency, penetration, overshoot, robot peak force, saturation |
| `e4_reach.py` | `line` + dither | M4, M5 × {0, 2600} | e_F and Λ eigenvalues along the line |
| `e5_kmax.py` | Inverse3, one run per K | ladder per method × delay × grip, `kmax.csv` | HF RMS > 8 Hz → stable / unstable (confirmed) |
| `e6_taps.py` | Inverse3 | 4 methods × 4 delays | contact + bounces, settling, HF RMS, energy |

`metrics.py` (shared): e_F on logged packet times, energy balance E(t), force-onset latency,
penetration at onset, overshoot, robot peak force, bounce count, settling time, RMS > 8 Hz.

## Order

1. Logging and alignment (A2–A5, A7, A8, A10), `metrics.py` (e_F, energy), B1.
2. Parameter selection: B2 → E5 at T = 0 → K, D → B3 → B4 + A9 → `params.yaml` frozen.
3. E1, E2 (A1 `sine`, `hcircle`; A6).
4. E3 (A1 `approach`; contact metrics).
5. E5, E4 (A1 `line`), E6.

`sim_teleop.py` has a constant diagonal Λ: E1/E3 dry-run in sim; E2/E4 need the robot.

## Status

**Step 1 (2026-10-23): done, sim-checked, not yet run on the robot.**

- A2: haptic log has `proxy_x_*`, `proxy_v_*`; `plant` is every controller tick (with `ee_velocity`).
- A3: haptic log has `mono_s`, metadata `t0_mono_s`; the haptic tick travels with every command
  (`command.tick`) through the delay link (`command_link.tick`) to the robot (`target` stream: every
  published target with its tick). `runs.load_run` puts the haptic stream on the robot's clock.
- A4: `model` stream (proxy-rim): `M_eff`, `z_i`, `f_eff`, `x_i` per update, when sent.
- A5: metadata `controller_params` (read back), `q0`, `t0_mono_s`. Λ₀ eigenvalues: with A6.
- A7: `feedforward_cap` 30 N in `teleop.yaml`.
- A8: the model line uses the feedback seed (identical delays when constant or time-indexed).
- A10: left to the experiment presets (`--home` + `--start-position`; `q0` is logged to check).
- `metrics.py`: `unfelt_force` (e_F on the measured tick → robot lag), `energy_balance`, `rms`.
  Sim check (scripted press, no hand): e_F RMS z — ZOH 0.05 N at 0 ms, 20.8 N at 425 ms/link;
  RIM 0.54 N at both; measured lag = delay + ~2 ms.
- B1 `tools/lambda_map.py`. At (0.4, 0, 0.2) from home: Λ eigenvalues 0.87, 1.95, 9.41 kg (ratio
  10.8), heaviest ≈ z tilted 18° to −x, lightest ≈ x. Outward to x = 0.75 m Λ *decreases* and
  σ_min stays ~0.25: E4's line must reach further to approach the singularity.
- Robot checks R1–R4 (`robot/checks/`): lag exact, Λ₀ matches B1, RIM e_F flat in delay
  (ZOH 2.0 → 19.6 N on z). Found: RIM's per-axis contact projection held a false −6.8 N lateral
  force in a vertical press → fixed in `pyrim.RIMIntegrator` (impulse along A⁻¹n, FORMULATION.md
  "3-DoF contact projection"); rerun R3 confirms (e_F x RMS 5.25 → 1.41 N). Plant subscriptions'
  queue 5 → 50 (2–19 % of controller ticks were dropped): rerun R1/R4 to confirm.
- Friction is RIM's floor: Coulomb ±6.2 N on z (flat in speed, flips with direction, identical
  under ZOH), plus a +1.3 N constant z bias — not payload (~10 g mounted): torque-sensor offset or
  gravity-model miscalibration, open. E1: pick frequencies/amplitudes with Λa ≫ 6 N.
