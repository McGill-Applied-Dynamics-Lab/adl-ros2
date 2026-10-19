# RIM under delay — plan

Conference presentation, ~2 weeks (from 2026-10-19). No user study: demonstrate the contribution on
the real FR3 + Haply Inverse3, 3-DoF translation, with contact on the table.

Methods and equations: [FORMULATION.md](FORMULATION.md). How to run things: [README.md](README.md).

## Claims and the evidence for each

| # | Claim | Evidence |
|---|---|---|
| C1 | Transparent free motion under delay | force felt in free motion vs. delay |
| C2 | Hard contact felt immediately, not one round trip late | force over time at contact, operator vs. robot; force onset vs. actual contact |
| C3 | Stable across the free → contact transition, also under variable delay | passivity energy, oscillation, guard trips; jitter / 5G conditions |
| C4 | Lower contact forces on the robot | peak table force, operator overshoot into the table |
| C5 | Little data exchanged | what crosses each link, per method (analytic table) |

**Assumption, stated in the talk:** the environment geometry is known (e.g. from a camera on the
remote side). No wall-mismatch experiment.

## Methods

| | Method | Role | Status |
|---|---|---|---|
| M1 | ZOH | model-free reference | exists |
| M2 | TDPA, two-port | passivity baseline for delayed links | **to implement** (today: one-port, inert) |
| M3 | God-object proxy | plain MMT: geometry only | **to decide** |
| M4 | Fixed-mass proxy | ablation: generic dynamics | exists |
| M5 | RIM, arm only | ours, today | exists |
| M6 | RIM, closed loop (arm + OSC) | ours: the controller in the model | **formulation to agree**, then implement |

Every method is validated (stage 3) before its delay results are used.

## Stages

### 0. Your pre-tests

Things to check on the robot before the full conditions. **To list.**

### 1. Formulation — agree on FORMULATION.md

- M6: massless target node, coupling $K, D$ independent of the OSC gains $K_r, D_r$.
- M2: which two-port TDPA (reference, PC types).
- M3: in or out.

### 2. Implementation

- M6: closed-loop integrator in `pyrim` (adl-python, on a branch; bump the submodule), then the
  method in `haptic_teleop`/`fr3_teleop` (sends the target, no feedforward). The model also carries
  $K_r, D_r$, read from the same parameters `fr3_teleop` sets on the controller.
- M2: two-port PO/PC, with energy carried on both links.
- M3: if kept, the projection proxy.
- Delays: 5G trace replay (`src/teleop/network_sim/data/5G-data.csv`, round trip → halved per link).

### 3. Validation, method by method (sim first, then robot)

| Method | Check |
|---|---|
| M1 ZOH | rendered force = formula; with no delay, the stiffest stable wall vs. the $K_{max} = 2b/T$ bound |
| M2 TDPA | an environment known to generate energy (negative damping, delayed stiff wall): observed energy stays ≥ 0, the PC dissipates exactly the deficit; energy balance across the link |
| M3/M4 | proxy trajectory vs. the analytic mass–spring (M4) / projection (M3) |
| M5/M6 | **prediction error**: the proxy (M5) or predicted robot (M6) vs. the measured robot, shifted by $T_c$. In sim (`SimFR3` is exactly Λ + OSC): ≈ 0 for M6. On the robot: the model-fidelity result |
| M5 vs M6 | sweep the OSC gains $K_r$: both agree at high $K_r$; M5's prediction error grows as $K_r$ drops, M6's should not |

### 4. Experiments

| | Task | Operator | Methods | Delays | For |
|---|---|---|---|---|---|
| E1 | press: approach, press, hold, retract | scripted (no hand) | all | 0, 50, 850, 2600 ms | C1, C2 |
| E2 | circle on the table | scripted (no hand) | all | same | C1, C2 (tangential) |
| E3 | press | scripted | M1, M2, M6 (+M5) | 5G trace; 850 ms ± jitter | C3 |
| E4 | press, circle, free task | you on the Inverse3 | M1, M2, M6 (+M5) | 0, 850, 2600 ms | C3, C4, video |

Delays are round trips, split evenly between the links. 50 ms is the local surface link (5G
replay is the realistic version), 850 ms the orbit-to-ground relay, 2.6 s the Earth–Moon link.

**Operators.**

- **Scripted, without the hand:** identical motion for every method. Used for validation, for C1/C2,
  and for bringing up long delays safely.
- **You on the Inverse3:** stability, contact force, the task, and the video.
- **The simulated hand (`--hand`):** sim exploration only. Its numbers depend on made-up grip parameters.

### 5. What to show

1. Side-by-side video at 2.6 s (ZOH vs. RIM).
2. One force-over-time figure: operator vs. robot, per method, at 850 ms and 2.6 s.
3. Metrics vs. delay, small multiples (free-motion force, error vs. an ideal table, peak robot force, stability).
4. Prediction error (M5 vs. M6) vs. OSC gain.
5. Summary table, and C5's data-per-link table.
6. One slide on assumptions and limits.

## Done so far (branch `feat/rim-delay-experiments`)

- `fr3_teleop`: 3-DoF translation (`--interface-axis xyz`, table on z), per-axis limits, `--run-name`.
- Scripted operator (`--device scripted --script press|circle`) and simulated hand (`--hand`).
- `SimFR3` + `sim_teleop.py`: the real haptic process, delay links and command loop against a simulated FR3.
- `run_conditions.py` (conditions grid, shuffled, resumable) and `analyze.py` (metrics + figures).
- Dev sim session, one repeat, `~/data/franka/rim-delays/sim/dev`. RIM's rendering does not
  degrade with delay; ZOH's free-motion force reaches 39 N at 2.6 s; one-port TDPA = ZOH.

## Open questions

1. Your pre-tests (stage 0).
2. M6: massless target node, OK? Held controller torques (orientation, null-space) in $\tilde f$ from the start, or measure first?
3. M2: two-port TDPA variant and reference.
4. M3 (god-object) in the ladder?
5. 5G replay for the 50 ms condition, or constant 50 ms plus a separate 5G experiment (E3)?
6. Jitter level for E3.

Data: `~/data/franka/rim-delays/<sim|robot>/<session>/`.
