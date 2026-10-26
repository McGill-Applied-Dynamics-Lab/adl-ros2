# Formulation — the methods compared under delay

One notation for every method, then each method as three answers:

1. **Haptic side** — what the operator feels, computed at 1 kHz.
2. **Robot side** — what `osc_controller` receives.
3. **Links** — what crosses each delayed link.

Status: **M1–M5 implemented** (M2: `tdpa2-zoh`, M3: `proxy-god-object`). M6's formulation is in progress.

---

## 1. Setup and notation

**Interface.** 3-DoF translation of the robot's tool point, in the robot base frame. The tool point
is the OSC task point. The table is the half-space $z \ge s$ ($s$ known, see §4).

| Symbol | Meaning |
|---|---|
| $x_l, v_l$ | leader (Inverse3 handle) position, velocity, in robot coordinates |
| $x, v$ | robot tool-point position, velocity (measured) |
| $f$ | force rendered to the operator (sign: the force the operator should feel) |
| $K, D$ | **virtual coupling**: stiffness, damping on the haptic side (chosen for the device) |
| $K_r, D_r$ | **OSC gains** on the translational axes (chosen for the robot) |
| $x_t, v_t, f_{ff}$ | what the OSC receives: target position, target velocity, feedforward force |
| $\lambda_c$ | table (contact) force on the robot |
| $T_c, T_f$ | delay haptic → robot (command), robot → haptic (feedback); round trip $T = T_c + T_f$ |

A quantity received over a link is the sender's value one delay ago, e.g. the haptic side sees
$x(t - T_f)$ and the robot sees $x_l(t - T_c)$. Between deliveries it is held.

**The robot under the OSC.** Inertia decoupling is off, so the gains are physical, in N/m and N·s/m:

$$
M\ddot q + c = J^\top F_{osc} + \tau_o + J^\top \lambda_c,
\qquad
F_{osc} = K_r\,(x_t - x) + D_r\,(v_t - v) + f_{ff}.
$$

$\tau_o$ collects everything else the controller does: orientation hold, null-space posture,
gravity and Coriolis compensation.

**Reduced to the interface (the RIM).** With $\Lambda = (J M^{-1} J^\top)^{-1}$, the controller force
lands on the interface unchanged, because $\Lambda J M^{-1} J^\top = I$:

$$
\Lambda\,\dot v = F_{osc} + \tilde f - z + \lambda_c,
\qquad
\tilde f = \Lambda J M^{-1}(\tau_o - c),\quad z = \Lambda \dot J \dot q .
$$

$\Lambda, z, \tilde f$ are computed on the robot side from $(q, \dot q)$ at the model update rate. They
are held between updates (and arrive $T_f$ late).

**Contact.** Unilateral and rigid, at $z = s$: the model's position is projected onto $z \ge s$ and
its velocity into the table is removed (`pyrim.RIMIntegrator`).

**Delays.** Each link is a latest-value delay line: an item sent at $t$ is delivered at
$t + d(t)$, and an item that arrives after a newer one is discarded, so arrivals never go back in
time. A delay condition is a round trip $T$, split evenly, $T_c = T_f = T/2$. Every method sees
the same delay sequence at a given condition: one seed per condition, $s$ on the feedback link and
$s + 1$ on the command link.

- **Constant** ($T$ = 0, 850 ms, 2.6 s): $d = T/2$ on each link.
- **5G**: a replay of the lab's 5G round-trip measurements (`src/teleop/network_sim/data/5G-data.csv`,
  condition 5G: four recordings from 2022, 21 000 samples, median 40 ms, p95 74 ms, p99 81 ms,
  max 637 ms). Each link replays the trace in order, from a start index drawn from its seed, one
  sample per 10 ms of send time, halved to a one-way delay:
  $d(t) = \tfrac12\,\tau_{k_0 + \lfloor (t - t_0)/10\,\text{ms} \rfloor}$. The two links read the
  trace from different starts, so the round trip is the sum of two independent halves, not one
  recorded round trip. The trace's sampling period is unknown and consecutive samples are
  uncorrelated, so the 10 ms period sets how fast the delay changes; it is a choice, stated as such.
  Packet loss is not modelled.

The rates at which each signal is sent, before its delay: haptic loop and targets
$f_h = f_t = 1$ kHz; robot state (M1, M2) 500 Hz; RIM model (M5) 25 Hz. A received value is held
until the next delivery.

---

## 2. Methods

### M1 — ZOH (position–position, no model) · *exists*

The OSC **is** the coupling: `fr3_teleop` sets $K_r := K$, $D_r := D$. The robot is pulled toward
the delayed leader, and the operator is pulled toward the delayed robot.

- Robot: $x_t = x_l(t - T_c)$, $v_t = v_l(t - T_c)$, $f_{ff} = 0$.
- Haptic: $f = K\,\big(x(t - T_f) - x_l\big) + D\,\big(v(t - T_f) - v_l\big)$ — the live leader against the last received robot state.
- Links: → $x_l, v_l$; ← $x, v$.

Under delay, free motion feels like a drag of about $K\,v\,T$, and contact arrives $T$ late.

### M2 — TDPA, two-port, position–position · *implemented: `tdpa2-zoh` (the one-port `tdpa-zoh` remains)*

M1 unchanged, plus a passivity observer/controller (PO/PC) at **each** end. The delayed channel,
together with both couplings, is treated as one two-port that must not generate energy
(Ryu et al. 2010 for time-varying delay; position–position as in Artigas et al.).
Position–position is chosen because the robot has no accurate force sensor. The robot-side power
uses the OSC's **computed** force $F_{osc}$ (the `task_wrench` it publishes), not a measurement.

**Port powers** (positive = energy flowing into the network):

- Operator port: $P_m = -f \cdot v_l$. The operator pushes against $f$.
- Robot port: $P_s = -F_{osc} \cdot v$. The robot is pushed by $F_{osc}$.

Each is integrated separately into an input energy $E^{in}$ (the integral of $\max(P, 0)$) and an
output energy $E^{out}$ (the integral of $\max(-P, 0)$). Both are cumulative, never decreasing.

**What crosses the links:** M1's signals, plus each side's cumulative input energy:
$E^{in}_m$ goes to the robot, $E^{in}_s$ comes back to the operator.

**Passivity observers** compare what a side has put out with what the other side has put in, as received:

$$
W_s(t) = E^{in}_m(t - T_c) - E^{out}_s(t) \ \ge 0,
\qquad
W_m(t) = E^{in}_s(t - T_f) - E^{out}_m(t) \ \ge 0 .
$$

With a time-varying delay, the received cumulative energy is simply the latest value delivered.
It never decreases, so late or superseded packets are harmless.

**Passivity controllers** dissipate a violation on the tick it happens. Both ports have impedance
causality (velocity in, force out), so both PCs are series dampers:

- Operator side: $f \leftarrow f - \alpha_m v_l$, with $\alpha_m = -W_m / (\Delta t\, \|v_l\|^2)$ when $W_m < 0$.
- Robot side: $F_{osc} \leftarrow F_{osc} - \alpha_s v$, with $\alpha_s = -W_s / (\Delta t\, \|v\|^2)$ when $W_s < 0$.
  It is applied as a feedforward wrench $f_{ff} = -\alpha_s v$. The classic alternative modifies
  the commanded position, which causes position drift; our OSC takes a wrench, so drift is avoided.
  To confirm against the reference.
- Both damping coefficients are capped, as in the current one-port. The unpaid part of a violation is
  carried to the next tick, and the dissipated energy is booked as output.

**Where it runs:** the operator end in the haptic process (1 kHz, `haptic_teleop.TDPA2Rendering`).
The robot end (`fr3_haptic.port.RobotPort`) observes every controller tick, *after* the command
delay, and its damping wrench goes out with the next target (command rate). On hardware that is this
client process, between the delayed link and `osc_controller`. `osc_controller`'s `task_wrench`
includes the feedforward wrench, so the robot end's own damping is booked by its observer.

**Damping caps:** `--tdpa-max-damping` (operator end, world N·s/m; the handle feels it × `force_gain`)
and `--tdpa-robot-max-damping` (robot end). First sim run (850 ms, scripted hand): the operator end
fires on most ticks at its cap (10 N·s/m), the robot end never (it receives far more than it puts
out). The caps decide how conservative M2 is: to choose in T5 on the device.

Today's `tdpa-zoh` is a one-port PO/PC at the device only. In the dev sim it never fired and equalled ZOH.

### M3 — God-object proxy (MMT, geometry only) · *implemented: `proxy-god-object`*

The local model is the table geometry alone. A massless proxy is the leader projected onto free space.

- Haptic: $x_p = \Pi_{z \ge s}(x_l)$ (the projection); $f = K\,(x_p - x_l)$ (+ damping in contact).
- Robot: $x_t = x_p(t - T_c)$, $v_t = v_p(t - T_c)$, $f_{ff} = -f$ — the operator's push, applied by the robot.
- Links: → $x_p, v_p, f_{ff}$; ← nothing. Robot state is used for monitoring only.

This is the zero-mass limit of M4.

### M4 — Fixed-mass proxy (MMT with generic dynamics) · *exists (`proxy-fixed-mass`)* · ablation

The same proxy machinery as RIM, with a constant mass $m$ in place of the robot's reduced dynamics.

- Haptic:
  $$m\,\dot v_p = K\,(x_l - x_p) + D\,(v_l - v_p) + \lambda_c(x_p),\qquad f = K\,(x_p - x_l) + D\,(v_p - v_l).$$
- Robot: $x_t = x_p(t - T_c)$, $v_t = v_p(t - T_c)$, $f_{ff} = -f(t - T_c)$.
- Links: → $x_p, v_p, f_{ff}$; ← nothing. The mass is fixed; the proxy is seeded once, from the leader.

This isolates what the robot's own model ($\Lambda, z, \tilde f$) buys over a plausible constant inertia.

### M5 — RIM, arm only · *exists (`proxy-rim`)*

The proxy is the arm's reduced dynamics, coupled to the leader by the virtual coupling. The
controller is not in the model: the robot is assumed to track the proxy exactly ($K_r \to \infty$).

- Haptic:
  $$\Lambda\,\dot v_p = K\,(x_l - x_p) + D\,(v_l - v_p) + f_{eff} - z + \lambda_c(x_p),\qquad f = K\,(x_p - x_l) + D\,(v_p - v_l).$$
  Here $f_{eff} = -\Lambda J M^{-1} c$: gravity is compensated, and no controller term is included.
- Robot: $x_t = x_p(t - T_c)$, $v_t = v_p(t - T_c)$, $f_{ff} = -f(t - T_c)$. The feedforward supplies the force; the OSC PD only rejects model error.
- Links: → $x_p, v_p, f_{ff}$; ← the model $(\Lambda, z, f_{eff})$ at the model update rate.

#### Orientation held · *implemented: `rim_hold_orientation` (`ModelConfig.hold_orientation`)*

The free reduction assumes the only force on the arm acts at the interface point. On the robot the
OSC also holds the end effector's orientation, with a spring (k_rot) whose rotational modes are at
~3.6–11 Hz: below them the hand barely turns, and the interface point moves as if the orientation
were constrained. With the rotation rows $J_h$ of the end-effector Jacobian ($J_h \ddot q + b_h = 0$,
$b_h = \dot J_h \dot q$) stacked under the interface rows, $J = [J_i; J_h]$:

$$\Lambda_6 = (J M^{-1} J^T)^{-1},\quad M_{eff} = [\Lambda_6]_{ii},\quad
z_i = [\Lambda_6 (J M^{-1} c - b)]_i,\quad f_{eff} = [\Lambda_6 J M^{-1} f_{ext}]_i .$$

Holding only adds inertia ($M_{eff}^{held} \succeq M_{eff}^{free}$). At (0.4, 0, 0.2) m:
$\Lambda_{xx}$ 1.69 → 10.55 kg, eigenvalues (0.87, 1.95, 9.41) → (4.79, 5.21, 14.46) kg. Measured on
the robot: a 0.5 Hz circle follows the held model in x (radial gap 7.4 → 1.2 N); 2–4 Hz sines sit
near the free one (T1) — the rotation is held only below the rotational modes. It is the limit
$k_{rot} \to \infty$ of M6's orientation part below.

#### 3-DoF contact projection (M4, M5) · *implemented in `pyrim.RIMIntegrator`*

The wall $n^T x_p \ge s$ ($n = e_z$) pushes along $n$ only, so its impulse $p \ge 0$ moves the
proxy along $\Lambda^{-1} n$, which is not along $n$ when $\Lambda$ couples axes. One step, with
the velocity implicit in the coupling and $A = \Lambda + \Delta t\,(D + \Delta t\,K)\,I$:

$$A\,v^* = \Lambda\,v + \Delta t\,\tilde r \quad\text{(free step)},\qquad
v^+ = v^* + A^{-1} n\,p,\qquad x^+ = x + \Delta t\,v^+,$$

$$p = \max\Big(0,\ \frac{-\max(g, 0)/\Delta t - n^T v^*}{n^T A^{-1} n}\Big),\qquad g = n^T x - s.$$

The proxy lands exactly on the wall; equivalently $v^+$ is the point nearest $v^*$ in the
$A$-metric with $n^T v^+ \ge -\max(g,0)/\Delta t$ (Gauss's principle). At rest in contact
$\Delta t\,\tilde r + n\,p = 0$: the tangential force vanishes and $f$ is normal (a frictionless
table). A proxy that starts inside the wall ($g < 0$) only stops moving in, and its position is
moved back along $A^{-1}n$ without a velocity change, so no energy is injected.

The per-axis projection used before (clamp $x_z$, $v_z \ge 0$) is the identity-metric one. At rest
it requires $(A^{-1}\tilde r)_x = 0$, i.e. a static tangential force
$r_x \approx -(\Lambda^{-1})_{xz}/(\Lambda^{-1})_{xx}\, r_z$: $-0.29\,r_z$ with $\Lambda_0$ at
$(0.4, 0, 0.2)$ m, about $-6$ N felt sideways in a 20 N vertical press (measured on the robot,
2026-10-23: $-6.8$ N, proxy 6.9 mm off the leader). M4 ($A \propto I$) and M3 (massless, isotropic
$K$) are unaffected: $A^{-1}n \parallel n$.

### M6 — RIM, closed loop (arm + OSC) · *proposal*

The RIM is the robot **under its controller**, seen from the controller's input (the target). The
virtual coupling attaches the operator to that target. The target is a massless node between two
spring–dampers:

```
operator x_l ─(K, D)─ target x_t ─(K_r, D_r: the OSC)─ robot x̂ (Λ, z, f̃) ─ table
```

- Haptic: integrate the predicted robot $\hat x$ and the target $x_t$ together:
  $$
  \underbrace{K\,(x_l - x_t) + D\,(v_l - v_t)}_{F} \;=\; K_r\,(x_t - \hat x) + D_r\,(v_t - \hat v)
  \qquad\text{(force balance at the massless target)}
  $$
  $$
  \Lambda\,\dot{\hat v} = F + \tilde f - z + \lambda_c(\hat x),
  \qquad f = -F .
  $$
- Robot: $x_t(t - T_c)$, $v_t(t - T_c)$, $f_{ff} = 0$. The real OSC, with its own $K_r, D_r$, produces $F$ itself.
- Links: → $x_t, v_t$; ← the model $(\Lambda, z, \tilde f)$ and the gains $K_r, D_r$.

Properties:

- **$K, D$ and $K_r, D_r$ are independent.** Lower $K_r$: the robot trails the target more, and
  the model predicts it. Higher $K_r$: tighter tracking.
- **The limit $K_r \to \infty$ gives $x_t = \hat x$, which is M5.** M5 is the perfect-tracking case of M6.
- **Force matching in contact:** the table force on the robot is $K_r(x_t - \hat x) = F$,
  exactly what the operator feels. The felt stiffness is the series stiffness $K K_r / (K + K_r)$:
  the true compliance of the remote side.
- **No feedforward is needed.** The target leads the robot by exactly what the real controller
  needs to produce $F$.
- **Passive:** a network of springs, dampers, a mass and a unilateral constraint, integrated
  implicitly. The exception, as in M5, is the held $\tilde f$.

Discretization: per step and per axis, the unknowns $(\hat v_{k+1}, x_{t,k+1})$ satisfy a 2 × 2
linear system (implicit spring–dampers), followed by the contact projection on $\hat x$. With
$D + D_r > 0$, the target is a first-order node:
$v_t = \big[K(x_l - x_t) + D v_l - K_r(x_t - \hat x) + D_r \hat v\big] / (D + D_r)$.

---

## 3. Side by side

| | Model on the haptic side | Operator feels | Robot receives | ← link (robot → haptic) | OSC gains |
|---|---|---|---|---|---|
| M1 ZOH | none | spring to the delayed robot | delayed leader | $x, v$ | $= K, D$ |
| M2 TDPA | none (+ PO/PC) | M1, passivity-corrected | delayed leader + PC damping wrench | $x, v$, energy | $= K, D$ |
| M3 God-object | geometry | spring to the projected leader | proxy + push | — | own |
| M4 Fixed mass | geometry + mass $m$ | spring to the proxy | proxy + coupling force | — | own |
| M5 RIM arm | geometry + $\Lambda, z, f_{eff}$ | spring to the proxy | proxy + coupling force | model | own (assumed ∞) |
| M6 RIM closed loop | geometry + $\Lambda, z, \tilde f$ + $K_r, D_r$ | spring to the target | target | model, gains | own (modelled) |

M3 → M4 → M5 → M6 add one thing to the local model at each step: mass, then the robot's inertia,
then its controller. M1 and M2 are the model-free references.

---

## 4. Assumptions (all methods)

- **Known geometry.** The table plane $z = s$ is known exactly, as it could be from a camera on the
  remote side. Not estimated online.
- **Rigid table** in the models. The real table and tool have some compliance; on the robot that is
  model error.
- **OSC below its limits** (force 80 N, torque, torque rate `delta_tau_max`). Saturation is not modelled.
- **Held terms.** $\Lambda, z, \tilde f$ are constant between model updates. Joint friction is unmodelled
  (the stick-slip in `experiments/benchmarks/osc_controller/FINDINGS.md`) — the main expected source
  of prediction drift for M5/M6 on the real arm.
- **Delays.** Each link is a latest-value delay line: a newer sample supersedes older ones still
  in flight. Round-trip conditions are split evenly, $T_c = T_f = T/2$.
