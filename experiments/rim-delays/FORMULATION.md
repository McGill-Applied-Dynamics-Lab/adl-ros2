# Formulation — the methods compared under delay

One notation for every method, then each method as three answers:

1. **Haptic side** — what the operator feels, computed at 1 kHz.
2. **Robot side** — what `osc_controller` receives.
3. **Links** — what crosses each delayed link.

Status: **M1–M5 implemented** (M2: `tdpa2-zoh`, M3: `proxy-god-object`, M4: `proxy-fixed-mass`, M5:
`proxy-rim`). M6 is a proposal. Merged on 2026-10-27 with the online formulation of 2026-10-23 (the
inertia section and the M1–M4 derivations come from it).

---

## 1. Setup and notation

**Interface.** 3-DoF translation of the robot's tool point, in the robot base frame. The tool point
is the OSC task point. Orientation is held by the OSC and is not teleoperated. Haptic-side models use
a set of known planes (see Contact, §2); the experiments use the table half-space $z \ge s$ alone
($s$ known, §7).

| Symbol | Meaning |
|---|---|
| $x_l, v_l$ | leader (Inverse3 handle) position, velocity, in robot coordinates |
| $x, v$ | robot tool-point position, velocity (measured) |
| $f$ | force rendered to the operator (sign: the force the operator should feel) |
| $K, D$ | **virtual coupling**: stiffness, damping on the haptic side (chosen for the device) |
| $K_r, D_r$ | **OSC gains** on the translational axes (chosen for the robot) |
| $x_t, v_t, f_{ff}$ | what the OSC receives: target position, target velocity, feedforward force |
| $x_p, v_p$ | proxy position, velocity (M3–M5) |
| $\lambda_c$ | real table (contact) force on the robot |
| $\hat\lambda$ | table force inside a haptic-side model |
| $T_c, T_f$ | delay haptic → robot (command), robot → haptic (feedback); round trip $T = T_c + T_f$ |

**Leader.** The handle at device position $p_d$ is mapped into robot coordinates, and the rendered
force is mapped back, with position scale $\alpha$ and force gain $\beta$:

$$
x_l = \alpha R\,p_d + o,\qquad v_l = \alpha R\,\dot p_d,\qquad f_d = \beta R^\top f .
$$

The device sees a stiffness $\alpha\beta K$ for a coupling stiffness $K$. The power exchanged with the
operator is $f_d \cdot \dot p_d = (\beta/\alpha)\, f \cdot v_l$. Since $\alpha, \beta > 0$, passivity
with respect to the operator is equivalent to passivity of the $(f, v_l)$ port, so all energy
bookkeeping is done in robot units. The offset $o$ is chosen so that $x_l(0) = x(0)$, and every proxy
is seeded there with zero velocity. Frozen: $\alpha = 1$, $\beta = 0.2$ (EXP_PLAN.md).

**Links and delays.** The command link (subscript $c$) carries samples at $f_t$ with delay
$T_c(t)$; the feedback link (subscript $f$) carries samples at $f_m$ with delay $T_f(t)$. Each is a
latest-value line: an item sent at $t$ is delivered at $t + d(t)$, and an item that arrives after a
newer one is discarded, so arrivals never go back in time. Between deliveries the value is held.
$\langle y\rangle_c$ and $\langle y\rangle_f$ denote the value of $y$ held at the receiver. With send
instants $t_k = k/f$:

$$
\langle y\rangle(t) = y(t_{k^*}),\qquad k^* = \max\{\,k : t_k + T(t_k) \le t\,\} .
$$

For a constant delay, $k^* = \lfloor (t - T) f \rfloor$, and the age of the held sample lies in
$[T, T + 1/f)$: the delay plus up to one period of hold. Under variable delay the age is bounded by
the largest delay in play plus the period. $\langle\cdot\rangle$ covers delay, sample-and-hold,
reordering and loss in one symbol, which is why delay and rate mismatch are treated as one mechanism.

**Delay conditions.** A round trip $T$, split evenly, $T_c = T_f = T/2$. Every method sees the same
delay sequence at a given condition: one seed per condition, $s$ on the feedback link and $s + 1$ on
the command link.

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

**Rates.**

| Symbol | Loop | Value |
|---|---|---|
| $f_h$ | haptic rendering | 1 kHz |
| $f_r$ | OSC | 1 kHz |
| $f_t$ | targets, haptic → robot | $= f_h$ |
| $f_m$ | robot → haptic | robot state (M1, M2) 500 Hz; RIM model (M5) 25 Hz |

---

## 2. The robot

**Plant.** The FR3, with $J$ the 3×7 translational Jacobian at the tool point:

$$
M(q)\,\ddot q + C(q,\dot q)\,\dot q + g(q) = \tau + J^\top \lambda_c .
$$

**Controller.** The OSC runs at $f_r$ without inertia decoupling, so its gains are physical (N/m,
N·s/m). $\tau_{ori}$ is the orientation hold, $\tau_{ns}$ the null-space posture task. Gravity is
compensated by libfranka; Coriolis is compensated by `osc_controller` (`control.coriolis_compensation:
true` in `osc_teleop.yaml`):

$$
\tau = J^\top F_{osc} + \tau_o,\qquad
F_{osc} = K_r\,(x_t - x) + D_r\,(v_t - v) + f_{ff},\qquad
\tau_o = g(q) + C\dot q + \tau_{ori} + \tau_{ns} .
$$

Without force sensing, inertia decoupling would only reshape free motion: in contact the remote
stiffness would become $\Lambda K_r'$, anisotropic and configuration-dependent, and the interaction
port would lose passivity. Physical gains keep every method a network of springs, dampers and masses.

**Reduced to the interface (the RIM).** In pyrim's convention (`RIMCalculator`), bias on the left,
Khatib-style. Let $c$ collect the plant's bias torques. From $M\ddot q + c = J^\top(F_{osc} + \lambda_c)
+ \tau_o$ and $\dot v = J\ddot q + \dot J\dot q$, with $\Lambda = (J M^{-1} J^\top)^{-1}$:

$$
\Lambda\,\dot v + z_i = F_{osc} + \lambda_c + f_{eff},\qquad
z_i = \Lambda\big(J M^{-1} c - \dot J \dot q\big),\qquad
f_{eff} = \Lambda J M^{-1} \tau_o .
$$

The controller force lands on the interface unchanged, because $\Lambda J M^{-1} J^\top = I$.

| Formulation | pyrim (`DynModel`, `RIMModel`) | Meaning |
|---|---|---|
| $\Lambda$ | `M_eff` | interface inertia |
| $J$ | `J_i` | interface Jacobian (3×7, translational) |
| $\dot J \dot q$ | `b_i` | Jacobian curvature term |
| $c$ | `c` | plant bias torques |
| $\tau_o$ | `f_ext` | controller's non-task torques |
| $z_i, f_{eff}$ | `z_i`, `f_eff` | reduced bias and reduced controller force |

**What the code builds** (`fr3_haptic.adapters.ModelEstimatorAdapter`, live path):
$c = C\dot q$ (`nle − g`: gravity left out, since it cancels against the compensation) and
`f_ext = None`, so $f_{eff} = 0$. The model therefore keeps $C\dot q$ in $z_i$, while the OSC
cancels it: see §8. $\tau_{ns}$ is not modelled (a stiffness-like force, consistent with the
30–60 N/m E1 measured).

**Contact.** The real table is rigid with friction: $\lambda_c = \lambda_n e_z$ plus friction,
$0 \le \lambda_n \perp (x_z - s) \ge 0$. Haptic-side models use a known set of rigid, frictionless
planes $n_i^\top x \ge s_i$, collected in $N = [n_1 \dots n_{n_c}]$ and $s$; the experiments use the
table alone ($n_c = 1$, $n = e_z$). The free region is $\mathcal C = \{y : N^\top y \ge s\}$, and
complementarity holds componentwise:

$$
\hat\lambda = N\hat\lambda_n,\qquad 0 \le \hat\lambda_n \perp \big(N^\top x_p - s\big) \ge 0 .
$$

---

## 3. The interface inertia Λ

$\Lambda$ is the arm's inertia as felt at the tool point: at an instant, ignoring velocity terms and
the controller, the force needed for a tool acceleration $a$ is $F = \Lambda a$. Away from
singularities it is symmetric positive definite:

$$
\Lambda = U\,\mathrm{diag}(\lambda_1, \lambda_2, \lambda_3)\,U^\top .
$$

The eigenvectors $u_i$ are the directions in which force and acceleration are parallel; the
eigenvalues $\lambda_i$ are the effective masses (kg) along them, the semi-axes of the inertia
ellipsoid. In any other direction, force and acceleration are not parallel.

**Anisotropy and coupling.** Two properties are easily confused:

- *Anisotropy* concerns the eigenvalues: whether all directions are equally heavy. It is
  frame-independent, and only $\Lambda = mI$ is isotropic. Its shape is measured by
  $\kappa = \lambda_{max}/\lambda_{min}$, its size by the spread $\lambda_{max} - \lambda_{min}$ in kg.
  The spread sets force errors: a scalar mass tuned to the mean is wrong by about
  $\tfrac12(\lambda_{max} - \lambda_{min})\,a$ in the worst direction.
- *Off-diagonal terms* concern the eigenvectors relative to the chosen axes. A diagonal $\Lambda$ can
  be strongly anisotropic, e.g. diag(2, 2, 10) kg.

Force and acceleration are not parallel whenever the acceleration is off the eigenvectors, which
requires only anisotropy; off-diagonal terms only say that the eigenvectors are rotated relative to
the world axes.

**Effective mass along a direction n.** Two notions, both between $\lambda_{min}$ and
$\lambda_{max}$, both equal to $\lambda_i$ along $u_i$, with $m_F(n) \le m_a(n)$:

$$
m_a(n) = n^\top \Lambda\, n,\qquad m_F(n) = \frac{1}{n^\top \Lambda^{-1} n} .
$$

- $m_a$, imposed motion along $n$: the force per unit acceleration when the motion is held on the line
  (which needs lateral forces too). The E1 and E4 situation, and why M4 is tuned to $n^\top\Lambda_0 n$.
- $m_F$, force applied along $n$ only: what an impulse along $n$ sees, e.g. the table pushing along
  its normal, the inverse of the Delassus term in the contact LCP. It sets the impact the robot
  experiences in E3.

**Configuration dependence.** $\Lambda$ depends on $q$: the ellipsoid changes size, shape and
orientation as the arm moves. Near a singularity $J$ loses rank, $\Lambda^{-1}$ becomes singular, and
the eigenvalue along the singular direction grows without bound: the arm cannot be accelerated that
way, and a proxy carrying $\Lambda$ feels locked.

**What Λ sets.**

| Quantity | Expression | Where it matters |
|---|---|---|
| OSC natural frequency per direction | $\omega_i = \sqrt{K_r/\lambda_i}$ | robot response (no inertia decoupling) |
| OSC damping ratio per direction | $\zeta_i = D_r/(2\sqrt{K_r \lambda_i})$ | $\lambda_{max}$ is the least damped: sets $D_r$ |
| Tracking lag under acceleration | $\lambda_i a / K_r$ | the force M3 leaves unfelt |
| M4's free-motion error | $(\Lambda - mI)\,a$ | E1, E2, E4 |
| E2 error amplitude ($m$ at the mean) | $\approx \tfrac12(\lambda_{max} - \lambda_{min})\,a$, at twice the circle frequency | E2 |
| Impact momentum along the normal | $m_F(n)\,v$ | E3 transients |
| Tangential velocity change at impact | misalignment of $\Lambda^{-1}n$ with $n$ | contact step (M5) |
| Relative change per model update | $\lVert\Delta\Lambda\rVert/\lVert\Lambda\rVert$ over $1/f_m$ | choice of $f_m$ |
| Kinetic energy change per update | $\tfrac12 v^\top \Delta\Lambda\, v$ | update rule ($v$ kept) |

**Which Λ: the orientation is held.** With the orientation free, the translational inertia is
$\Lambda_3 = (J_v M^{-1} J_v^\top)^{-1}$. With the orientation held rigidly, it is the translational
block of the 6-DoF $\Lambda_6$. Since $\Lambda_3^{-1}$ is the translational block of $\Lambda_6^{-1}$,
a Schur complement gives

$$
\Lambda_3 = \Lambda_{6,vv} - \Lambda_{6,v\omega}\,\Lambda_{6,\omega\omega}^{-1}\,\Lambda_{6,\omega v}
\;\preceq\; \Lambda_{6,vv} :
$$

holding the orientation makes the tool at least as heavy in every direction. At (0.4, 0, 0.2) m:
$\Lambda_{xx}$ 1.69 → 10.55 kg, eigenvalues (0.87, 1.95, 9.41) → (4.79, 5.21, 14.46) kg.

The arm under its controller is a closed-loop plant whose apparent inertia depends on frequency and
on how stiffly each rotation is held, between two asymptotes: well below the rotational modes the
orientation is held and the tool feels $\Lambda_{6,vv}$; well above them the wrist cannot follow the
controller and the tool feels $\Lambda_3$. With the tool point imposed (the proxy drives it), the
rotational modes are the eigenfrequencies of $\Lambda_{6,\omega\omega}^{-1} K_{rot}$, with $K_{rot}$
= diag(85, 265, 10) N·m/rad: **3.6, 4.7 and 11.4 Hz** at the start (0.4, 0, 0.2), 3.7, 4.6 and
11.4 Hz at the E2 circle's centre, 2.6, 5.6 and 11.4 Hz at the E6 contact (0.4, 0, 0.083). Yaw, held
softly at 10 N·m/rad, is the *highest* (11.4 Hz): it turns only the light wrist. (The earlier √K_rot
estimate, yaw ≈ 0.8 Hz, assumed comparable rotational inertias and is superseded.)

Measured: on the first E2 circle (0.5 Hz, 0.32 m/s, 1.05 m/s²) $\Lambda_3$ gave $\Lambda_{xx}$ ≈ 1.4 kg
at the circle centre while the arm behaved like about 9.7 kg, the held value; with the held model the
radial gap fell from +7.4 to +1.2 N (along x, +10.8 → +2.6 N). 2–4 Hz sines (T1) sit near the free
one. The free-motion experiments therefore stay at or below 1 Hz.

**Ablation (not run): a frozen-Λ proxy.** M5 with $\Lambda_0$ held and never updated. A scalar $m$
fails on anisotropy; a frozen $\Lambda_0$ captures anisotropy and fails only on configuration
change. Between M4 and M5 it separates the two: E2 should show it matching M5, E4 failing.

---

## 4. Methods

### M1 — ZOH (position–position, no model) · *implemented: `zoh`*

The OSC **is** the coupling: `fr3_teleop` sets $K_r := K$, $D_r := D$. Each side couples to the latest
received state of the other.

- Haptic (at $f_h$): $f = K\big(\langle x\rangle_f - x_l\big) + D\big(\langle v\rangle_f - v_l\big)$.
- Robot (at $f_r$): $x_t = \langle x_l\rangle_c$, $v_t = \langle v_l\rangle_c$, $f_{ff} = 0$.
- Links: → $x_l, v_l$ at $f_t$; ← $x, v$ at $f_m$.

Remarks.

- **Symmetric gains.** With $T = 0$ and the robot static on the table, the operator feels $K(x - x_l)$
  and the robot pushes with $K_r(x_l - x)$: equal and opposite only if $K_r = K$, which gives force
  transparency in steady contact. The price: the robot's stiffness is set by what the Inverse3 can
  render ($\alpha\beta K$), not by the task.
- **Zero-delay limit.** With $T = 0$ and $f_m < f_h$, M1 is the RA-L rate-mismatch baseline: the haptic
  loop renders against a state held for $1/f_m$. Delay and rate mismatch are one mechanism, a stale
  held sample, at two scales.
- **Free motion under delay.** If the robot tracks its delayed target, $x(t - T_f) \approx x_l(t - T)$:
  the operator is pulled toward where the leader was a full round trip ago. At constant velocity the
  $D$ term cancels and $f \approx -K T v$. This is velocity-proportional, like a drag of coefficient
  $KT$ (2600 N·s/m at $K$ = 1000 N/m, $T$ = 2.6 s), but it is a stretched spring, not a damper: when
  the operator stops, the force persists for about $T$. For a sine of amplitude $A$, the unfelt force
  has RMS $K\cdot 2A\lvert\sin(\omega T/2)\rvert/\sqrt2$, periodic in $T$ (E1). Imperfect tracking
  adds a lag of about $\Lambda a/K_r$.
- **Transient damping.** When the velocity changes, $D$ acts on a velocity difference that is $T$
  stale, so even the damper is not reliably dissipative under delay.
- **Contact under delay.** The robot reaches the table at $t_0$, but the operator first feels it at
  $t_0 + T_f$ or later. Meanwhile the target keeps advancing, so the robot presses with $K_r$ times
  the overshoot; when the information arrives, the operator gets a force proportional to their own
  penetration, about $vT$. The delayed channel generates energy: M1 is not passive, and its
  stability depends on $K$, $D$ and $T$.

### M2 — TDPA, two-port, position–position · *implemented: `tdpa2-zoh` (the one-port `tdpa-zoh` remains)*

M1 unchanged, plus a passivity observer/controller (PO/PC) at **each** end (Ryu et al. 2010 for
time-varying delay; position–position as in Artigas et al.). The network is everything between the two
ports: the haptic coupling, both delayed links with their holds, and the OSC coupling. Both ports have
impedance causality (velocity in, force out). PC damping is written $b$, since $\alpha$ is the
position scale.

| Port | Flow | Effort from the network | Power into the network |
|---|---|---|---|
| Operator | $v_l$ | $f$ | $P_m = -f \cdot v_l$ |
| Robot | $v$ | $F_{osc}$ | $P_s = -F_{osc} \cdot v$ |

The robot port uses the OSC's **computed** force (the `task_wrench` it publishes), not a measurement:
passivity is guaranteed for the network as the controller computes it, while the OSC stays below its
limits.

**Energies.** Each port keeps cumulative input and output energies,
$E^{in} = \sum \max(P, 0)\Delta t$ and $E^{out} = \sum \max(-P, 0)\Delta t$, both nondecreasing.

**Links:** M1's signals, plus each side's cumulative input energy: $E^{in}_m$ with the targets (→),
$E^{in}_s$ with the state (←). Since cumulative energy never decreases, latest-value delivery is safe:
a late or superseded packet can only carry a smaller value.

**Observers.** Each side compares what it has put out with what the other side has put in, as received:

$$
W_s = \langle E^{in}_m\rangle_c - E^{out}_s,\qquad W_m = \langle E^{in}_s\rangle_f - E^{out}_m .
$$

**Operator end** (each haptic tick $k$, $\Delta t_h = 1/f_h$):

$$
\begin{aligned}
f^0_k &= K\big(\langle x\rangle_f - x_{l,k}\big) + D\big(\langle v\rangle_f - v_{l,k}\big)\\
W^-_k &= \langle E^{in}_s\rangle_f - E^{out}_{m,k-1} - \max\!\big(f^0_k\cdot v_{l,k},\,0\big)\,\Delta t_h\\
b_{m,k} &= \min\!\Big(b_m^{max},\ \frac{\max(-W^-_k,\,0)}{\Delta t_h\,\lVert v_{l,k}\rVert^2}\Big)
\quad (0 \text{ if } \lVert v_{l,k}\rVert \text{ is below a threshold}),\qquad
f_k = f^0_k - b_{m,k}\, v_{l,k}
\end{aligned}
$$

The energies are then updated with the corrected power $P_{m,k} = -f_k \cdot v_{l,k}$.

**Robot end** (each OSC tick $j$, $\Delta t_r = 1/f_r$): the same structure, with
$F^0_j = K_r(x_t - x_j) + D_r(v_t - v_j)$, the received $\langle E^{in}_m\rangle_c$ and damping $b_{s,j}$:

$$
F_{osc,j} = F^0_j - b_{s,j}\,v_j \quad\Longleftrightarrow\quad f_{ff,j} = -b_{s,j}\,v_j .
$$

**Closed bookkeeping.** Booking with the corrected force means the PC's dissipation shows up as
reduced output; nothing is booked separately. Without a cap, the correction brings $W$ to exactly
zero and never flips the sign of the port power. With a cap, the unpaid deficit stays in $W$ and the
PC keeps acting on later ticks.

**Passivity.** If neither cap binds, $W_m, W_s \ge 0$ at every tick. Summing them and using
$E^{in}(t) \ge \langle E^{in}\rangle$:

$$
E^{in}_m + E^{in}_s - E^{out}_m - E^{out}_s \ge 0 .
$$

The network is passive for any time-varying delay and any hold, including the $1/f_m$ hold. While a
cap binds the guarantee is suspended: log the fraction of ticks at the cap and the minimum of $W$.

Remarks.

- **Stability, not transparency.** The PO acts only on output the received input cannot cover. The
  free-motion drag $-KTv$ is input ($P_m > 0$), so it is untouched. At contact the operator still
  penetrates by about $vT$; what the PC removes is the energetic rebound.
- **Conservatism of the split.** $W_m$ credits the operator end only with energy that came from the
  robot side: energy stored in the local coupling and returned counts as a violation. This matches the
  first sim run at 850 ms (operator end at its cap on most ticks, robot end never). The
  energy-reflection variant (TDPA-ER, Panzirsch and Singh, RA-L 2021, P-P version) removes this
  conservatism without position drift.
- **No position drift.** The PC modifies the force, not $x_t$; the cost is a transient lag.
- **Damping as felt.** $b_m$ is in robot units; the handle feels $\alpha\beta\, b_m$. Caps:
  `--tdpa-max-damping` (operator end) and `--tdpa-robot-max-damping` (robot end).

**Implementation (and a known deviation).** The operator end runs in the haptic process (1 kHz,
`haptic_teleop.TDPA2Rendering`). The robot end (`fr3_haptic.port.RobotPort`) observes every controller
tick after the command delay, but its damping wrench goes out **with the next target**. That violates
the constraint the derivation needs: the PC must act on the tick it observes, with the current
velocity. A precomputed wrench $-b_s v_j$ applied at $v_{j+1}$ has power $b_s\, v_j \cdot v_{j+1}$,
not guaranteed positive, and injects energy on a velocity reversal. Fix before M2 is used: run the PC
inside the OSC tick, or send $b_s$ and let `osc_controller` apply $-b_s v$. The observer stays exact
as long as it books the applied `task_wrench`. M2 is not in the talk.

### M3 — God-object (massless proxy, geometry only) · *implemented: `proxy-god-object`*

The local model is the table geometry alone. The proxy is the point of the free region closest to the
leader in the coupling's metric (the point that minimizes the coupling energy). Its velocity is the
leader's, projected in the same metric onto the directions that keep the active constraints
$\mathcal A$ satisfied:

$$
x_p = \arg\min_{y \in \mathcal C}\ \tfrac12 (y - x_l)^\top K\,(y - x_l),\qquad
v_p = \arg\min_{u\,:\ n_i^\top u \ge 0,\ i \in \mathcal A}\ \tfrac12 (u - v_l)^\top K\,(u - v_l),
$$

$$
f = K\,(x_p - x_l) + D\,(v_p - v_l),\qquad f \leftarrow \Pi_{\mathrm{cone}(N_{\mathcal A})}(f) .
$$

With one plane and isotropic $K$: $x_p = x_l + \max(0, s - n^\top x_l)\, n$. In free space $x_p = x_l$,
$v_p = v_l$ and $f = 0$: the damping acts only in contact, with no switch. The final projection keeps
the wall unilateral (with one plane, the clamp $n^\top f \ge 0$); without it the damper pulls the
handle back in on a fast exit, a sticky wall.

- Robot: $x_t = \langle x_p\rangle_c$, $v_t = \langle v_p\rangle_c$, $f_{ff} = -\langle f\rangle_c$ — the operator's push, applied by the robot.
- Links: → $x_p, v_p, f$; ← nothing. Robot state is used for monitoring only.

Remarks.

- **Free motion.** The operator feels nothing, while the robot, driven by its PD alone, needs about
  $\Lambda a$ and lags by about $\Lambda a/K_r$: $e_F \approx \Lambda a$, independent of delay.
- **Contact.** The onset is immediate, since the geometry is local, and the rendered stiffness is $K$.
  The delay only shifts when the robot executes: in steady contact it presses with $K \times$ depth,
  $T_c$ late. At impact the target velocity drops to zero while the robot still carries about
  $m_F(n)\,v$ of momentum; that impact is never rendered.
- **Passivity.** A convex potential plus a damper (the clamp only reduces returned energy): passive,
  and the delay does not enter it. In sampled data the virtual-wall condition applies (Colgate and
  Schenkel): the device's physical damping must exceed about $K\Delta t/2 + D$. That condition, not
  the delay, bounds $K_{max}$ in E5.
- **No feedback.** Nothing flows back, so no delayed loop can destabilize; nothing corrects the model
  either. If the table is elsewhere, the operator never learns it.
- It is the massless, undamped limit of M4: $K(x_l - x_p) + N\hat\lambda_n = 0$ with complementarity
  is exactly the optimality condition of the projection.

### M4 — Fixed-mass proxy · *implemented: `proxy-fixed-mass`* · ablation

The same proxy machinery as RIM, with a constant mass in place of the robot's reduced dynamics.
$M_p = mI$ (M5 replaces it with $\Lambda$); $K$ and $D$ are isotropic in the experiments:

$$
M_p\,\dot v_p = K\,(x_l - x_p) + D\,(v_l - v_p) + N\hat\lambda_n,\qquad f = K\,(x_p - x_l) + D\,(v_p - v_l) .
$$

**Discretization** (each haptic tick $k$, $\Delta t = 1/f_h$), as in `pyrim.RIMIntegrator`. Implicit
Euler on the coupling, with the leader extrapolated one step:

$$
A = M_p + \Delta t\,D + \Delta t^2 K,\qquad
v^* = A^{-1}\big[M_p\,v_k + \Delta t\,K\,(\hat x_l - x_k) + \Delta t\,D\,v_{l,k}\big],\qquad
\hat x_l = x_{l,k} + \Delta t\,v_{l,k} .
$$

Contact is a velocity-level LCP on the start-of-step gaps $g = N^\top x_k - s$, clipped at zero, with
impulse $p = \Delta t\,\hat\lambda_n$:

$$
v_{k+1} = v^* + A^{-1} N p,\qquad 0 \le p \perp W p + q \ge 0,\qquad
W = \Delta t\,N^\top A^{-1} N,\quad q = \max(g, 0) + \Delta t\,N^\top v^* .
$$

The position follows, then a post-stabilization projects any remaining penetration back onto
$\mathcal C$ in the $A$-metric, without changing the velocity:

$$
x_{k+1} = x_k + \Delta t\,v_{k+1},\qquad
x_{k+1} \leftarrow \arg\min_{y \in \mathcal C}\ \tfrac12 (y - x_{k+1})^\top A\,(y - x_{k+1}) .
$$

The rendered force is the negative of the coupling force the proxy was integrated with, at the same
extrapolated leader:

$$
f_k = K\,(x_{k+1} - \hat x_l) + D\,(v_{k+1} - v_{l,k}) .
$$

So $f_{ff} = -f$ is exactly the coupling force on the proxy. At constant leader velocity in free space
the proxy settles at $\hat x_l$ and $f$ vanishes; rendering against $x_{l,k}$ instead would leave
$f = K\Delta t\, v_l$ along the motion, a negative damping of $K\Delta t$.

For a proxy on or outside the wall ($g \ge 0$), $Wp + q$ is the vector of end-of-step gaps: the step
is perfectly inelastic and lands exactly on the wall, so the post-stabilization never acts. It acts
only for a proxy that starts inside; clipping $g$ at zero means such a proxy is only stopped from
moving further in, not given an outward velocity $-g/\Delta t$ (which would inject kinetic energy).
The position correction stretches the coupling when the leader is inside the wall, so proxies are
seeded outside.

$W$ is the Delassus operator of the discretized proxy. Implemented: one plane, for which
$p = \max\big(0, (-\max(g, 0)/\Delta t - n^\top v^*)/(n^\top A^{-1} n)\big)$ and the post-stabilization
is $x + A^{-1}n\,(s - n^\top x)/(n^\top A^{-1} n)$. With $M_p = mI$ and isotropic gains, $A = aI$ and
the step reduces to the clamp $n^\top v_{k+1} = \max(n^\top v^*, -\max(g, 0)/\Delta t)$. Several planes
(an edge, a corner) couple through $W$'s off-diagonal terms even for a scalar mass, and are solved
exactly by enumerating active sets (≤ 3 planes); not in the code yet.

- Robot: identical to M3: $x_t = \langle x_p\rangle_c$, $v_t = \langle v_p\rangle_c$, $f_{ff} = -\langle f\rangle_c$.
- Links: → $x_p, v_p, f$; ← nothing. The mass is fixed; the proxy is seeded once, from the leader.
- Parameter: $m = n^\top\Lambda_0 n$ (held $\Lambda_0$) along the motion direction at the start
  configuration, retuned per direction in E1: M4's best case, by rule.

Remarks.

- **Free motion.** The operator feels about $m a_p$, and the robot receives the same as feedforward.
  It needs $\Lambda a$ plus bias, so the PD supplies the rest: $e_F \approx (\Lambda - m)a$,
  independent of delay. It vanishes along the tuning direction at the tuning configuration only.
- **Contact.** The proxy impacts the wall inelastically with momentum $m v_p$, the robot with
  $m_F(n)\,v$. On hardware the difference does not reach the operator at a steady approach: the rigid
  constraint absorbs the proxy's momentum (E3, steady sweep).
- **Passivity.** A mass, a spring, a damper and an inelastic unilateral constraint, integrated with
  implicit Euler: passive for any delay.
- **Relation to M3.** As $m \to 0$ with $D = 0$, M4 reduces exactly to M3. With $D > 0$ the massless
  limit lags the leader by about $(D/K)\,v_l$, a few milliseconds.

### M5 — RIM, arm only · *implemented: `proxy-rim`*

The proxy is the arm's reduced dynamics, coupled to the leader by the virtual coupling. The controller
is not in the model: the robot is assumed to track the proxy exactly ($K_r \to \infty$).

- Haptic:
  $$\Lambda\,\dot v_p + z_i = K\,(x_l - x_p) + D\,(v_l - v_p) + f_{eff} + \hat\lambda,\qquad f = K\,(x_p - x_l) + D\,(v_p - v_l),$$
  integrated as M4 with $M_p = \Lambda$ (held between model updates), $z_i$ and $f_{eff}$ explicit.
  As built (§2): $c = C\dot q$, $f_{eff} = 0$.
- Robot: $x_t = \langle x_p\rangle_c$, $v_t = \langle v_p\rangle_c$, $f_{ff} = -\langle f\rangle_c$. The feedforward supplies the force; the OSC PD only rejects model error.
- Links: → $x_p, v_p, f$; ← the model $(\Lambda, z_i, f_{eff})$ at $f_m$ = 25 Hz.
- Model update: `update_rim` keeps $v_p$, so each update changes the proxy's kinetic energy by
  $\tfrac12 v_p^\top(\Lambda_{new} - \Lambda_{old})\,v_p$.

#### Orientation held · *implemented: `rim_hold_orientation` (`ModelConfig.hold_orientation`)*

Below the rotational modes (§3) the interface point moves as if the orientation were constrained.
With the rotation rows $J_h$ of the end-effector Jacobian ($J_h \ddot q + b_h = 0$,
$b_h = \dot J_h \dot q$) stacked under the interface rows, $J = [J_i; J_h]$, $b = [b_i; b_h]$:

$$
\Lambda_6 = (J M^{-1} J^\top)^{-1},\quad \Lambda = [\Lambda_6]_{ii},\quad
z_i = \big[\Lambda_6 (J M^{-1} c - b)\big]_i,\quad f_{eff} = \big[\Lambda_6 J M^{-1} \tau_o\big]_i .
$$

Once rotation is locked, the orientation torque is whatever the constraint needs: a reaction, not an
input, so $\tau_o$ excludes $\tau_{ori}$ (it would otherwise be counted twice). M4's $m$ is taken from
the same held $\Lambda_0$. It is the limit $k_{rot} \to \infty$ of M6's orientation part below.

The exact version (later): a 6-DoF proxy whose rotation is held by the OSC's own rotational spring and
damper captures the frequency dependence. `RIMIntegrator` is dimension-general; it needs matrix gains
$K = \mathrm{blkdiag}(K I_3, K_{rot})$, and the same for $D$.

#### 3-DoF contact projection (M4, M5) · *implemented in `pyrim.RIMIntegrator`*

The wall pushes along $n$ only, so its impulse $p \ge 0$ moves the proxy along $A^{-1} n$, which is not
along $n$ when $\Lambda$ couples axes (the M4 step above, with $A = \Lambda + \Delta t\,(D + \Delta t\,K)\,I$).
The proxy lands exactly on the wall; equivalently $v^+$ is the point nearest $v^*$ in the $A$-metric
with $n^\top v^+ \ge -\max(g,0)/\Delta t$ (Gauss's principle). At rest in contact the tangential
force vanishes and $f$ is normal (a frictionless table).

The per-axis projection used before (clamp $x_z$, $v_z \ge 0$) is the identity-metric one. At rest it
requires a static tangential force $r_x \approx -(\Lambda^{-1})_{xz}/(\Lambda^{-1})_{xx}\, r_z$:
$-0.29\,r_z$ with $\Lambda_0$ at (0.4, 0, 0.2) m, about −6 N felt sideways in a 20 N press (measured
2026-10-23: −6.8 N, proxy 6.9 mm off the leader). M4 ($A \propto I$) and M3 are unaffected:
$A^{-1}n \parallel n$.

### M6 — RIM, closed loop (arm + OSC) · *proposal*

The RIM is the robot **under its controller**, seen from the controller's input (the target). The
virtual coupling attaches the operator to that target, a massless node between two spring–dampers:

```
operator x_l ─(K, D)─ target x_t ─(K_r, D_r: the OSC)─ robot x̂ (Λ, z_i, f_eff) ─ table
```

- Haptic: integrate the predicted robot $\hat x$ and the target $x_t$ together:
  $$
  \underbrace{K\,(x_l - x_t) + D\,(v_l - v_t)}_{F} \;=\; K_r\,(x_t - \hat x) + D_r\,(v_t - \hat v)
  \qquad\text{(force balance at the massless target)}
  $$
  $$
  \Lambda\,\dot{\hat v} + z_i = F + f_{eff} + \hat\lambda(\hat x),
  \qquad f = -F .
  $$
- Robot: $x_t(t - T_c)$, $v_t(t - T_c)$, $f_{ff} = 0$. The real OSC, with its own $K_r, D_r$, produces $F$ itself.
- Links: → $x_t, v_t$; ← the model $(\Lambda, z_i, f_{eff})$ and the gains $K_r, D_r$.

Properties:

- **$K, D$ and $K_r, D_r$ are independent.** Lower $K_r$: the robot trails the target more, and the
  model predicts it.
- **The limit $K_r \to \infty$ gives $x_t = \hat x$, which is M5.**
- **Force matching in contact:** the table force on the robot is $K_r(x_t - \hat x) = F$, exactly what
  the operator feels. The felt stiffness is the series stiffness $K K_r / (K + K_r)$.
- **No feedforward is needed.** The target leads the robot by exactly what the real controller needs.
- **Passive:** springs, dampers, a mass and a unilateral constraint, integrated implicitly; the
  exception, as in M5, is the held $z_i$, $f_{eff}$.

Discretization: per step and per axis, the unknowns $(\hat v_{k+1}, x_{t,k+1})$ satisfy a 2 × 2 linear
system, followed by the contact projection on $\hat x$. With $D + D_r > 0$, the target is a first-order
node: $v_t = \big[K(x_l - x_t) + D v_l - K_r(x_t - \hat x) + D_r \hat v\big] / (D + D_r)$.

---

## 5. Side by side

| | Model on the haptic side | Operator feels | Robot receives | ← link (robot → haptic) | OSC gains |
|---|---|---|---|---|---|
| M1 ZOH | none | spring to the delayed robot | delayed leader | $x, v$ | $= K, D$ |
| M2 TDPA | none (+ PO/PC) | M1, passivity-corrected | delayed leader + PC damping wrench | $x, v$, energy | $= K, D$ |
| M3 God-object | geometry | spring to the projected leader | proxy + push | — | own |
| M4 Fixed mass | geometry + mass $m$ | spring to the proxy | proxy + coupling force | — | own |
| M5 RIM arm | geometry + $\Lambda, z_i, f_{eff}$ | spring to the proxy | proxy + coupling force | model | own (assumed ∞) |
| M6 RIM closed loop | geometry + $\Lambda, z_i, f_{eff}$ + $K_r, D_r$ | spring to the target | target | model, gains | own (modelled) |

M3 → M4 → M5 → M6 add one thing to the local model at each step: mass, then the robot's inertia, then
its controller. M1 and M2 are the model-free references.

---

## 6. What each method leaves unfelt (free motion, predicted)

| Method | $e_F$ | Depends on delay |
|---|---|---|
| M1 | $\approx -KTv$; RMS $K\cdot 2A\lvert\sin(\omega T/2)\rvert/\sqrt2$ on a sine | yes |
| M3 | $\approx \Lambda a$ | no |
| M4 | $\approx (\Lambda - mI)\,a$ | no |
| M5 | model error: friction, unmodelled inertia (rotor), $\tau_{ns}$, held terms | no |

---

## 7. Assumptions (all methods)

- **Known geometry.** The table plane $z = s$ is known exactly, as it could be from a camera on the
  remote side. Not estimated online.
- **Rigid table** in the models. The real table and tool have some compliance; on the robot that is
  model error.
- **OSC below its limits** (force 80 N, torque, torque rate `delta_tau_max`, contact force limiter).
  Saturation is not modelled.
- **Held terms.** $\Lambda, z_i, f_{eff}$ are constant between model updates. Joint friction and rotor
  inertia are unmodelled (the stick-slip in `experiments/benchmarks/osc_controller/FINDINGS.md`; E1
  measured the arm ~1.4× the held model) — the main sources of M5's residual on the real arm.
- **Delays.** Each link is a latest-value delay line; round-trip conditions are split evenly.

---

## 8. Open items

- **Coriolis in M5's model.** The OSC compensates $C\dot q$ (`control.coriolis_compensation: true`), so
  in $c - \tau_o$ it cancels, as gravity does. The live model keeps $c = C\dot q$ in $z_i$ with
  $f_{eff} = 0$, i.e. it renders a Coriolis force the controlled arm does not have. Consistent options:
  $c = 0$ in the model ($z_i = -[\Lambda_6 b]_i$), or $\tau_o \supset C\dot q$ in `f_ext`. Small at the
  experiments' speeds (≤ 0.4 m/s); fix before the paper.
- **M2's robot end** applies its damping one tick late (§4, M2): fix before M2 is used.
- **Reference hold under jitter.** A held nonzero $v_t$ against a frozen $x_t$ makes $D_r(v_t - v)$
  fight the spring. Extrapolate $x_t$ along $v_t$ (bounded), or zero $v_t$ after a timeout.
- **Several planes** (an edge, a corner) in `RIMIntegrator`: formulated in M4, not implemented.
- **Link notation:** subscript $f$ clashes with the force $f$ and the rates. Alternatives:
  $\langle y\rangle_\to$ and $\langle y\rangle_\leftarrow$, or $c$ and $b$ (back).
- **M2 variant:** confirm the observer form against Ryu et al. (2010) and Artigas et al. (2010), and
  decide whether to move to TDPA-ER P-P.

Resolved: contact step in `RIMIntegrator` (velocity-level LCP on $\max(g,0)$, A-metric
post-stabilization, 3-DoF impulse along $A^{-1}n$); $\Lambda$ update rule ($v$ kept); rendered and
integrated force at the same extrapolated leader; the integrator docstring's sign of $\lambda_i$; which
$\Lambda$ (held, §3); the terms the adapter builds (§2).
