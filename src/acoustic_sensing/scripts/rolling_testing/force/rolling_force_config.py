# Rolling contact experiment – FORCE CONTROL configuration
# Used by rolling_contact_experiment_force.py.
# Compress and decompress phases are still position-controlled (same as fr3pose version).
# The slide phase maintains a constant Z contact force using robot.set_target() at
# FORCE_CTRL_HZ, advancing Y at SLIDE_SPEED_M_S and correcting Z proportionally to
# the Fz error.

# ---------------------------------------------------------------------------
# Contact position
# ---------------------------------------------------------------------------

CONTACT_POSITION_M = [0.5307, -0.1211, 0.0977]
APPROACH_HEIGHT_M = 0.030        # [m]

# ---------------------------------------------------------------------------
# Roller geometry
# ---------------------------------------------------------------------------

ROLLER_DIAMETER_M = 0.0250
COMPRESS_DEPTH_M = 0.010         # [m]  initial position-controlled compress depth

# ---------------------------------------------------------------------------
# Roll / slide motion
# ---------------------------------------------------------------------------

FINRAY_LENGTH_M = 0.1250
ROLL_LENGTH_PERCENT = 100.0

# Slide speed [m/s] — Y advances at this constant rate during force control.
SLIDE_SPEED_M_S = 0.005          # [m/s]

# Maximum Y travel allowed per pass before the loop forcibly terminates.
# Set to None to use FINRAY_LENGTH_M * ROLL_LENGTH_PERCENT / 100.
MAX_SLIDE_DISTANCE_M = None      # [m] or None

# Timeout for the entire force-controlled slide [s].
SLIDE_TIMEOUT_S = 120.0          # [s]

# ---------------------------------------------------------------------------
# Force control parameters
# ---------------------------------------------------------------------------

# Force dead-band [N].
# Sign convention: end_effector_wrench["force"][2] is NEGATIVE when the robot
# is pressing into the surface.  More negative = harder contact.
#
#   fz > FORCE_UPPER_N  → not enough force → press down (PID active)
#   fz < FORCE_LOWER_N  → too much force   → lift up   (PID active)
#   FORCE_LOWER_N ≤ fz ≤ FORCE_UPPER_N    → hold Z    (no correction)
#
# A narrow 1 N band [-11, -10] gives the robot a quiet zone so it does not
# bang-bang between press-down and lift-up on every cycle.
# The acquisition phase presses until fz <= FORCE_UPPER_N before sliding starts.
FORCE_UPPER_N = -25.0            # [N]  press down if fz rises above this
FORCE_LOWER_N = -26.0            # [N]  lift up   if fz drops below this

# Adaptive Kp — estimated online from the local fin-ray stiffness.
# The actuator stiffness varies along the sliding direction, so a fixed Kp
# would over-correct in stiff regions and under-correct in compliant ones.
#
# Each step: k_est = Δfz / Δz  [N/m]  (from a sliding window of recent samples)
#            Kp    = 1 / k_est  [m/N]  (clamped to [KP_MIN, KP_MAX])
#
# KP_INITIAL is used until enough samples are available for a valid estimate.
KP_INITIAL = 0.0002              # [m/N]  fallback before stiffness can be estimated
KP_MIN     = 0.0001              # [m/N]  lower bound (very stiff region near fin-ray base)
KP_MAX     = 0.0007              # [m/N]  upper bound (compliant tip region)

# Number of past samples used for the stiffness slope estimate.
# Larger = smoother but slower to adapt.
STIFFNESS_WINDOW = 25            # [samples]

# Minimum Z displacement in the window required to trust the slope estimate [m].
# If the robot has barely moved in Z, the slope is noise-dominated.
STIFFNESS_MIN_DZ_M = 5e-5        # [m]  (0.05 mm)

# ---------------------------------------------------------------------------
# PID gains for force control
# ---------------------------------------------------------------------------
# The P gain is adaptive (1/k_est, see above). Ki and Kd are fixed.
#
# Ki [m/(N·s)]: integrator — corrects persistent steady-state force offset
#   (e.g. surface slope or gravity bias). Keep small; windup is clamped.
# Kd [m·s/N]:  derivative — damps oscillation when stiffness changes rapidly.
#   Applied to the rate of change of f_error, so it resists sudden force spikes.
#
# Set Ki = 0 and Kd = 0 to fall back to pure adaptive-P control.
KI_FORCE = 0.00005               # [m/(N·s)]
KD_FORCE = 0.0001                # [m·s/N]

# Anti-windup clamp: maximum accumulated integral correction [m].
# Prevents the integrator from winding up during long contact-lost periods.
KI_WINDUP_CLAMP_M = 0.002        # [m]

# ---------------------------------------------------------------------------
# Force acquisition (pre-slide press-to-force phase)
# ---------------------------------------------------------------------------
# After the position-controlled compress, the sensor may not yet be in the
# target force band.  A dedicated acquisition phase presses slowly using
# set_target() until fz <= FORCE_UPPER_N, then rebases the Z window around
# that acquired depth before starting the slide.

# Maximum additional Z depth to press during acquisition [m].
# Must be large enough to bring the sensor from the compressed position into
# the target force band.  Safe to set generously (the loop stops as soon as
# force is reached).
FORCE_ACQUIRE_MAX_DEPTH_M = 0.020   # [m]  up to 20 mm extra press

# Speed at which the acquisition phase presses down [m/s].
FORCE_ACQUIRE_SPEED_M_S = 0.0026     # [m/s]  slow and controlled

# Timeout: give up if force not acquired within this many seconds.
FORCE_ACQUIRE_TIMEOUT_S = 15.0      # [s]

# ---------------------------------------------------------------------------
# Slide force-window parameters
# -------------------------------------1, --------------------------------------

# Maximum Z correction allowed during the slide, measured from the
# ACQUIRED contact Z (not the raw compressed Z).
# The fin-ray gets dramatically stiffer toward the clamped base (125 mm),
# so the surface profile may require more Z travel than at the tip.
MAX_Z_CORRECTION_M = 0.020       # [m]  ±20 mm from acquired contact depth

# Maximum Z correction velocity [m/s].
# Y is decoupled: Y always advances at SLIDE_SPEED_M_S regardless of Z.
# Z correction is capped independently at this speed for smooth motion.
# Total EEF speed ≈ sqrt(SLIDE_SPEED_M_S² + vz²) — very close to constant
# since vz << vy in normal operation.
MAX_VZ_SPEED_M_S = 0.002         # [m/s]  (2.0 mm/s max Z velocity)

# Contact is considered lost when fz rises ABOVE this value during the slide.
# Set less negative than FORCE_UPPER_N so brief over-shoots don't trip it.
CONTACT_FORCE_THRESHOLD_N = -1.0  # [N]  (lost when fz > -1 N)

# Force control loop rate [Hz].
FORCE_CTRL_HZ = 50.0             # [Hz]

# ---------------------------------------------------------------------------
# Speeds (position-controlled phases)
# ---------------------------------------------------------------------------

APPROACH_SPEED_M_S = 0.005      # [m/s]
RETURN_SPEED_M_S = 0.01         # [m/s]

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------

SETTLE_SEC = 0.5                 # [s]

# ---------------------------------------------------------------------------
# Robot / controller
# ---------------------------------------------------------------------------

ROBOT_NAMESPACE = "fr3"
BASE_ORI_EULER_DEG = [180.0, 0.0, 0.0]
EE_SAMPLE_PERIOD_SEC = 0.02

# ---------------------------------------------------------------------------
# Serial / Teensy
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyACM1"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 10

# -----------------------------6----------------------------------------------
# Dry-run / testing
# ---------------------------------------------------------------------------

DRY_RUN = False
