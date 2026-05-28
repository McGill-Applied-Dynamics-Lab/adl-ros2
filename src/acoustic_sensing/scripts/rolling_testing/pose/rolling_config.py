# Rolling contact experiment – configuration
# Edit this file to change the experiment setup.
# rolling_contact_experiment.py imports everything from here.

# ---------------------------------------------------------------------------
# Contact position
# ---------------------------------------------------------------------------

# World-frame XYZ of the cylinder surface at φ=0 (the top, where the sensor
# first touches).  Jog the robot until the sensor just makes contact with the
# top of the cylinder, read off the end-effector position, and paste it here.
# Units: metres.  Set to None to fall back to capturing the pose at startup.
CONTACT_POSITION_M = [0.4892, -0.0912, 0.0867]

# How far above the contact point the robot starts (the safe approach height).
# The script moves from CONTACT_POSITION_M + [0, 0, +APPROACH_HEIGHT_M] down
# to contact, so this must clear the cylinder when moving laterally.
APPROACH_HEIGHT_M = 0.030        # [m]

# ---------------------------------------------------------------------------
# Roller geometry
# ---------------------------------------------------------------------------

# Roller diameter [m]. Default 2.5 cm tube.
ROLLER_DIAMETER_M = 0.0240

# How far past contact to compress the sensor into the cylinder surface.
# Provides a consistent preload throughout the roll.  Start small (2–3 mm).
COMPRESS_DEPTH_M = 0.004      # [m]

# ---------------------------------------------------------------------------
# Roll / slide motion
# ---------------------------------------------------------------------------

# Fin-ray active length to roll across [m].
FINRAY_LENGTH_M = 0.1250

# Percentage of the fin-ray length to test with the roller.
ROLL_LENGTH_PERCENT = 100.0

# Safety margin subtracted from the end of the roll so the roller does NOT
# travel all the way to the tip of the sensor (avoids running off the
# fin-ray's free end).  Effective roll distance =
#   FINRAY_LENGTH_M * (ROLL_LENGTH_PERCENT/100) - ROLL_END_OFFSET_M
ROLL_END_OFFSET_M = 0.020        # [m]

# Maximum Cartesian spacing between successive planned poses for each segment.
# Smaller values produce denser interpolation and more IK points.
APPROACH_POINT_SPACING_M = 0.010
CONTACT_POINT_SPACING_M = 0.006
COMPRESS_POINT_SPACING_M = 0.003
SLIDE_POINT_SPACING_M = 0.006
DECOMPRESS_POINT_SPACING_M = 0.003
RETURN_POINT_SPACING_M = 0.012

# ---------------------------------------------------------------------------
# Speeds
# ---------------------------------------------------------------------------

# List of rolling speeds to test; one full pass is made at each speed.
ROLL_SPEEDS_M_S = [0.005,0.010,0.020]   # [m/s]

# How many times to repeat each speed.  Total trials per session =
# len(ROLL_SPEEDS_M_S) * N_REPEATS.
N_REPEATS = 2

# Speed used for the slow approach/compress/decompress phases.
APPROACH_SPEED_M_S = 0.005      # [m/s]

# Speed used for the fast return to approach pose between trials.
RETURN_SPEED_M_S = 0.05         # [m/s]

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------

# Settle time inserted between executed planned trajectories.
# Used for approach and return phases only; contact phases (compress/slide/decompress)
# use settle_time=0.0 directly to avoid dead-hold stops at segment boundaries.
SETTLE_SEC = 0.5                 # [s]

# Hold the robot still at "contact reached" before reading the measured contact
# pose.  Lets the EEF fully settle so subsequent compress targets are accurate.
CONTACT_HOLD_SEC = 1.0           # [s]

# Hold the robot still after planning is finished and just before the compress
# trajectory starts streaming.  Visual checkpoint to verify the planned poses.
PRE_COMPRESS_HOLD_SEC = 1.0      # [s]

# Hold the robot still between slide completion and decompress start.  Without
# it, the EE finishes the slide with +Y velocity and the controller is asked
# to move +Z immediately — that abrupt direction change spikes joint
# acceleration and trips `cartesian_motion_generator_joint_acceleration_discontinuity`.
# At higher slide speeds (≥20 mm/s) needs ~2 s to fully decelerate.
POST_SLIDE_HOLD_SEC = 1.0        # [s]

# When True, live_rolling_view auto-advances through trial prompts (no need to
# click the "Continue" button between trials).  The first prompt at "approach
# reached" is still surfaced so the operator can abort if needed.
LIVE_AUTO_RUN = True

# ---------------------------------------------------------------------------
# Trajectory safety (anti-reflex guardrails)
# ---------------------------------------------------------------------------

# Hard cap on Cartesian segment speed.  Any planned segment that would exceed
# this speed is stretched in time until it complies.  Defends against
# `cartesian_motion_generator_joint_acceleration_discontinuity` reflexes when a
# trajectory's path length collapses to near-zero (e.g. degenerate compress).
MAX_CARTESIAN_SPEED_M_S = 0.05   # [m/s]

# Floor on total trajectory duration.  Stops `_duration_for_move` from
# returning a tiny value when start≈end, which otherwise crushes all
# waypoints into a <1 s window and triggers acceleration spikes.
MIN_TRAJ_DURATION_S = 1.0        # [s]

# Minimum time between successive trajectory waypoints.  `_build_..._traj`
# caps n_waypoints so consecutive waypoints are never closer than this in
# time, preventing the controller from racing through dense waypoints.
MIN_WAYPOINT_DT_S = 0.05         # [s]

# ---------------------------------------------------------------------------
# Robot / controller
# ---------------------------------------------------------------------------

ROBOT_NAMESPACE = "fr3"
CONTROLLER_NAME = "joint_trajectory_controller"
BASE_ORI_EULER_DEG = [180.0, -2.8, 0.0]
EE_SAMPLE_PERIOD_SEC = 0.02
IK_POSITION_WEIGHT = 250.0
IK_ORIENTATION_WEIGHT = 150.0
IK_SIMILARITY_WEIGHT = 0.005
FIRST_WAYPOINT_MAX_JOINT_SPEED_RAD_S = 0.10

# ---------------------------------------------------------------------------
# Serial / Teensy
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 10

# Number of RF frames to capture while the robot is stationary (no contact)
# before any rolling trial begins. Set to 0 to skip.
STATIONARY_BASELINE_FRAMES = 20
STATIONARY_BASELINE_TIMEOUT_SEC = 30.0

# Additional Z offset above the approach pose where the stationary baseline
# is captured (so the sensor is well clear of the cylinder).
# Baseline pose Z = contact_z + APPROACH_HEIGHT_M + BASELINE_EXTRA_HEIGHT_M.
BASELINE_EXTRA_HEIGHT_M = 0.050   # [m]

# ---------------------------------------------------------------------------
# Dry-run / testing
# ---------------------------------------------------------------------------

# Set to True to skip all serial communication and RF acquisition.
# The robot still moves through the full motion sequence so you can verify
# poses and speeds without needing the Teensy connected.
DRY_RUN = False
