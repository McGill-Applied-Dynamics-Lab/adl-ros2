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
CONTACT_POSITION_M = [0.5668, -0.1165, 0.0817]

# How far above the contact point the robot starts (the safe approach height).
# The script moves from CONTACT_POSITION_M + [0, 0, +APPROACH_HEIGHT_M] down
# to contact, so this must clear the cylinder when moving laterally.
APPROACH_HEIGHT_M = 0.030        # [m]

# ---------------------------------------------------------------------------
# Roller geometry
# ---------------------------------------------------------------------------

# Roller diameter [m]. Default 2.5 cm tube.
ROLLER_DIAMETER_M = 0.0250

# How far past contact to compress the sensor into the cylinder surface.
# Provides a consistent preload throughout the roll.  Start small (2–3 mm).
COMPRESS_DEPTH_M = 0.005      # [m]

# ---------------------------------------------------------------------------
# Roll / slide motion
# ---------------------------------------------------------------------------

# Fin-ray active length to roll across [m].
FINRAY_LENGTH_M = 0.1250

# Percentage of the fin-ray length to test with the roller.
ROLL_LENGTH_PERCENT = 100.0

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
ROLL_SPEEDS_M_S = [0.005]   # [m/s]

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

# ---------------------------------------------------------------------------
# Robot / controller
# ---------------------------------------------------------------------------

ROBOT_NAMESPACE = "fr3"
CONTROLLER_NAME = "joint_trajectory_controller"
BASE_ORI_EULER_DEG = [180.0, 0.0, 0.0]
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

# ---------------------------------------------------------------------------
# Dry-run / testing
# ---------------------------------------------------------------------------

# Set to True to skip all serial communication and RF acquisition.
# The robot still moves through the full motion sequence so you can verify
# poses and speeds without needing the Teensy connected.
DRY_RUN = False
