# Bar orientation experiment – configuration
# Edit this file to change the experiment setup.
# orientation_experiment.py imports everything from here.

# ---------------------------------------------------------------------------
# Contact position
# ---------------------------------------------------------------------------

# World-frame XYZ of the sensor just touching the target surface [m].
# Jog the robot until the sensor makes contact, read off the end-effector
# position, and paste it here.  Set to None to capture the pose at startup.
CONTACT_POSITION_M =  [0.5668, -0.1165, 0.0817]  # e.g. [0.5668, -0.1165, 0.0817]

# How far above the contact point to start and return to [m].
APPROACH_HEIGHT_M = 0.030

# ---------------------------------------------------------------------------
# Compression
# ---------------------------------------------------------------------------

# How far past the contact point to press the sensor into the target [m].
COMPRESS_DEPTH_M = 0.005

# Time to hold at full compression while RF data is collected [s].
HOLD_AT_CONTACT_SEC = 3.0

# ---------------------------------------------------------------------------
# Speeds
# ---------------------------------------------------------------------------

# Speed for the slow descent to contact and compression [m/s].
APPROACH_SPEED_M_S = 0.005

# Speed for lifting and returning to approach pose [m/s].
RETURN_SPEED_M_S = 0.02

# ---------------------------------------------------------------------------
# Robot / controller
# ---------------------------------------------------------------------------

ROBOT_NAMESPACE = "fr3"
CONTROLLER_NAME = "fr3_pose_controller"
BASE_ORI_EULER_DEG = [180.0, 0.0, 0.0]

# Path to the fr3_pose_controller parameter file, relative to arm_client CONFIG_DIR.
PROBING_CONFIG_REL = "controllers/fr3_pose/probing.yaml"

# ---------------------------------------------------------------------------
# Sensor geometry calibration
# ---------------------------------------------------------------------------

# Lateral positions of the 4 waveguides across the sensor face [mm].
# Origin is at the sensor centre-line; positive is in the +X sensor direction.
WAVEGUIDE_LATERAL_POSITIONS_MM = [-4.5, -1.5, 1.5, 4.5]

# Physical distance along a waveguide represented by one sample index step [mm].
WAVEGUIDE_SAMPLE_PITCH_MM = 0.1

# ---------------------------------------------------------------------------
# Serial / Teensy
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 30

# ---------------------------------------------------------------------------
# Dry-run / testing
# ---------------------------------------------------------------------------

# Set to True to skip all serial communication and RF acquisition.
# The robot still moves through the full motion sequence so you can verify
# poses and speeds without needing the Teensy connected.
DRY_RUN = False
