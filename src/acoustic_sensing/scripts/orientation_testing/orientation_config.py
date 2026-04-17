# Bar orientation experiment – configuration
# Edit this file to change the experiment setup.
# orientation_experiment.py imports everything from here.

# ---------------------------------------------------------------------------
# Contact position
# ---------------------------------------------------------------------------

# World-frame XYZ of the sensor just touching the target surface [m].
# Jog the robot until the sensor makes contact, read off the end-effector
# position, and paste it here.  Set to None to capture the pose at startup.
CONTACT_POSITION_M =  [0.7503, -0.0607, 0.1136]  # e.g. [0.5668, -0.1165, 0.0817]

# How far above the contact point to start and return to [m].
APPROACH_HEIGHT_M = 0.030

# ---------------------------------------------------------------------------
# Compression
# ---------------------------------------------------------------------------

# How far past the contact point to press the sensor into the target [m].
COMPRESS_DEPTH_M = 0.012

# Time to hold at full compression while RF data is collected [s].
HOLD_AT_CONTACT_SEC = 3.0

# Number of press-hold-lift repetitions per bar angle.
N_REPEATS = 4

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
BASE_ORI_EULER_DEG = [177.0, 0.0, 0.0]

# Path to the fr3_pose_controller parameter file, relative to arm_client CONFIG_DIR.
PROBING_CONFIG_REL = "controllers/fr3_pose/orientation_pressing.yaml"

# ---------------------------------------------------------------------------
# Sensor geometry calibration
# ---------------------------------------------------------------------------

# Lateral positions of the 4 waveguides ACMacross the sensor face [mm].
# Origin is at the sensor centre-line; positive is in the +X sensor direction.
WAVEGUIDE_LATERAL_POSITIONS_MM = [-4.5, -1.5, 1.5, 4.5]

# Physical distance along a waveguide represented by one sample index step [mm].
WAVEGUIDE_SAMPLE_PITCH_MM = 0.1

# ---------------------------------------------------------------------------
# Serial / Teensy
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/ttyACM1"
BAUD_RATE = 3_000_000
SERIAL_TIMEOUT_SEC = 30

# ---------------------------------------------------------------------------
# EEF sampling
# ---------------------------------------------------------------------------

# Period between end-effector state samples (position, orientation, wrench,
# velocity, acceleration) during each trial [s].  Default 0.01 s = 100 Hz.
EE_SAMPLE_PERIOD_SEC = 0.01

# ---------------------------------------------------------------------------
# Hold-phase force control
# ---------------------------------------------------------------------------

# Set True to run a Z-force controller during the hold instead of a plain sleep.
# The controller samples Fz right after compression, then nudges Z each tick to
# keep the force at that level so the bar cannot push the robot back up.
HOLD_FORCE_CONTROL = True

# Control loop rate during hold [Hz].
HOLD_FORCE_CTRL_HZ = 50.0

# Settle time after the compress trajectory before sampling fz_target [s].
# Gives the position controller time to reach the compressed depth so the
# force snapshot reflects the true contact state, not a transient.
HOLD_SETTLE_SEC = 0.5

# Proportional gain [m/N]: per-tick Z nudge = -Kp * f_error.
# At 50 Hz this gives an effective speed of Kp/dt m/s per N of error.
HOLD_FORCE_KP = 0.0001

# Dead-band around the target force [N]. Errors smaller than this are ignored
# to avoid buzzing at the contact surface.
HOLD_FORCE_DEADBAND_N = 0.1

# Maximum Z correction in either direction from the initial compressed Z [m].
HOLD_FORCE_MAX_Z_CORR_M = 0.010

# ---------------------------------------------------------------------------
# Hold-phase rotational compliance (torque-based orientation adaptation)
# ---------------------------------------------------------------------------

# Set True to allow Ry and Rz to adapt during the hold based on torque feedback.
# Rx (the 180° flip) is always kept fixed.  With this enabled, BASE_ORI_EULER_DEG
# can be [180, 0, 0] — the surface tilt is found automatically from the torques,
# removing the need to hardcode the -2.1° (or similar) inclination manually.
HOLD_TORQUE_ADAPT = True

# Proportional gain [deg / Nm] per tick: delta_angle = Kto * torque_error.
# Positive torque → positive angle (compliant: follow the surface torque).
# If the robot tilts the wrong way, negate this value.
HOLD_TORQUE_KP_DEG_PER_NM = 2.0

# Dead-band [Nm]: torques smaller than this are ignored (avoids buzzing).
HOLD_TORQUE_DEADBAND_NM = 0.05

# Maximum angular correction from the initial orientation [deg] for Ry and Rz.
HOLD_TORQUE_MAX_CORR_DEG = 8.0

# ---------------------------------------------------------------------------
# Dry-run / testing
# ---------------------------------------------------------------------------

# Set to True to skip all serial communication and RF acquisition.
# The robot still moves through the full motion sequence so you can verify
# poses and speeds without needing the Teensy connected.
DRY_RUN = False
