import pickle
import numpy as np
from pathlib import Path

def convert_pickle():
    """ Convert pickle file generated during experiment to numpy-friendly arrays for device cross-compatibility. """
    


clean_dict = {}

# Load the pickle file
PROJECT_ROOT = Path(__file__).resolve().parent
data_file = PROJECT_ROOT / "results" / "64_GRID_TEST.pkl"
with open(data_file, "rb") as f:
    exp_dict = pickle.load(f)
print("Data loaded from:", data_file)

# Convert to numpy arrays
ts = np.array(exp_dict["ts"])
sf_positions = np.array(exp_dict["sf_positions"])
rf_positions = np.array(exp_dict["rf_positions"])

# Unpack target poses
target_poses = exp_dict["target_poses"]
target_positions = np.array([[pose.position for pose in trial] for trial in target_poses])
target_orientations = np.array([[pose.orientation.as_quat() for pose in trial] for trial in target_poses])

# Unpack end-effector poses
ee_poses = exp_dict["ee_poses"]
ee_positions = np.array([[pose.position for pose in trial] for trial in ee_poses])
ee_orientations = np.array([[pose.orientation.as_quat() for pose in trial] for trial in ee_poses])

# Unpack end-effector forces
ee_forces = np.array(exp_dict["ee_forces"])

print("Data converted to numpy arrays.")

# Create clean dictionary
clean_dict["ts"] = ts
clean_dict["sf_positions"] = sf_positions
clean_dict["rf_positions"] = rf_positions
clean_dict["target_positions"] = target_positions
clean_dict["target_orientations"] = target_orientations
clean_dict["ee_positions"] = ee_positions
clean_dict["ee_orientations"] = ee_orientations
clean_dict["ee_forces"] = ee_forces