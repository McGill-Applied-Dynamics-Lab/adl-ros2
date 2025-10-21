import numpy as np
import matplotlib.pyplot as plt
import pickle
from pathlib import Path

# Load the data from the pickle file
data_file = Path(__file__).resolve().parent / "results" / "64_GRID_TRAIN.pkl"
with open(data_file, "rb") as f:
    exp_dict = pickle.load(f)
print("Data loaded from:", data_file)

# Plot the data
"""
Structure
exp_dict = {
        "ts": [],
        "target_poses": [],
        "ee_poses": [],
        "ee_forces": [],
    }
"""
# 2x2 grid
fig, ax = plt.subplots(2, 2, figsize=(12, 8))
for idx in range(len(exp_dict['ts'])):
    ax[0, 0].plot(exp_dict['ts'][idx], [val[2] for val in exp_dict['ee_forces'][idx]], label=f'Trial {idx+1}')
ax[0, 0].set_xlabel("Time (s)")
ax[0, 0].set_ylabel("Z Force (N)")
ax[0, 0].set_title("End Effector Z Force")
ax[0, 0].legend()
ax[0, 0].grid(True)

for idx in range(len(exp_dict['ts'])):
    ax[0, 1].plot(exp_dict['ts'][idx], [pose.position[2] for pose in exp_dict['ee_poses'][idx]], label=f'Trial {idx+1}')
    ax[0, 1].plot(exp_dict['ts'][idx], [pose.position[2] for pose in exp_dict['target_poses'][idx]], linestyle='--', label=f'Target {idx+1}')
ax[0, 1].set_xlabel("Time (s)")
ax[0, 1].set_ylabel("X Position (m)")
ax[0, 1].set_title("End Effector X Position")
ax[0, 1].legend()
ax[0, 1].grid(True)

plt.show()