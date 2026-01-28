#!/usr/bin/env python3
"""
Plot results from granular bed probing experiments.

This script loads and visualizes data saved by main_probing.py.
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_ROOT / "results" / "R_MEGA_GRID.pkl"

with open(RESULTS_DIR, "rb") as f:
    results = pickle.load(f)

# Plot the z-values for each plunge
positions = results['ee_poses']
forces = results['ee_forces']
times = results['ts']
fig, ax = plt.subplots(1,2,figsize=(12, 8))

# Plot z-values
for i, probe in enumerate(positions):
    t = times[i]
    ax[0].plot(t, [vals[2] for vals in probe['positions']], '.')
    ax[0].set_title(f"Probe {i+1}")

# Plot forces
for i, force in enumerate(forces):
    t = times[i]
    ax[1].plot(t, [vals[2] for vals in force], '.')
    ax[1].set_title(f"Force {i+1}")

# plt.savefig(PROJECT_ROOT / "results" / "combo.png")
plt.show()
