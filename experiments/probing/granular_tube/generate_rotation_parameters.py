#!/usr/bin/env python3
"""
Generate random wrist rotation parameters (angles and speeds).

This script creates a pickle file with pre-generated random parameters that will be
used by twist.py. This ensures parameters are generated once and reused across runs.

Simply run:
    python generate_rotation_parameters.py

Edit the CONFIG section below to customize parameters.
"""

import numpy as np
import pickle
from pathlib import Path

# ===== CONFIGURATION =====
NUM_POINTS = 10  # Number of rotation pairs to generate
ANGLE_RANGE = (0.0, np.deg2rad(125))  # Min and max rotation angle in radians (±28.6°)
SPEED_RANGE = (0.10, 0.40)  # Min and max rotation speed in radians/second
RANDOM_SEED = None  # Use None for random, or set to integer (e.g., 42) for reproducibility

OUTPUT_FILE = Path(__file__).parent / "results" / "grids" / "rotation_params.pkl"
# ========================


def generate_rotation_parameters(
    num_points: int = NUM_POINTS,
    angle_range: tuple = ANGLE_RANGE,
    speed_range: tuple = SPEED_RANGE,
    seed: int = RANDOM_SEED,
    output_file: Path = OUTPUT_FILE
):
    """
    Generate random wrist rotation parameters.

    Args:
        num_points: Number of rotation pairs to generate
        angle_range: (min, max) rotation angle in radians
        speed_range: (min, max) rotation speed in radians/second
        seed: Random seed for reproducibility (None = random)
        output_file: Path to save parameters
    """
    if seed is not None:
        np.random.seed(seed)

    # Ensure output directory exists
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Generate random parameters
    angles = np.random.uniform(angle_range[0], angle_range[1], num_points)
    speeds = np.random.uniform(speed_range[0], speed_range[1], num_points)

    params_dict = {
        "angles": angles,
        "speeds": speeds,
        "num_points": num_points,
        "angle_range": angle_range,
        "speed_range": speed_range,
        "seed": seed,
    }

    # Save to file
    with open(output_file, "wb") as f:
        pickle.dump(params_dict, f)

    print(f"✓ Generated {num_points} rotation parameters")
    print(f"  Angle range: {np.degrees(angle_range[0]):.2f}° to {np.degrees(angle_range[1]):.2f}°")
    print(f"  Speed range: {np.degrees(speed_range[0]):.2f}°/s to {np.degrees(speed_range[1]):.2f}°/s")
    print(f"  Seed: {seed if seed is not None else 'random'}")
    print(f"  Saved to: {output_file}")

    # Print sample parameters
    print(f"\nSample parameters (first 5):")
    print(f"{'Index':<6} {'Angle (°)':<12} {'Speed (°/s)':<12}")
    print("-" * 30)
    for i in range(min(5, num_points)):
        print(f"{i:<6} {np.degrees(angles[i]):<12.2f} {np.degrees(speeds[i]):<12.2f}")
    if num_points > 5:
        print(f"... ({num_points - 5} more)")

    return params_dict


if __name__ == "__main__":
    generate_rotation_parameters()
