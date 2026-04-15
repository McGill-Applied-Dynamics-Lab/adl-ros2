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

# List of (min, max) angle ranges in radians to allow for discontinuous sampling
ANGLE_RANGES = [
    (np.radians(-110.0), np.radians(-50.0)),
    (np.radians(50.0), np.radians(110.0)),
]

SPEED_RANGE = (
    np.radians(40.0),
    np.radians(60.0),
)  # Min and max rotation speed in radians/second

RANDOM_SEED = (
    None  # Use None for random, or set to integer (e.g., 42) for reproducibility
)

OUTPUT_FILE = Path(__file__).parent / "results" / "grids" / "rotation_params.pkl"
# ========================


def generate_rotation_parameters(
    num_points: int = NUM_POINTS,
    angle_ranges: list = ANGLE_RANGES,
    speed_range: tuple = SPEED_RANGE,
    seed: int = RANDOM_SEED,
    output_file: Path = OUTPUT_FILE,
):
    """
    Generate random wrist rotation parameters across discontinuous ranges.

    Args:
        num_points: Number of rotation pairs to generate
        angle_ranges: List of (min, max) rotation angle tuples in radians
        speed_range: (min, max) rotation speed in radians/second
        seed: Random seed for reproducibility (None = random)
        output_file: Path to save parameters
    """
    if seed is not None:
        np.random.seed(seed)

    # Ensure output directory exists
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Calculate probabilities for each range based on its width
    # This ensures true uniform sampling across all valid domains
    widths = [r[1] - r[0] for r in angle_ranges]
    total_width = sum(widths)
    probs = [w / total_width for w in widths]

    # 1. Randomly pick which range each point belongs to based on width probability
    chosen_range_indices = np.random.choice(len(angle_ranges), size=num_points, p=probs)

    # 2. Sample uniformly within the chosen range
    angles = np.array(
        [
            np.random.uniform(angle_ranges[idx][0], angle_ranges[idx][1])
            for idx in chosen_range_indices
        ]
    )

    # Generate speeds normally
    speeds = np.random.uniform(speed_range[0], speed_range[1], num_points)

    params_dict = {
        "angles": angles,
        "speeds": speeds,
        "num_points": num_points,
        "angle_ranges": angle_ranges,
        "speed_range": speed_range,
        "seed": seed,
    }

    # Save to file
    with open(output_file, "wb") as f:
        pickle.dump(params_dict, f)

    print(f"✓ Generated {num_points} rotation parameters")
    print("  Angle ranges:")
    for r in angle_ranges:
        print(f"    - {np.degrees(r[0]):.2f}° to {np.degrees(r[1]):.2f}°")
    print(
        f"  Speed range: {np.degrees(speed_range[0]):.2f}°/s to {np.degrees(speed_range[1]):.2f}°/s"
    )
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
