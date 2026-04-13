#!/usr/bin/env python3
"""
Generate random bending parameters (phi, theta, speed) for bend experiments.

This script creates a parameter file with randomized spherical arc parameters
that can be used by bend.py to run a series of bending experiments.
"""

import numpy as np
import pickle
from pathlib import Path


def generate_bend_parameters(
    num_experiments: int = 50,
    theta_min: float = 15.0,
    theta_max: float = 60.0,
    phi_min: float = -180.0,
    phi_max: float = 180.0,
    speed_min: float = 5.0,
    speed_max: float = 30.0,
    seed: int = None,
) -> dict:
    """
    Generate random bending parameters.

    Args:
        num_experiments: Number of bend experiments to generate
        theta_min: Minimum polar angle (degrees)
        theta_max: Maximum polar angle (degrees)
        phi_min: Minimum azimuthal angle (degrees)
        phi_max: Maximum azimuthal angle (degrees)
        speed_min: Minimum angular speed (degrees/second)
        speed_max: Maximum angular speed (degrees/second)
        seed: Random seed for reproducibility

    Returns:
        Dictionary with 'phi', 'theta', 'angular_speed', and 'num_points'
    """
    if seed is not None:
        np.random.seed(seed)

    # Generate random parameters
    phi_deg = np.random.uniform(phi_min, phi_max, num_experiments)
    theta_deg = np.random.uniform(theta_min, theta_max, num_experiments)
    angular_speed = np.random.uniform(speed_min, speed_max, num_experiments)

    parameters = {
        "phi": phi_deg,  # Azimuthal angle (degrees)
        "theta": theta_deg,  # Polar angle (degrees)
        "angular_speed": angular_speed,  # Angular speed (degrees/second)
        "num_points": num_experiments,
    }

    return parameters


def save_parameters(
    parameters: dict,
    output_file: Path = None,
) -> Path:
    """
    Save parameters to a pickle file.

    Args:
        parameters: Dictionary of parameters
        output_file: Path to save file (default: results/grids/bend_params.pkl)

    Returns:
        Path to saved file
    """
    if output_file is None:
        output_file = (
            Path(__file__).resolve().parent / "results" / "grids" / "bend_params.pkl"
        )

    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, "wb") as f:
        pickle.dump(parameters, f)

    print(f"✓ Saved {parameters['num_points']} parameter sets to {output_file}")
    return output_file


def print_parameters(parameters: dict) -> None:
    """Print a summary of generated parameters."""
    print("\n" + "=" * 70)
    print("BEND EXPERIMENT PARAMETERS")
    print("=" * 70)
    print(f"Number of experiments: {parameters['num_points']}")
    print("\nPolar Angle (theta):")
    print(f"  Min: {parameters['theta'].min():.2f}°")
    print(f"  Max: {parameters['theta'].max():.2f}°")
    print(f"  Mean: {parameters['theta'].mean():.2f}°")

    print("\nAzimuthal Angle (phi):")
    print(f"  Min: {parameters['phi'].min():.2f}°")
    print(f"  Max: {parameters['phi'].max():.2f}°")
    print(f"  Mean: {parameters['phi'].mean():.2f}°")

    print("\nAngular Speed (degrees/second):")
    print(f"  Min: {parameters['angular_speed'].min():.2f}°/s")
    print(f"  Max: {parameters['angular_speed'].max():.2f}°/s")
    print(f"  Mean: {parameters['angular_speed'].mean():.2f}°/s")

    print("\nFirst 5 experiments:")
    print("-" * 70)
    for i in range(min(5, parameters["num_points"])):
        # Calculate execution time for display
        execution_time = parameters["theta"][i] / parameters["angular_speed"][i]
        print(
            f"  Run {i + 1:3d}: phi={parameters['phi'][i]:7.2f}°, "
            f"theta={parameters['theta'][i]:6.2f}°, "
            f"speed={parameters['angular_speed'][i]:5.2f}°/s "
            f"(exec_time={execution_time:.2f}s)"
        )
    print("=" * 70 + "\n")


def main():
    """Generate default parameters and save to file."""
    print("Generating bend experiment parameters...")

    # Generate parameters with reasonable defaults
    # Angular speeds in deg/s: 5-30 deg/s is typical for smooth robotic motion
    parameters = generate_bend_parameters(
        num_experiments=10,
        theta_min=15.0,
        theta_max=60.0,
        phi_min=-180.0,
        phi_max=180.0,
        speed_min=5.0,  # deg/s
        speed_max=30.0,  # deg/s
        seed=42,  # For reproducibility
    )

    # Print summary
    print_parameters(parameters)

    # Save to file
    output_path = save_parameters(parameters)

    print(f"\nTo use these parameters, run:")
    print(f"  python bend.py")


if __name__ == "__main__":
    main()
