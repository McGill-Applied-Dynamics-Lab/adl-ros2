#!/usr/bin/env python3
"""
Plotting script for wrist rotation experiment data.

Loads torque and angle data from RANDOM_WRIST_ROTATION_DATA.pkl and creates
overlay plots to visualize all measurements together.
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_data(results_file):
    """Load experiment data from pickle file."""
    with open(results_file, "rb") as f:
        exp_dict = pickle.load(f)
    return exp_dict


def plot_torques_overlay(exp_dict, output_path=None):
    """
    Create overlay plot of all measured torques during forward rotations.
    
    Each rotation produces a torque vector. This plots the magnitude or
    a specific component (e.g., z-axis torque) over time.
    
    Args:
        exp_dict: Dictionary with keys 'ts_forward', 'torques_forward', etc.
        output_path: Optional path to save figure
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot forward rotation torques
    ax = axes[0]
    ax.set_title("Forward Rotation - Torque Overlay (All Measurements)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (N·m)")
    ax.grid(True, alpha=0.3)
    
    ts_fwd_list = exp_dict["ts_forward"]
    torques_fwd_list = exp_dict["torques_forward"]
    
    for i, (ts, torques) in enumerate(zip(ts_fwd_list, torques_fwd_list)):
        # Extract z-component (index 2) of torque, or use magnitude
        torque_z = np.array([t[2] for t in torques]) if len(torques[0]) >= 3 else np.array([np.linalg.norm(t) for t in torques])
        ax.plot(ts, torque_z, alpha=0.5, linewidth=1, label=f"Rotation {i+1}")
    
    ax.legend(loc="best", fontsize=8, ncol=2)
    
    # Plot reverse rotation torques
    ax = axes[1]
    ax.set_title("Reverse Rotation - Torque Overlay (All Measurements)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (N·m)")
    ax.grid(True, alpha=0.3)
    
    ts_rev_list = exp_dict["ts_reverse"]
    torques_rev_list = exp_dict["torques_reverse"]
    
    for i, (ts, torques) in enumerate(zip(ts_rev_list, torques_rev_list)):
        # Extract z-component (index 2) of torque, or use magnitude
        torque_z = np.array([t[2] for t in torques]) if len(torques[0]) >= 3 else np.array([np.linalg.norm(t) for t in torques])
        ax.plot(ts, torque_z, alpha=0.5, linewidth=1, label=f"Rotation {i+1}")
    
    ax.legend(loc="best", fontsize=8, ncol=2)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✓ Saved torque plot to: {output_path}")
    
    return fig


def plot_angles_overlay(exp_dict, output_path=None):
    """
    Create overlay plot of all measured joint 7 angles.
    
    Plots absolute joint angles (in radians) over time for all rotations.
    
    Args:
        exp_dict: Dictionary with keys 'ts_forward', 'joint7_angles_forward', etc.
        output_path: Optional path to save figure
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot forward rotation angles
    ax = axes[0]
    ax.set_title("Forward Rotation - Joint 7 Angle Overlay (All Measurements)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Joint 7 Angle (radians)")
    ax.grid(True, alpha=0.3)
    
    ts_fwd_list = exp_dict["ts_forward"]
    angles_fwd_list = exp_dict["joint7_angles_forward"]
    
    for i, (ts, angles) in enumerate(zip(ts_fwd_list, angles_fwd_list)):
        ax.plot(ts, angles, alpha=0.5, linewidth=1, label=f"Rotation {i+1}")
    
    ax.legend(loc="best", fontsize=8, ncol=2)
    
    # Plot reverse rotation angles
    ax = axes[1]
    ax.set_title("Reverse Rotation - Joint 7 Angle Overlay (All Measurements)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Joint 7 Angle (radians)")
    ax.grid(True, alpha=0.3)
    
    ts_rev_list = exp_dict["ts_reverse"]
    angles_rev_list = exp_dict["joint7_angles_reverse"]
    
    for i, (ts, angles) in enumerate(zip(ts_rev_list, angles_rev_list)):
        ax.plot(ts, angles, alpha=0.5, linewidth=1, label=f"Rotation {i+1}")
    
    ax.legend(loc="best", fontsize=8, ncol=2)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"✓ Saved angle plot to: {output_path}")
    
    return fig


def main():
    """Load data and create plots."""
    # File paths
    project_root = Path(__file__).resolve().parent
    results_file = project_root / "results" / "RANDOM_WRIST_ROTATION_DATA.pkl"
    
    # Check if results file exists
    if not results_file.exists():
        print(f"ERROR: Results file not found: {results_file}")
        print("Run twist.py first to generate data.")
        return
    
    # Load data
    print(f"Loading data from {results_file}...")
    exp_dict = load_data(results_file)
    
    # Print summary
    num_rotations = len(exp_dict["target_angles"])
    print(f"✓ Loaded {num_rotations} rotation measurements")
    print(f"  - Target angles: {exp_dict['target_angles']}")
    print(f"  - Target speeds: {exp_dict['target_speeds']}")
    
    # Create output directory
    output_dir = project_root / "results" / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create plots
    print("\nGenerating plots...")

    torque_plot_path = output_dir / "torques_overlay.png"
    plot_torques_overlay(exp_dict, output_path=torque_plot_path)

    angle_plot_path = output_dir / "angles_overlay.png"
    plot_angles_overlay(exp_dict, output_path=angle_plot_path)
    
    print(f"\n✓ Plots saved to: {output_dir}")
    print(f"  - Torque plot: {torque_plot_path.name}")
    print(f"  - Angle plot: {angle_plot_path.name}")
    
    # Show plots (comment out if running headless)
    plt.show()


if __name__ == "__main__":
    main()
