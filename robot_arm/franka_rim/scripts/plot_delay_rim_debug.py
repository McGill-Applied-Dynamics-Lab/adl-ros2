#!/usr/bin/env python3
"""
Script to plot DelayRIM debug data from CSV files.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
import sys


def plot_delay_rim_debug(csv_file: Path, save_plots: bool = False):
    """Plot DelayRIM debug data from CSV file"""

    # Read CSV data
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    print(f"Loaded {len(df)} debug entries from {csv_file}")
    print(f"Packets: {df['packet_id'].nunique()}")

    # Get unique packet IDs
    packet_ids = df["packet_id"].unique()

    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f"DelayRIM Debug Analysis - {csv_file.name}", fontsize=16)

    # Color palette for different packets
    colors = plt.cm.tab10(np.linspace(0, 1, len(packet_ids)))

    for i, packet_id in enumerate(packet_ids):
        packet_data = df[df["packet_id"] == packet_id]
        color = colors[i % len(colors)]
        label = f"Packet {packet_id} ({packet_data.iloc[0]['delay_ms']:.1f}ms)"

        # Plot 1: Position trajectories
        axes[0, 0].plot(
            packet_data["step_number"],
            packet_data["haptic_position"],
            "--",
            color=color,
            alpha=0.7,
            label=f"{label} - Haptic",
        )
        axes[0, 0].plot(
            packet_data["step_number"],
            packet_data["real_mass_position"],
            "-",
            color=color,
            alpha=0.9,
            label=f"{label} - Real Mass",
        )
        axes[0, 0].plot(
            packet_data["step_number"],
            packet_data["estimated_rim_position"],
            ":",
            color=color,
            alpha=0.9,
            linewidth=2,
            label=f"{label} - Estimated",
        )

        # Plot 2: Position error
        position_error = packet_data["estimated_rim_position"] - packet_data["real_mass_position"]
        axes[0, 1].plot(packet_data["step_number"], position_error, "-", color=color, label=label)

        # Plot 3: Velocity trajectories
        axes[1, 0].plot(
            packet_data["step_number"],
            packet_data["haptic_velocity"],
            "--",
            color=color,
            alpha=0.7,
            label=f"{label} - Haptic",
        )
        axes[1, 0].plot(
            packet_data["step_number"],
            packet_data["real_mass_velocity"],
            "-",
            color=color,
            alpha=0.9,
            label=f"{label} - Real Mass",
        )
        axes[1, 0].plot(
            packet_data["step_number"],
            packet_data["estimated_rim_velocity"],
            ":",
            color=color,
            alpha=0.9,
            linewidth=2,
            label=f"{label} - Estimated",
        )

        # Plot 4: Velocity error
        velocity_error = packet_data["estimated_rim_velocity"] - packet_data["real_mass_velocity"]
        axes[1, 1].plot(packet_data["step_number"], velocity_error, "-", color=color, label=label)

        # Plot 5: Interface force
        axes[2, 0].plot(packet_data["step_number"], packet_data["interface_force"], "-", color=color, label=label)

        # Plot 6: Final accuracy summary
        if i == 0:  # Only plot once
            final_errors = []
            delays = []
            for pid in packet_ids:
                pdata = df[df["packet_id"] == pid]
                final_pos_error = abs(pdata.iloc[-1]["estimated_rim_position"] - pdata.iloc[-1]["real_mass_position"])
                final_errors.append(final_pos_error)
                delays.append(pdata.iloc[0]["delay_ms"])

            axes[2, 1].scatter(delays, final_errors, s=50, alpha=0.7)
            axes[2, 1].set_xlabel("Delay (ms)")
            axes[2, 1].set_ylabel("Final Position Error")
            axes[2, 1].set_title("Final Estimation Accuracy vs Delay")
            axes[2, 1].grid(True)

    # Configure subplots
    axes[0, 0].set_xlabel("Integration Step")
    axes[0, 0].set_ylabel("Position")
    axes[0, 0].set_title("Position Trajectories During Catch-up")
    axes[0, 0].legend(bbox_to_anchor=(1.05, 1), loc="upper left")
    axes[0, 0].grid(True)

    axes[0, 1].set_xlabel("Integration Step")
    axes[0, 1].set_ylabel("Position Error")
    axes[0, 1].set_title("Position Estimation Error")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].set_xlabel("Integration Step")
    axes[1, 0].set_ylabel("Velocity")
    axes[1, 0].set_title("Velocity Trajectories During Catch-up")
    axes[1, 0].legend(bbox_to_anchor=(1.05, 1), loc="upper left")
    axes[1, 0].grid(True)

    axes[1, 1].set_xlabel("Integration Step")
    axes[1, 1].set_ylabel("Velocity Error")
    axes[1, 1].set_title("Velocity Estimation Error")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    axes[2, 0].set_xlabel("Integration Step")
    axes[2, 0].set_ylabel("Interface Force")
    axes[2, 0].set_title("Interface Force During Catch-up")
    axes[2, 0].legend()
    axes[2, 0].grid(True)

    plt.tight_layout()

    if save_plots:
        plot_file = csv_file.with_suffix(".png")
        plt.savefig(plot_file, dpi=300, bbox_inches="tight")
        print(f"Plot saved to: {plot_file}")

    plt.show()

    # Print summary statistics
    print("\nSummary Statistics:")
    print("==================")
    for packet_id in packet_ids:
        packet_data = df[df["packet_id"] == packet_id]
        final_pos_error = abs(
            packet_data.iloc[-1]["estimated_rim_position"] - packet_data.iloc[-1]["real_mass_position"]
        )
        delay = packet_data.iloc[0]["delay_ms"]
        computation_time = packet_data.iloc[-1]["computation_time_ms"]

        print(
            f"Packet {packet_id}: {delay:.1f}ms delay, "
            f"final error: {final_pos_error:.4f}, "
            f"computation time: {computation_time:.2f}ms"
        )


def main():
    parser = argparse.ArgumentParser(description="Plot DelayRIM debug data")
    parser.add_argument("csv_file", type=str, help="Path to CSV debug file")
    parser.add_argument("--save", action="store_true", help="Save plots to PNG file")

    args = parser.parse_args()

    csv_file = Path(args.csv_file)
    if not csv_file.exists():
        print(f"Error: CSV file not found: {csv_file}")
        # Try to find the latest file in /tmp/delay_rim_debug/
        debug_dir = Path("/tmp/delay_rim_debug")
        if debug_dir.exists():
            csv_files = list(debug_dir.glob("delay_rim_debug_*.csv"))
            if csv_files:
                latest_file = max(csv_files, key=lambda f: f.stat().st_mtime)
                print(f"Using latest debug file: {latest_file}")
                csv_file = latest_file
            else:
                print("No debug files found in /tmp/delay_rim_debug/")
                sys.exit(1)
        else:
            sys.exit(1)

    plot_delay_rim_debug(csv_file, save_plots=args.save)


if __name__ == "__main__":
    main()
