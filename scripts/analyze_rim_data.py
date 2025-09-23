#!/usr/bin/env python3
"""
Script to analyze RIM experiment data from ROS 2 bag files.
"""

import argparse
from pathlib import Path
from typing import List
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

BAG_FOLDER_PATH = Path(__file__).parent.parent / "data"  # Default folder for bag files


class RIMDataAnalyzer:
    """Analyze RIM experiment data from ROS 2 bag files."""

    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        self.data = {}
        self.topics_info = {}

    def load_bag_data(self, topics: List[str] = None):
        """Load data from ROS 2 bag file."""
        # Check if the bag path exists and get the correct URI
        if not self.bag_path.exists():
            raise FileNotFoundError(f"Bag file not found: {self.bag_path}")

        # For compressed bags, rosbag2 automatically handles decompression
        # Just provide the directory path without the specific .db3 file
        bag_uri = str(self.bag_path)
        if self.bag_path.is_file():
            # If a specific file was provided, use its parent directory
            bag_uri = str(self.bag_path.parent)

        storage_options = StorageOptions(uri=bag_uri, storage_id="sqlite3")
        converter_options = ConverterOptions("", "")

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        self.topics_info = {topic.name: topic.type for topic in topic_types}

        # Filter topics if specified
        if topics:
            self.topics_info = {k: v for k, v in self.topics_info.items() if k in topics}

        print(f"Available topics: {list(self.topics_info.keys())}")

        # Initialize data storage
        for topic in self.topics_info.keys():
            self.data[topic] = {"timestamps": [], "messages": []}

        # Read messages
        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic in self.topics_info:
                msg_type = get_message(self.topics_info[topic])
                msg = deserialize_message(data, msg_type)

                self.data[topic]["timestamps"].append(timestamp / 1e9)  # Convert to seconds
                self.data[topic]["messages"].append(msg)

        # SequentialReader doesn't need explicit closing in newer versions
        print(f"Loaded {sum(len(v['messages']) for v in self.data.values())} messages")

    def list_available_topics(self):
        """List all available topics in the bag file."""
        print("\nAvailable topics in bag file:")
        for topic, msg_type in self.topics_info.items():
            count = len(self.data.get(topic, {}).get("messages", []))
            print(f"  {topic} ({msg_type}) - {count} messages")
        print()

    def extract_force_data(self) -> pd.DataFrame:
        """Extract force data from /rim/interface_force topic."""
        topic = "/rim/interface_force"
        if topic not in self.data:
            print(f"Warning: {topic} not found in bag data")
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        force_data = []
        for ts, msg in zip(timestamps, messages):
            force_data.append(
                {
                    "timestamp": ts,
                    "force_x": msg.wrench.force.x,
                    "force_y": msg.wrench.force.y,
                    "force_z": msg.wrench.force.z,
                    "torque_x": msg.wrench.torque.x,
                    "torque_y": msg.wrench.torque.y,
                    "torque_z": msg.wrench.torque.z,
                }
            )

        return pd.DataFrame(force_data)

    def extract_position_data(self) -> pd.DataFrame:
        """Extract position data from /rim/pose topic."""
        topic = "/rim/pose"
        if topic not in self.data:
            print(f"Warning: {topic} not found in bag data")
            # Try alternative topic name
            topic = "/rim/interface_position"
            if topic not in self.data:
                print(f"Warning: {topic} not found in bag data either")
                return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        position_data = []
        for ts, msg in zip(timestamps, messages):
            position_data.append(
                {
                    "timestamp": ts,
                    "pos_x": msg.pose.position.x,
                    "pos_y": msg.pose.position.y,
                    "pos_z": msg.pose.position.z,
                    "quat_x": msg.pose.orientation.x,
                    "quat_y": msg.pose.orientation.y,
                    "quat_z": msg.pose.orientation.z,
                    "quat_w": msg.pose.orientation.w,
                }
            )

        return pd.DataFrame(position_data)

    def extract_joint_states(self) -> pd.DataFrame:
        """Extract joint state data."""
        topic = "/joint_states"
        if topic not in self.data:
            print(f"Warning: {topic} not found in bag data")
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        joint_data = []
        for ts, msg in zip(timestamps, messages):
            row = {"timestamp": ts}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    row[f"{name}_pos"] = msg.position[i]
                if i < len(msg.velocity):
                    row[f"{name}_vel"] = msg.velocity[i]
                if i < len(msg.effort):
                    row[f"{name}_effort"] = msg.effort[i]
            joint_data.append(row)

        return pd.DataFrame(joint_data)

    def plot_force_data(self, save_path: str = None):
        """Plot force and torque data."""
        force_df = self.extract_force_data()
        if force_df.empty:
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

        # Normalize timestamps to start from 0
        t_start = force_df["timestamp"].min()
        time = force_df["timestamp"] - t_start

        # Plot forces
        ax1.plot(time, force_df["force_x"], label="F_x", alpha=0.8)
        ax1.plot(time, force_df["force_y"], label="F_y", alpha=0.8)
        ax1.plot(time, force_df["force_z"], label="F_z", alpha=0.8)
        ax1.set_ylabel("Force [N]")
        ax1.set_title("Interface Forces")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot torques
        ax2.plot(time, force_df["torque_x"], label="τ_x", alpha=0.8)
        ax2.plot(time, force_df["torque_y"], label="τ_y", alpha=0.8)
        ax2.plot(time, force_df["torque_z"], label="τ_z", alpha=0.8)
        ax2.set_ylabel("Torque [Nm]")
        ax2.set_xlabel("Time [s]")
        ax2.set_title("Interface Torques")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"Force plot saved to {save_path}")
        else:
            plt.show()

    def plot_position_data(self, save_path: str = None):
        """Plot position data."""
        pos_df = self.extract_position_data()
        if pos_df.empty:
            return

        fig, ax = plt.subplots(1, 1, figsize=(10, 6))

        # Normalize timestamps
        t_start = pos_df["timestamp"].min()
        time = pos_df["timestamp"] - t_start

        ax.plot(time, pos_df["pos_x"], label="X", alpha=0.8)
        ax.plot(time, pos_df["pos_y"], label="Y", alpha=0.8)
        ax.plot(time, pos_df["pos_z"], label="Z", alpha=0.8)
        ax.set_ylabel("Position [m]")
        ax.set_xlabel("Time [s]")
        ax.set_title("End-Effector Position")
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"Position plot saved to {save_path}")
        else:
            plt.show()

    def generate_summary_report(self, output_path: str = None):
        """Generate a summary report of the experiment."""
        report = {
            "experiment_summary": {
                "bag_path": str(self.bag_path),
                "topics_recorded": list(self.topics_info.keys()),
                "total_messages": sum(len(v["messages"]) for v in self.data.values()),
            }
        }

        # Add force statistics
        force_df = self.extract_force_data()
        if not force_df.empty:
            duration = force_df["timestamp"].max() - force_df["timestamp"].min()
            report["force_analysis"] = {
                "duration_seconds": duration,
                "message_count": len(force_df),
                "avg_frequency_hz": len(force_df) / duration if duration > 0 else 0,
                "force_stats": {
                    "max_force_magnitude": np.sqrt(
                        force_df["force_x"] ** 2 + force_df["force_y"] ** 2 + force_df["force_z"] ** 2
                    ).max(),
                    "mean_force_x": force_df["force_x"].mean(),
                    "mean_force_y": force_df["force_y"].mean(),
                    "mean_force_z": force_df["force_z"].mean(),
                },
            }

        if output_path:
            with open(output_path, "w") as f:
                yaml.dump(report, f, default_flow_style=False)
            print(f"Summary report saved to {output_path}")
        else:
            print(yaml.dump(report, default_flow_style=False))

        return report


def main():
    parser = argparse.ArgumentParser(description="Analyze RIM experiment data")
    parser.add_argument("bag_file_name", help="Name of the ROS 2 bag directory (without path)")
    parser.add_argument("--topics", nargs="+", help="Specific topics to analyze (default: all)")
    parser.add_argument(
        "--output-dir", "-o", default="./analysis_output", help="Output directory for plots and reports"
    )
    parser.add_argument("--show-plots", action="store_true", help="Display plots interactively")

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True, parents=True)

    # Initialize analyzer - bag_file_name should be a directory name
    bag_file_path: Path = BAG_FOLDER_PATH / args.bag_file_name

    if not bag_file_path.exists():
        print(f"Error: Bag directory not found: {bag_file_path}")
        print(f"Available bags in {BAG_FOLDER_PATH}:")
        if BAG_FOLDER_PATH.exists():
            for item in BAG_FOLDER_PATH.iterdir():
                if item.is_dir() and item.name.startswith("franka_rim_data_"):
                    print(f"  {item.name}")
        return

    if not bag_file_path.is_dir():
        print(f"Error: {bag_file_path} is not a directory. Please provide the bag directory name.")
        return

    topics_to_load = [
        "/fr3/current_pose",
        "/haptic_pose",
        "/fr3_rim",
        "/rim_pose",
        "/rim/interface_force",
        "/fr3/interface_force",
    ]

    # Analyze data
    analyzer = RIMDataAnalyzer(bag_file_path)

    try:
        # Load data
        analyzer.load_bag_data(topics=topics_to_load)

        # Show available topics for user reference
        analyzer.list_available_topics()

        # Generate plots
        bag_name = bag_file_path.name
        force_plot_path = None
        position_plot_path = None
        if not args.show_plots:
            plot_dir = output_dir / bag_name
            plot_dir.mkdir(exist_ok=True, parents=True)
            force_plot_path = plot_dir / "forces.png"
            position_plot_path = plot_dir / "positions.png"

        analyzer.plot_force_data(save_path=force_plot_path)
        analyzer.plot_position_data(save_path=position_plot_path)

        # Generate summary report
        report_path = output_dir / f"{bag_name}_summary.yaml"
        analyzer.generate_summary_report(output_path=report_path)

        print("Analysis completed successfully!")

    except Exception as e:
        print(f"Error during analysis: {e}")
        print("This might be due to:")
        print("1. Corrupted bag file")
        print("2. Missing dependencies (try: pip install rosbag2-py)")
        print("3. Bag file still being written to")
        return


if __name__ == "__main__":
    main()
