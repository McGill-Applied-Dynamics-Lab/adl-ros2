#!/usr/bin/env python3
"""
Script to process ROS 2 bag files and convert them to pandas DataFrames.
Supports multiple configurations and automatically detects processed files.
"""

import argparse
from pathlib import Path
from typing import Dict, Any
import pandas as pd
import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Default paths
BAG_FOLDER_PATH = Path(__file__).parent.parent / "data"
CONFIG_FILE_PATH = Path(__file__).parent / "processing_config.yaml"


class BagDataProcessor:
    """Process ROS 2 bag files and convert to pandas DataFrames."""

    def __init__(self, bag_path: Path):
        self.bag_path = bag_path
        self.data = {}
        self.topics_info = {}

    def load_bag_data(self, topics: Dict[str, str]):
        """Load data from ROS 2 bag file for specified topics.

        Args:
            topics: Dictionary mapping topic names to ROS topic paths
                   e.g., {"rim_force": "/rim/interface_force"}
        """
        if not self.bag_path.exists():
            raise FileNotFoundError(f"Bag file not found: {self.bag_path}")

        storage_options = StorageOptions(uri=str(self.bag_path), storage_id="sqlite3")
        converter_options = ConverterOptions("", "")

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Get all available topics
        topic_types = reader.get_all_topics_and_types()
        all_topics_info = {topic.name: topic.type for topic in topic_types}

        # Filter to only requested topics
        ros_topics = list(topics.values())
        self.topics_info = {topic: msg_type for topic, msg_type in all_topics_info.items() if topic in ros_topics}

        print(f"Processing topics: {list(self.topics_info.keys())}")

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

        print(f"Loaded {sum(len(v['messages']) for v in self.data.values())} messages")

    def extract_wrench_stamped(self, topic: str) -> pd.DataFrame:
        """Extract data from WrenchStamped messages."""
        if topic not in self.data:
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        wrench_data = []
        for ts, msg in zip(timestamps, messages):
            wrench_data.append(
                {
                    "timestamp": ts,
                    "force_x": msg.wrench.force.x,
                    "force_y": msg.wrench.force.y,
                    "force_z": msg.wrench.force.z,
                    "torque_x": msg.wrench.torque.x,
                    "torque_y": msg.wrench.torque.y,
                    "torque_z": msg.wrench.torque.z,
                    "frame_id": msg.header.frame_id if hasattr(msg, "header") else "",
                }
            )

        return pd.DataFrame(wrench_data)

    def extract_pose_stamped(self, topic: str) -> pd.DataFrame:
        """Extract data from PoseStamped messages."""
        if topic not in self.data:
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        pose_data = []
        for ts, msg in zip(timestamps, messages):
            pose_data.append(
                {
                    "timestamp": ts,
                    "pos_x": msg.pose.position.x,
                    "pos_y": msg.pose.position.y,
                    "pos_z": msg.pose.position.z,
                    "quat_x": msg.pose.orientation.x,
                    "quat_y": msg.pose.orientation.y,
                    "quat_z": msg.pose.orientation.z,
                    "quat_w": msg.pose.orientation.w,
                    "frame_id": msg.header.frame_id if hasattr(msg, "header") else "",
                }
            )

        return pd.DataFrame(pose_data)

    def extract_twist_stamped(self, topic: str) -> pd.DataFrame:
        """Extract data from TwistStamped messages."""
        if topic not in self.data:
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        twist_data = []
        for ts, msg in zip(timestamps, messages):
            twist_data.append(
                {
                    "timestamp": ts,
                    "linear_x": msg.twist.linear.x,
                    "linear_y": msg.twist.linear.y,
                    "linear_z": msg.twist.linear.z,
                    "angular_x": msg.twist.angular.x,
                    "angular_y": msg.twist.angular.y,
                    "angular_z": msg.twist.angular.z,
                    "frame_id": msg.header.frame_id if hasattr(msg, "header") else "",
                }
            )

        return pd.DataFrame(twist_data)

    def extract_joint_state(self, topic: str) -> pd.DataFrame:
        """Extract data from JointState messages."""
        if topic not in self.data:
            return pd.DataFrame()

        timestamps = self.data[topic]["timestamps"]
        messages = self.data[topic]["messages"]

        joint_data = []
        for ts, msg in zip(timestamps, messages):
            row = {"timestamp": ts}

            # Add joint positions, velocities, and efforts
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    row[f"{name}_pos"] = msg.position[i]
                if i < len(msg.velocity):
                    row[f"{name}_vel"] = msg.velocity[i]
                if i < len(msg.effort):
                    row[f"{name}_effort"] = msg.effort[i]

            joint_data.append(row)

        return pd.DataFrame(joint_data)

    def process_topics(self, topics_config: Dict[str, str]) -> Dict[str, pd.DataFrame]:
        """Process all topics and return dictionary of DataFrames."""
        dataframes = {}

        for topic_name, ros_topic in topics_config.items():
            if ros_topic not in self.topics_info:
                print(f"Warning: Topic {ros_topic} not found in bag data")
                continue

            msg_type = self.topics_info[ros_topic]
            print(f"Processing {topic_name} ({ros_topic}) - Type: {msg_type}")

            # Extract data based on message type
            if "WrenchStamped" in msg_type:
                df = self.extract_wrench_stamped(ros_topic)
            elif "PoseStamped" in msg_type:
                df = self.extract_pose_stamped(ros_topic)
            elif "TwistStamped" in msg_type:
                df = self.extract_twist_stamped(ros_topic)
            elif "JointState" in msg_type:
                df = self.extract_joint_state(ros_topic)
            else:
                print(f"Warning: Unsupported message type {msg_type} for topic {ros_topic}")
                continue

            if not df.empty:
                dataframes[topic_name] = df
                print(f"  Extracted {len(df)} messages")
            else:
                print(f"  No data extracted for {topic_name}")

        return dataframes


def load_config(config_file: Path) -> Dict[str, Any]:
    """Load processing configuration from YAML file."""
    if not config_file.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_file}")

    with open(config_file, "r") as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="Process ROS 2 bag files to DataFrames")
    parser.add_argument(
        "--config", "-c", type=str, default="default", help="Configuration name to use from config file"
    )
    parser.add_argument("--config-file", type=Path, default=CONFIG_FILE_PATH, help="Path to configuration YAML file")
    parser.add_argument("--force", action="store_true", help="Force reprocessing even if output file exists")

    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config_file)

    if args.config not in config:
        print(f"Error: Configuration '{args.config}' not found in {args.config_file}")
        print(f"Available configurations: {list(config.keys())}")
        return

    config_data = config[args.config]
    experiments = config_data.get("experiments", [])
    topics = config_data.get("topics", {})
    output_filename = config_data.get("output_filename", "processed_data.parquet")

    print(f"Using configuration: {args.config}")
    print(f"Processing {len(experiments)} experiments")
    print(f"Topics to extract: {list(topics.keys())}")

    # Process each experiment
    for experiment in experiments:
        bag_path = BAG_FOLDER_PATH / experiment
        output_path = bag_path / output_filename

        # Skip if already processed and not forcing
        if output_path.exists() and not args.force:
            print(f"Skipping {experiment} - already processed (use --force to override)")
            continue

        print(f"\nProcessing experiment: {experiment}")

        if not bag_path.exists():
            print(f"Warning: Bag directory not found: {bag_path}")
            continue

        try:
            # Process the bag file
            processor = BagDataProcessor(bag_path)
            processor.load_bag_data(topics)
            dataframes = processor.process_topics(topics)

            if dataframes:
                # Create output data with metadata
                output_data = {
                    "metadata": {
                        "experiment": experiment,
                        "bag_path": str(bag_path),
                        "topics": topics,
                        "processing_timestamp": pd.Timestamp.now().isoformat(),
                    },
                    "dataframes": dataframes,
                }

                # Save using pickle for easy loading
                pd.to_pickle(output_data, output_path)

                print(f"Saved processed data to: {output_path}")
                print(f"DataFrames: {list(dataframes.keys())}")
                print(f"Total rows: {sum(len(df) for df in dataframes.values())}")

        except Exception as e:
            print(f"Error processing {experiment}: {e}")
            continue

    print("\nProcessing complete!")


if __name__ == "__main__":
    main()
