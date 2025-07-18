import csv
import time
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.node import Node


@dataclass
class DelayRIMDebugEntry:
    """Single entry in the DelayRIM debug trajectory"""

    packet_id: int
    timestamp: float
    step_number: int
    total_steps: int
    delay_ms: float

    # Positions during catch-up
    haptic_position: float
    real_mass_position: float
    estimated_rim_position: float

    # Velocities during catch-up
    haptic_velocity: float
    real_mass_velocity: float
    estimated_rim_velocity: float

    # Forces
    interface_force: float

    # Debug info
    is_final_step: bool = False
    computation_time_ms: float = 0.0


class DelayRIMDebugger:
    """Handles debugging and visualization of DelayRIM catch-up trajectories"""

    def __init__(self, node: Node, enable_csv: bool = True, enable_rviz: bool = True):
        self.node = node
        self.enable_csv = enable_csv
        self.enable_rviz = enable_rviz

        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv:
            self._setup_csv_logging()

        # RViz visualization
        if self.enable_rviz:
            self._debug_marker_pub = node.create_publisher(MarkerArray, "/delay_rim_debug/trajectory", 10)

        # Debug data storage
        self.current_trajectory: List[DelayRIMDebugEntry] = []
        self.trajectory_history: List[List[DelayRIMDebugEntry]] = []

    def _setup_csv_logging(self):
        """Setup CSV file for logging debug data"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_dir = Path("/tmp/delay_rim_debug")
        log_dir.mkdir(exist_ok=True)

        self.csv_file = log_dir / f"delay_rim_debug_{timestamp}.csv"

        # Create CSV file with headers
        with open(self.csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "packet_id",
                    "timestamp",
                    "step_number",
                    "total_steps",
                    "delay_ms",
                    "haptic_position",
                    "real_mass_position",
                    "estimated_rim_position",
                    "haptic_velocity",
                    "real_mass_velocity",
                    "estimated_rim_velocity",
                    "interface_force",
                    "is_final_step",
                    "computation_time_ms",
                ]
            )

        self.node.get_logger().info(f"DelayRIM debug CSV logging to: {self.csv_file}")

    def start_trajectory(self, packet_id: int, delay_ms: float, total_steps: int):
        """Start logging a new catch-up trajectory"""
        self.current_trajectory = []
        self.node.get_logger().debug(
            f"Starting DelayRIM debug trajectory for packet {packet_id}, delay={delay_ms:.1f}ms, steps={total_steps}"
        )

    def log_step(
        self,
        packet_id: int,
        step_number: int,
        total_steps: int,
        delay_ms: float,
        haptic_position: float,
        real_mass_position: float,
        estimated_rim_position: float,
        haptic_velocity: float,
        real_mass_velocity: float,
        estimated_rim_velocity: float,
        interface_force: float,
        is_final_step: bool = False,
        computation_time_ms: float = 0.0,
    ):
        """Log a single step of the catch-up trajectory"""

        entry = DelayRIMDebugEntry(
            packet_id=packet_id,
            timestamp=time.time(),
            step_number=step_number,
            total_steps=total_steps,
            delay_ms=delay_ms,
            haptic_position=haptic_position,
            real_mass_position=real_mass_position,
            estimated_rim_position=estimated_rim_position,
            haptic_velocity=haptic_velocity,
            real_mass_velocity=real_mass_velocity,
            estimated_rim_velocity=estimated_rim_velocity,
            interface_force=interface_force,
            is_final_step=is_final_step,
            computation_time_ms=computation_time_ms,
        )

        self.current_trajectory.append(entry)

        # Log to CSV
        if self.enable_csv and self.csv_file:
            self._log_to_csv(entry)

    def _log_to_csv(self, entry: DelayRIMDebugEntry):
        """Write entry to CSV file"""
        try:
            with open(self.csv_file, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        entry.packet_id,
                        entry.timestamp,
                        entry.step_number,
                        entry.total_steps,
                        entry.delay_ms,
                        entry.haptic_position,
                        entry.real_mass_position,
                        entry.estimated_rim_position,
                        entry.haptic_velocity,
                        entry.real_mass_velocity,
                        entry.estimated_rim_velocity,
                        entry.interface_force,
                        entry.is_final_step,
                        entry.computation_time_ms,
                    ]
                )
        except Exception as e:
            self.node.get_logger().error(f"Failed to write to CSV: {e}")

    def finish_trajectory(self):
        """Finish current trajectory and publish visualization"""
        if not self.current_trajectory:
            return

        # Store trajectory in history
        self.trajectory_history.append(self.current_trajectory.copy())

        # Keep only last 5 trajectories for visualization
        if len(self.trajectory_history) > 5:
            self.trajectory_history.pop(0)

        # Publish RViz visualization
        if self.enable_rviz:
            self._publish_rviz_trajectory()

        # Log summary
        final_entry = self.current_trajectory[-1]
        self.node.get_logger().info(
            f"DelayRIM trajectory complete - Packet {final_entry.packet_id}: "
            f"Final estimated position: {final_entry.estimated_rim_position:.3f}, "
            f"Real position: {final_entry.real_mass_position:.3f}, "
            f"Error: {abs(final_entry.estimated_rim_position - final_entry.real_mass_position):.3f}"
        )

    def _publish_rviz_trajectory(self):
        """Publish trajectory markers for RViz visualization"""
        if not self.trajectory_history:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Color palette for different trajectories
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),  # Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),  # Blue
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),  # Yellow
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),  # Magenta
        ]

        for traj_idx, trajectory in enumerate(self.trajectory_history):
            if not trajectory:
                continue

            color = colors[traj_idx % len(colors)]

            # Create trajectory line for estimated RIM position
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = self.node.get_clock().now().to_msg()
            line_marker.ns = "delay_rim_debug"
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # Add points for the estimated trajectory
            for entry in trajectory:
                point = Point()
                point.x = entry.estimated_rim_position
                point.y = 0.1 + traj_idx * 0.05  # Offset different trajectories
                point.z = 0.0
                line_marker.points.append(point)

            line_marker.scale.x = 0.005  # Line width
            line_marker.color = color

            marker_array.markers.append(line_marker)

            # Add start and end markers
            if trajectory:
                # Start marker (small sphere)
                start_marker = Marker()
                start_marker.header.frame_id = "world"
                start_marker.header.stamp = self.node.get_clock().now().to_msg()
                start_marker.ns = "delay_rim_debug"
                start_marker.id = marker_id
                marker_id += 1
                start_marker.type = Marker.SPHERE
                start_marker.action = Marker.ADD

                start_marker.pose.position.x = trajectory[0].estimated_rim_position
                start_marker.pose.position.y = 0.1 + traj_idx * 0.05
                start_marker.pose.position.z = 0.0
                start_marker.pose.orientation.w = 1.0

                start_marker.scale.x = 0.02
                start_marker.scale.y = 0.02
                start_marker.scale.z = 0.02
                start_marker.color = color

                marker_array.markers.append(start_marker)

                # End marker (larger sphere)
                end_marker = Marker()
                end_marker.header.frame_id = "world"
                end_marker.header.stamp = self.node.get_clock().now().to_msg()
                end_marker.ns = "delay_rim_debug"
                end_marker.id = marker_id
                marker_id += 1
                end_marker.type = Marker.SPHERE
                end_marker.action = Marker.ADD

                end_marker.pose.position.x = trajectory[-1].estimated_rim_position
                end_marker.pose.position.y = 0.1 + traj_idx * 0.05
                end_marker.pose.position.z = 0.0
                end_marker.pose.orientation.w = 1.0

                end_marker.scale.x = 0.04
                end_marker.scale.y = 0.04
                end_marker.scale.z = 0.04
                end_marker.color = color

                marker_array.markers.append(end_marker)

        self._debug_marker_pub.publish(marker_array)
    def get_csv_file_path(self) -> Optional[Path]:
        """Get the path to the CSV log file"""
        return self.csv_file if self.enable_csv else None
