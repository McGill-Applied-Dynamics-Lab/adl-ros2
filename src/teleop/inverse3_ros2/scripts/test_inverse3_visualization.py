#!/usr/bin/env python3
"""
Test script for Inverse3 WebSocket interface with real-time visualization.

This script connects to the Inverse3 device via WebSocket and displays:
- Real-time 3D position visualization
- Position and velocity values
- Connection status
- Basic force feedback testing

Usage:
    python test_inverse3_visualization.py [--uri ws://localhost:10001]
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

from inverse3_ros2.websocket_inverse3 import Inverse3


class Inverse3Visualizer:
    """Real-time visualizer for Inverse3 haptic device."""

    def __init__(self, uri: str = "ws://localhost:10001", max_points: int = 100):
        """
        Initialize the visualizer.

        Args:
            uri: WebSocket URI for the Inverse3 service
            max_points: Maximum number of points to keep in trajectory
        """
        self.inverse3 = Inverse3(uri)
        self.max_points = max_points

        # Data storage for trajectory
        self.positions = deque(maxlen=max_points)
        self.velocities = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)

        # Current state
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.is_connected = False

        # Threading for data collection
        self.data_thread = None
        self.running = False
        self.data_lock = threading.Lock()

        # Force testing
        self.force_amplitude = 0.5  # N
        self.force_frequency = 0.5  # Hz
        self.apply_test_forces = False

        # Setup matplotlib with optimization
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.canvas.toolbar_visible = False  # Hide toolbar for cleaner display
        self._setup_plots()
        
        # Plot elements for efficient updates
        self.plot_elements = {}
        self._init_plot_elements()
        
        # Update counter for status refresh
        self._update_counter = 0
        self._status_update_interval = 10  # Update status every 10 frames

    def _setup_plots(self):
        """Setup matplotlib subplots."""
        # 3D position plot
        self.ax_3d = self.fig.add_subplot(2, 2, 1, projection="3d")
        self.ax_3d.set_title("End-Effector Position (3D)")
        self.ax_3d.set_xlabel("X (m)")
        self.ax_3d.set_ylabel("Y (m)")
        self.ax_3d.set_zlabel("Z (m)")

        # Position over time
        self.ax_pos = self.fig.add_subplot(2, 2, 2)
        self.ax_pos.set_title("Position vs Time")
        self.ax_pos.set_xlabel("Time (s)")
        self.ax_pos.set_ylabel("Position (m)")
        self.ax_pos.grid(True)

        # Velocity over time
        self.ax_vel = self.fig.add_subplot(2, 2, 3)
        self.ax_vel.set_title("Velocity vs Time")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (m/s)")
        self.ax_vel.grid(True)

        # Status text
        self.ax_status = self.fig.add_subplot(2, 2, 4)
        self.ax_status.axis("off")

        plt.tight_layout()

    def _init_plot_elements(self):
        """Initialize plot elements for efficient updates."""
        # 3D plot elements
        self.plot_elements['trajectory_3d'], = self.ax_3d.plot([], [], [], 'b-', alpha=0.6, linewidth=1, label='Trajectory')
        self.plot_elements['current_pos_3d'] = self.ax_3d.scatter([], [], [], c='red', s=100, label='Current Position')
        self.ax_3d.legend()
        
        # Position plot elements
        self.plot_elements['pos_x'], = self.ax_pos.plot([], [], 'r-', label='X', linewidth=2)
        self.plot_elements['pos_y'], = self.ax_pos.plot([], [], 'g-', label='Y', linewidth=2)
        self.plot_elements['pos_z'], = self.ax_pos.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax_pos.legend()
        
        # Velocity plot elements
        self.plot_elements['vel_x'], = self.ax_vel.plot([], [], 'r-', label='Vx', linewidth=2)
        self.plot_elements['vel_y'], = self.ax_vel.plot([], [], 'g-', label='Vy', linewidth=2)
        self.plot_elements['vel_z'], = self.ax_vel.plot([], [], 'b-', label='Vz', linewidth=2)
        self.ax_vel.legend()
        
        # Status text element
        self.plot_elements['status_text'] = self.ax_status.text(
            0.05, 0.95, "", transform=self.ax_status.transAxes,
            fontsize=10, verticalalignment="top", fontfamily="monospace"
        )

    def start(self) -> bool:
        """Start the device connection and data collection."""
        print("Starting Inverse3 connection...")

        if not self.inverse3.start():
            print("Failed to connect to Inverse3 device!")
            return False

        self.is_connected = True
        print("Connected to Inverse3 device!")

        # Print device info
        device_info = self.inverse3.device_wakeup_dict()
        print(f"Device ID: {device_info.get('device_id')}")
        print(f"Handedness: {device_info.get('handedness')}")

        # Start data collection thread
        self.running = True
        self.data_thread = threading.Thread(target=self._data_collection_loop, daemon=True)
        self.data_thread.start()

        return True

    def stop(self):
        """Stop data collection and device connection."""
        self.running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        self.inverse3.stop()
        self.is_connected = False

    def _data_collection_loop(self):
        """Main data collection loop running in separate thread."""
        start_time = time.time()

        while self.running and self.inverse3.is_connected():
            try:
                # Get current state
                position, velocity = self.inverse3.get_state()
                current_time = time.time() - start_time

                # Apply test forces if enabled
                if self.apply_test_forces:
                    self._apply_test_forces(current_time)

                # Update data with thread safety
                with self.data_lock:
                    self.current_position = position.copy()
                    self.current_velocity = velocity.copy()
                    self.positions.append(position.copy())
                    self.velocities.append(velocity.copy())
                    self.timestamps.append(current_time)

                time.sleep(0.01)  # 100 Hz update rate

            except Exception as e:
                print(f"Error in data collection loop: {e}")
                break

        print("Data collection stopped")

    def _apply_test_forces(self, current_time: float):
        """Apply sinusoidal test forces to demonstrate force feedback."""
        # Simple sinusoidal force in X direction
        force_x = self.force_amplitude * np.sin(2 * np.pi * self.force_frequency * current_time)
        force = np.array([force_x, 0.0, 0.0])
        self.inverse3.apply_force(force)

    def _update_plots(self, frame):
        """Update all plots with current data - optimized version."""
        with self.data_lock:
            if len(self.positions) == 0:
                return self.plot_elements.values()

            positions_array = np.array(list(self.positions))
            velocities_array = np.array(list(self.velocities))
            timestamps_array = np.array(list(self.timestamps))

            current_pos = self.current_position.copy()
            current_vel = self.current_velocity.copy()

        # Update 3D position plot
        if len(positions_array) > 0:
            # Update trajectory line
            self.plot_elements['trajectory_3d'].set_data_3d(
                positions_array[:, 0], positions_array[:, 1], positions_array[:, 2]
            )
            
            # Update current position scatter
            self.plot_elements['current_pos_3d'].remove()
            self.plot_elements['current_pos_3d'] = self.ax_3d.scatter(
                current_pos[0], current_pos[1], current_pos[2], c='red', s=100, label='Current Position'
            )
            
            # Update axis limits only if data range changed significantly
            self._update_3d_limits(positions_array)

        # Update position over time plots
        if len(timestamps_array) > 0:
            self.plot_elements['pos_x'].set_data(timestamps_array, positions_array[:, 0])
            self.plot_elements['pos_y'].set_data(timestamps_array, positions_array[:, 1])
            self.plot_elements['pos_z'].set_data(timestamps_array, positions_array[:, 2])
            
            # Update axis limits for position plot
            self.ax_pos.set_xlim(timestamps_array[0], timestamps_array[-1])
            pos_min, pos_max = np.min(positions_array), np.max(positions_array)
            margin = (pos_max - pos_min) * 0.1 if pos_max != pos_min else 0.01
            self.ax_pos.set_ylim(pos_min - margin, pos_max + margin)

        # Update velocity over time plots
        if len(timestamps_array) > 0:
            self.plot_elements['vel_x'].set_data(timestamps_array, velocities_array[:, 0])
            self.plot_elements['vel_y'].set_data(timestamps_array, velocities_array[:, 1])
            self.plot_elements['vel_z'].set_data(timestamps_array, velocities_array[:, 2])
            
            # Update axis limits for velocity plot
            self.ax_vel.set_xlim(timestamps_array[0], timestamps_array[-1])
            vel_min, vel_max = np.min(velocities_array), np.max(velocities_array)
            margin = (vel_max - vel_min) * 0.1 if vel_max != vel_min else 0.01
            self.ax_vel.set_ylim(vel_min - margin, vel_max + margin)

        # Update status text less frequently for better performance
        self._update_counter += 1
        if self._update_counter % self._status_update_interval == 0:
            self._update_status_text(current_pos, current_vel)

        return self.plot_elements.values()

    def _update_3d_limits(self, positions_array):
        """Update 3D plot limits efficiently."""
        margin = 0.01  # 1cm margin
        
        # Get current limits
        current_xlim = self.ax_3d.get_xlim()
        current_ylim = self.ax_3d.get_ylim()
        current_zlim = self.ax_3d.get_zlim()
        
        # Calculate new limits
        new_limits = []
        current_limits = [current_xlim, current_ylim, current_zlim]
        
        for i in range(3):
            pos_min = np.min(positions_array[:, i]) - margin
            pos_max = np.max(positions_array[:, i]) + margin
            
            # Only update if the change is significant
            if (pos_min < current_limits[i][0] or pos_max > current_limits[i][1] or
                current_limits[i][1] - current_limits[i][0] > 0.1):  # Reset if range is too large
                new_limits.append([pos_min, pos_max])
            else:
                new_limits.append(current_limits[i])
        
        # Apply new limits
        self.ax_3d.set_xlim(new_limits[0])
        self.ax_3d.set_ylim(new_limits[1])
        self.ax_3d.set_zlim(new_limits[2])

    def _update_status_text(self, current_pos, current_vel):
        """Update status text efficiently."""
        status_text = f"""CONNECTION STATUS
Connected: {self.is_connected}
Device ID: {self.inverse3.device_id}
Handedness: {self.inverse3.handedness}

CURRENT STATE
Position: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}] m
Velocity: [{current_vel[0]:.4f}, {current_vel[1]:.4f}, {current_vel[2]:.4f}] m/s
Speed: {np.linalg.norm(current_vel):.4f} m/s

CONTROLS
'f' - Toggle force feedback test
'r' - Reset view
'q' - Quit

Force Test: {"ON" if self.apply_test_forces else "OFF"}
FPS: {1000/50:.1f} (target)"""

        self.plot_elements['status_text'].set_text(status_text)

    def _on_key_press(self, event):
        """Handle keyboard input."""
        if event.key == "f":
            self.apply_test_forces = not self.apply_test_forces
            print(f"Force feedback test: {'ON' if self.apply_test_forces else 'OFF'}")
        elif event.key == "r":
            # Reset 3D view
            self.ax_3d.view_init(elev=20, azim=45)
            print("3D view reset")
        elif event.key == "q":
            print("Quitting...")
            plt.close("all")

    def run_visualization(self):
        """Run the real-time visualization."""
        if not self.start():
            return

        try:
            # Connect keyboard events
            self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

            # Start animation with optimized settings
            ani = animation.FuncAnimation(
                self.fig, 
                self._update_plots, 
                interval=50,  # 20 FPS
                blit=False,   # Disable blitting for 3D plots
                cache_frame_data=False,
                repeat=False
            )

            print("\nVisualization started!")
            print("Controls:")
            print("  'f' - Toggle force feedback test")
            print("  'r' - Reset 3D view")
            print("  'q' - Quit")
            print("Move the haptic device to see the visualization update...")

            # Set window title
            self.fig.canvas.manager.set_window_title("Inverse3 WebSocket Visualization")
            
            plt.show()

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Test Inverse3 WebSocket interface with visualization")
    parser.add_argument("--uri", default="ws://localhost:10001", help="WebSocket URI (default: ws://localhost:10001)")
    parser.add_argument(
        "--max-points", type=int, default=100, help="Maximum trajectory points to display (default: 100)"
    )

    args = parser.parse_args()

    print("Inverse3 WebSocket Test & Visualization")
    print("=" * 40)
    print(f"URI: {args.uri}")
    print(f"Max trajectory points: {args.max_points}")
    print()

    # Create and run visualizer
    visualizer = Inverse3Visualizer(uri=args.uri, max_points=args.max_points)

    try:
        visualizer.run_visualization()
    except Exception as e:
        print(f"Error running visualization: {e}")
        visualizer.stop()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Test script for Inverse3 WebSocket interface with real-time visualization.

This script connects to the Inverse3 device via WebSocket and displays:
- Real-time 3D position visualization
- Position and velocity values
- Connection status
- Basic force feedback testing

Usage:
    python test_inverse3_visualization.py [--uri ws://localhost:10001]
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

# from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from collections import deque
import threading

from inverse3_ros2.websocket_inverse3 import Inverse3


class Inverse3Visualizer:
    """Real-time visualizer for Inverse3 haptic device."""

    def __init__(self, uri: str = "ws://localhost:10001", max_points: int = 100):
        """
        Initialize the visualizer.

        Args:
            uri: WebSocket URI for the Inverse3 service
            max_points: Maximum number of points to keep in trajectory
        """
        self.inverse3 = Inverse3(uri)
        self.max_points = max_points

        # Data storage for trajectory
        self.positions = deque(maxlen=max_points)
        self.velocities = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)

        # Current state
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.is_connected = False

        # Threading for data collection
        self.data_thread = None
        self.running = False
        self.data_lock = threading.Lock()

        # Force testing
        self.force_amplitude = 0.5  # N
        self.force_frequency = 0.5  # Hz
        self.apply_test_forces = False

        # Setup matplotlib with optimization
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.canvas.toolbar_visible = False  # Hide toolbar for cleaner display
        self._setup_plots()
        
        # Plot elements for efficient updates
        self.plot_elements = {}
        self._init_plot_elements()
        
        # Update counter for status refresh
        self._update_counter = 0
        self._status_update_interval = 10  # Update status every 10 frames

    def _setup_plots(self):
        """Setup matplotlib subplots."""
        # 3D position plot
        self.ax_3d = self.fig.add_subplot(2, 2, 1, projection="3d")
        self.ax_3d.set_title("End-Effector Position (3D)")
        self.ax_3d.set_xlabel("X (m)")
        self.ax_3d.set_ylabel("Y (m)")
        self.ax_3d.set_zlabel("Z (m)")

        # Position over time
        self.ax_pos = self.fig.add_subplot(2, 2, 2)
        self.ax_pos.set_title("Position vs Time")
        self.ax_pos.set_xlabel("Time (s)")
        self.ax_pos.set_ylabel("Position (m)")
        self.ax_pos.grid(True)

        # Velocity over time
        self.ax_vel = self.fig.add_subplot(2, 2, 3)
        self.ax_vel.set_title("Velocity vs Time")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (m/s)")
        self.ax_vel.grid(True)

        # Status text
        self.ax_status = self.fig.add_subplot(2, 2, 4)
        self.ax_status.axis("off")

        plt.tight_layout()

    def _init_plot_elements(self):
        """Initialize plot elements for efficient updates."""
        # 3D plot elements
        self.plot_elements['trajectory_3d'], = self.ax_3d.plot([], [], [], 'b-', alpha=0.6, linewidth=1, label='Trajectory')
        self.plot_elements['current_pos_3d'] = self.ax_3d.scatter([], [], [], c='red', s=100, label='Current Position')
        self.ax_3d.legend()
        
        # Position plot elements
        self.plot_elements['pos_x'], = self.ax_pos.plot([], [], 'r-', label='X', linewidth=2)
        self.plot_elements['pos_y'], = self.ax_pos.plot([], [], 'g-', label='Y', linewidth=2)
        self.plot_elements['pos_z'], = self.ax_pos.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax_pos.legend()
        
        # Velocity plot elements
        self.plot_elements['vel_x'], = self.ax_vel.plot([], [], 'r-', label='Vx', linewidth=2)
        self.plot_elements['vel_y'], = self.ax_vel.plot([], [], 'g-', label='Vy', linewidth=2)
        self.plot_elements['vel_z'], = self.ax_vel.plot([], [], 'b-', label='Vz', linewidth=2)
        self.ax_vel.legend()
        
        # Status text element
        self.plot_elements['status_text'] = self.ax_status.text(
            0.05, 0.95, "", transform=self.ax_status.transAxes,
            fontsize=10, verticalalignment="top", fontfamily="monospace"
        )

    def start(self) -> bool:
        """Start the device connection and data collection."""
        print("Starting Inverse3 connection...")

        if not self.inverse3.start():
            print("Failed to connect to Inverse3 device!")
            return False

        self.is_connected = True
        print("Connected to Inverse3 device!")

        # Print device info
        device_info = self.inverse3.device_wakeup_dict()
        print(f"Device ID: {device_info.get('device_id')}")
        print(f"Handedness: {device_info.get('handedness')}")

        # Start data collection thread
        self.running = True
        self.data_thread = threading.Thread(target=self._data_collection_loop, daemon=True)
        self.data_thread.start()

        return True

    def stop(self):
        """Stop data collection and device connection."""
        self.running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        self.inverse3.stop()
        self.is_connected = False

    def _data_collection_loop(self):
        """Main data collection loop running in separate thread."""
        start_time = time.time()

        while self.running and self.inverse3.is_connected():
            try:
                # Get current state
                position, velocity = self.inverse3.get_state()
                current_time = time.time() - start_time

                # Apply test forces if enabled
                if self.apply_test_forces:
                    self._apply_test_forces(current_time)

                # Update data with thread safety
                with self.data_lock:
                    self.current_position = position.copy()
                    self.current_velocity = velocity.copy()
                    self.positions.append(position.copy())
                    self.velocities.append(velocity.copy())
                    self.timestamps.append(current_time)

                time.sleep(0.01)  # 100 Hz update rate

            except Exception as e:
                print(f"Error in data collection loop: {e}")
                break

        print("Data collection stopped")

    def _apply_test_forces(self, current_time: float):
        """Apply sinusoidal test forces to demonstrate force feedback."""
        # Simple sinusoidal force in X direction
        force_x = self.force_amplitude * np.sin(2 * np.pi * self.force_frequency * current_time)
        force = np.array([force_x, 0.0, 0.0])
        self.inverse3.apply_force(force)

    def _update_plots(self, frame):
        """Update all plots with current data - optimized version."""
        with self.data_lock:
            if len(self.positions) == 0:
                return self.plot_elements.values()

            positions_array = np.array(list(self.positions))
            velocities_array = np.array(list(self.velocities))
            timestamps_array = np.array(list(self.timestamps))

            current_pos = self.current_position.copy()
            current_vel = self.current_velocity.copy()

        # Update 3D position plot
        if len(positions_array) > 0:
            # Update trajectory line
            self.plot_elements['trajectory_3d'].set_data_3d(
                positions_array[:, 0], positions_array[:, 1], positions_array[:, 2]
            )
            
            # Update current position scatter
            self.plot_elements['current_pos_3d'].remove()
            self.plot_elements['current_pos_3d'] = self.ax_3d.scatter(
                current_pos[0], current_pos[1], current_pos[2], c='red', s=100, label='Current Position'
            )
            
            # Update axis limits only if data range changed significantly
            self._update_3d_limits(positions_array)

        # Update position over time plots
        if len(timestamps_array) > 0:
            self.plot_elements['pos_x'].set_data(timestamps_array, positions_array[:, 0])
            self.plot_elements['pos_y'].set_data(timestamps_array, positions_array[:, 1])
            self.plot_elements['pos_z'].set_data(timestamps_array, positions_array[:, 2])
            
            # Update axis limits for position plot
            self.ax_pos.set_xlim(timestamps_array[0], timestamps_array[-1])
            pos_min, pos_max = np.min(positions_array), np.max(positions_array)
            margin = (pos_max - pos_min) * 0.1 if pos_max != pos_min else 0.01
            self.ax_pos.set_ylim(pos_min - margin, pos_max + margin)

        # Update velocity over time plots
        if len(timestamps_array) > 0:
            self.plot_elements['vel_x'].set_data(timestamps_array, velocities_array[:, 0])
            self.plot_elements['vel_y'].set_data(timestamps_array, velocities_array[:, 1])
            self.plot_elements['vel_z'].set_data(timestamps_array, velocities_array[:, 2])
            
            # Update axis limits for velocity plot
            self.ax_vel.set_xlim(timestamps_array[0], timestamps_array[-1])
            vel_min, vel_max = np.min(velocities_array), np.max(velocities_array)
            margin = (vel_max - vel_min) * 0.1 if vel_max != vel_min else 0.01
            self.ax_vel.set_ylim(vel_min - margin, vel_max + margin)

        # Update status text less frequently for better performance
        self._update_counter += 1
        if self._update_counter % self._status_update_interval == 0:
            self._update_status_text(current_pos, current_vel)

        return self.plot_elements.values()

    def _update_3d_limits(self, positions_array):
        """Update 3D plot limits efficiently."""
        margin = 0.01  # 1cm margin
        
        # Get current limits
        current_xlim = self.ax_3d.get_xlim()
        current_ylim = self.ax_3d.get_ylim()
        current_zlim = self.ax_3d.get_zlim()
        
        # Calculate new limits
        new_limits = []
        current_limits = [current_xlim, current_ylim, current_zlim]
        
        for i in range(3):
            pos_min = np.min(positions_array[:, i]) - margin
            pos_max = np.max(positions_array[:, i]) + margin
            
            # Only update if the change is significant
            if (pos_min < current_limits[i][0] or pos_max > current_limits[i][1] or
                current_limits[i][1] - current_limits[i][0] > 0.1):  # Reset if range is too large
                new_limits.append([pos_min, pos_max])
            else:
                new_limits.append(current_limits[i])
        
        # Apply new limits
        self.ax_3d.set_xlim(new_limits[0])
        self.ax_3d.set_ylim(new_limits[1])
        self.ax_3d.set_zlim(new_limits[2])

    def _update_status_text(self, current_pos, current_vel):
        """Update status text efficiently."""
        status_text = f"""CONNECTION STATUS
Connected: {self.is_connected}
Device ID: {self.inverse3.device_id}
Handedness: {self.inverse3.handedness}

CURRENT STATE
Position: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}] m
Velocity: [{current_vel[0]:.4f}, {current_vel[1]:.4f}, {current_vel[2]:.4f}] m/s
Speed: {np.linalg.norm(current_vel):.4f} m/s

CONTROLS
'f' - Toggle force feedback test
'r' - Reset view
'q' - Quit

Force Test: {"ON" if self.apply_test_forces else "OFF"}
FPS: {1000/50:.1f} (target)"""

        self.plot_elements['status_text'].set_text(status_text)

    def _on_key_press(self, event):
        """Handle keyboard input."""
        if event.key == "f":
            self.apply_test_forces = not self.apply_test_forces
            print(f"Force feedback test: {'ON' if self.apply_test_forces else 'OFF'}")
        elif event.key == "r":
            # Reset 3D view
            self.ax_3d.view_init(elev=20, azim=45)
            print("3D view reset")
        elif event.key == "q":
            print("Quitting...")
            plt.close("all")

    def run_visualization(self):
        """Run the real-time visualization."""
        if not self.start():
            return

        try:
            # Connect keyboard events
            self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

            # Start animation with optimized settings
            ani = animation.FuncAnimation(
                self.fig, 
                self._update_plots,interval=50,  # 20 FPS                
                blit=False,   # Disable blitting for 3D plots                
                cache_frame_data=False,                
                repeat=False
            )            
            print("\nVisualization started!")            
            print("Controls:")            
            print("  'f' - Toggle force feedback test")
            print("  'r' - Reset 3D view")
            print("  'q' - Quit")
            print("Move the haptic device to see the visualization update...")

            # Set window title
            self.fig.canvas.manager.set_window_title("Inverse3 WebSocket Visualization")
            
            plt.show()

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Test Inverse3 WebSocket interface with visualization")
    parser.add_argument("--uri", default="ws://localhost:10001", help="WebSocket URI (default: ws://localhost:10001)")
    parser.add_argument(
        "--max-points", type=int, default=100, help="Maximum trajectory points to display (default: 100)"
    )

    args = parser.parse_args()

    print("Inverse3 WebSocket Test & Visualization")
    print("=" * 40)
    print(f"URI: {args.uri}")
    print(f"Max trajectory points: {args.max_points}")
    print()

    # Create and run visualizer
    visualizer = Inverse3Visualizer(uri=args.uri, max_points=args.max_points)

    try:
        visualizer.run_visualization()
    except Exception as e:
        print(f"Error running visualization: {e}")
        visualizer.stop()


if __name__ == "__main__":
    main()
