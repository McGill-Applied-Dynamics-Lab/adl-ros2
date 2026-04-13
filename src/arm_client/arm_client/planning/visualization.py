"""Optional trajectory visualization helpers using viser."""

import time
from typing import Any

import numpy as np

import viser
from viser.extras import ViserUrdf

from arm_client.planning.ik_pyroki import load_fr3_urdf
from arm_client.planning.types import PlannedJointTrajectory

# Lazy-loaded globals to keep visualization optional
_viser_server = None
_viser_urdf_vis = None


class RobotVisualizer:
    """Stateful Viser server manager for visualization and teleoperation."""

    def __init__(self):
        global _viser_server, _viser_urdf_vis

        if _viser_server is None:
            _viser_server = viser.ViserServer()
            _viser_server.scene.add_grid("/ground", width=2, height=2)

            urdf = load_fr3_urdf()
            _viser_urdf_vis = ViserUrdf(_viser_server, urdf, root_node_name="/base")

        self.server = _viser_server
        self.urdf_vis = _viser_urdf_vis

    def update_robot_config(self, joint_positions: np.ndarray):
        """Immediately update the robot visual mesh."""
        # Ensure we pad with finger joints if missing
        q = np.asarray(joint_positions)
        if q.shape[-1] == 7:  # FR3 has 7 arm joints, hand expects more
            q_vis = np.hstack([q, 0.04 * np.ones((q.ndim > 1 and q.shape[0] or 1, 1))]).squeeze()
        else:
            q_vis = q

        self.urdf_vis.update_cfg(q_vis)

    def visualize_planned_joint_trajectory(
        self,
        trajectory: PlannedJointTrajectory,
        playback_hz: float = 10.0,
        respect_timing: bool = True,
    ) -> bool:
        """Visualize a planned joint trajectory and return user approval.

        Args:
            trajectory: Planned trajectory to preview.
            playback_hz: UI update rate for the viewer loop.
            respect_timing: If True, animate using `trajectory.time_from_start`.

        Returns:
            bool: True if approved, False if rejected.
        """
        if len(trajectory.joint_positions) == 0:
            raise ValueError("Trajectory is empty")
        if len(trajectory.time_from_start) != len(trajectory.joint_positions):
            raise ValueError("Trajectory time_from_start and joint_positions length mismatch")

        print("Opening viser trajectory viewer at http://localhost:8080")
        print("Use Approve/Reject in GUI to continue.")

        # Clean old GUI if present
        self.server.gui.clear()

        status_label = self.server.gui.add_text("Status", "Initializing...")
        timesteps = len(trajectory.joint_positions)
        slider = self.server.gui.add_slider("Timestep", min=0, max=timesteps - 1, step=1, initial_value=0)
        playing = self.server.gui.add_checkbox("Playing", initial_value=True)
        respect_timing_checkbox = self.server.gui.add_checkbox("Respect time_from_start", initial_value=respect_timing)
        speed_slider = self.server.gui.add_slider("Playback speed", min=0.1, max=3.0, step=0.1, initial_value=1.0)
        approve_btn = self.server.gui.add_button("Approve")
        reject_btn = self.server.gui.add_button("Reject")

        finished = False
        approved = False

        @approve_btn.on_click
        def _approve(_):
            nonlocal finished
            nonlocal approved
            approved = True
            finished = True

        @reject_btn.on_click
        def _reject(_):
            nonlocal finished
            nonlocal approved
            approved = False
            finished = True

        q = np.asarray(trajectory.joint_positions)
        times = np.asarray(trajectory.time_from_start, dtype=float)
        if np.any(np.diff(times) < -1e-9):
            raise ValueError("Trajectory times must be monotonically nondecreasing")
        total_duration = max(float(times[-1]), 1e-9)

        play_start_wall = time.perf_counter()
        play_start_elapsed = float(times[slider.value])

        try:
            while not finished:
                if playing.value:
                    if respect_timing_checkbox.value:
                        now = time.perf_counter()
                        elapsed = play_start_elapsed + (now - play_start_wall) * float(speed_slider.value)
                        elapsed_wrapped = elapsed % total_duration
                        idx = int(np.searchsorted(times, elapsed_wrapped, side="right") - 1)
                        slider.value = int(np.clip(idx, 0, timesteps - 1))
                    else:
                        slider.value = (slider.value + 1) % timesteps
                else:
                    play_start_elapsed = float(times[slider.value])
                    play_start_wall = time.perf_counter()

                self.update_robot_config(q[slider.value])
                status_label.value = (
                    f"Waypoint {slider.value + 1}/{timesteps} | t={times[slider.value]:.2f}s / {total_duration:.2f}s"
                )

                time.sleep(max(1.0 / playback_hz, 0.001))
        except KeyboardInterrupt:
            approved = False

        self.server.gui.clear()
        return approved


def visualize_planned_joint_trajectory(
    trajectory: PlannedJointTrajectory,
    playback_hz: float = 10.0,
    respect_timing: bool = True,
) -> bool:
    """Wrapper to maintain backwards compatibility while lazily instantiating RobotVisualizer."""
    visualizer = RobotVisualizer()
    return visualizer.visualize_planned_joint_trajectory(
        trajectory, playback_hz=playback_hz, respect_timing=respect_timing
    )
