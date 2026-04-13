"""Optional trajectory visualization helpers using viser."""

import time

import numpy as np

from arm_client.planning.ik_pyroki import load_fr3_urdf
from arm_client.planning.types import PlannedJointTrajectory


def visualize_planned_joint_trajectory(
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

    # Lazy imports to keep visualization optional.
    import viser
    from viser.extras import ViserUrdf

    urdf = load_fr3_urdf()

    print("Opening viser trajectory viewer at http://localhost:8080")
    print("Use Approve/Reject in GUI to continue.")

    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    status_label = server.gui.add_text("Status", "Initializing...")
    timesteps = len(trajectory.joint_positions)
    slider = server.gui.add_slider("Timestep", min=0, max=timesteps - 1, step=1, initial_value=0)
    playing = server.gui.add_checkbox("Playing", initial_value=True)
    respect_timing_checkbox = server.gui.add_checkbox("Respect time_from_start", initial_value=respect_timing)
    speed_slider = server.gui.add_slider("Playback speed", min=0.1, max=3.0, step=0.1, initial_value=1.0)
    approve_btn = server.gui.add_button("Approve")
    reject_btn = server.gui.add_button("Reject")

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

    # FR3-specific: planner outputs arm joints only; viewer URDF includes a hand joint.
    # Append fixed finger opening for visualization when needed.
    q = np.asarray(trajectory.joint_positions)
    times = np.asarray(trajectory.time_from_start, dtype=float)
    if np.any(np.diff(times) < -1e-9):
        raise ValueError("Trajectory times must be monotonically nondecreasing")
    total_duration = max(float(times[-1]), 1e-9)

    if q.shape[1] == len(trajectory.joint_names):
        q_vis = np.hstack([q, 0.04 * np.ones((q.shape[0], 1))])
    else:
        q_vis = q

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
                # Keep timing anchor aligned with manual slider changes while paused.
                play_start_elapsed = float(times[slider.value])
                play_start_wall = time.perf_counter()

            urdf_vis.update_cfg(q_vis[slider.value])
            status_label.value = (
                f"Waypoint {slider.value + 1}/{timesteps} | t={times[slider.value]:.2f}s / {total_duration:.2f}s"  # noqa: B023
            )

            time.sleep(max(1.0 / playback_hz, 0.001))
    except KeyboardInterrupt:
        approved = False

    return approved
