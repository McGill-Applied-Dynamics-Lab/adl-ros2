"""Manual real-device validation for Inverse3 mapping and force rendering.

Run manually (not a pytest file):
  pixi run -e humble python src/arm_client/test/manual_inverse3_viser_validation.py
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import viser

from scipy.spatial.transform import Rotation as R

from arm_client.teleop.inverse3_teleop import Inverse3Config, Inverse3Device


def _set_frame_pose(handle, position: np.ndarray, wxyz: np.ndarray) -> None:
    """Set frame pose in a version-agnostic way."""
    handle.position = position
    handle.wxyz = wxyz


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual Inverse3 validation UI")
    parser.add_argument("--uri", default="ws://localhost:10001", help="Inverse3 websocket URI")
    parser.add_argument("--hz", type=float, default=120.0, help="UI update rate")
    parser.add_argument("--center-on-start", action="store_true", help="Center Inverse3 before running")
    parser.add_argument("--sine-force", action="store_true", help="Render sine force command", default=False)
    parser.add_argument("--sine-axis", choices=["x", "y", "z", "cycle"], default="cycle")
    parser.add_argument("--sine-amp", type=float, default=0.7, help="Sine force amplitude (N)")
    parser.add_argument("--sine-freq", type=float, default=0.5, help="Sine force frequency (Hz)")
    parser.add_argument("--axis-hold-seconds", type=float, default=4.0, help="For cycle mode: hold time per axis")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    config = Inverse3Config(
        uri=args.uri,
        i3_origin=[0.0, -0.17, 0.16],
        i3_origin_rpy=[0.0, 0.0, 90.0],
        i3_origin_rpy_degrees=True,
        center_on_start=False,
    )

    initial_robot_position = np.array([1.0, 0.0, 0.4])  # In robot base frame

    i3 = Inverse3Device(initial_robot_position=initial_robot_position, config=config)

    # --- Setup Viser
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2.0, height=2.0)

    i3_base_frame = server.scene.add_frame(
        "/i3_base_frame",
        axes_length=0.08,
        axes_radius=0.004,
        position=(0.0, 0.0, 0.0),
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    i3_origin_frame = server.scene.add_frame(
        "/i3_origin_frame",
        axes_length=0.08,
        axes_radius=0.004,
        position=(0.0, 0.0, 0.0),
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    robot_frame = server.scene.add_frame(
        "/robot_frame",
        axes_length=0.08,
        axes_radius=0.004,
        position=tuple(initial_robot_position),
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    position_i3_origin_frame = server.scene.add_frame(
        "/p_i3",
        axes_length=0.05,
        axes_radius=0.003,
        position=(0.0, 0.0, 0.0),
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    position_robot_frame = server.scene.add_frame(
        "/i3_position_in_robot_frame",
        axes_length=0.05,
        axes_radius=0.003,
        position=(0.0, 0.0, 0.0),
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    vector_base_to_p_base = server.scene.add_line_segments(
        "/vectors/base_to_p_base",
        points=np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]], dtype=np.float32),
        colors=np.array([255, 80, 80], dtype=np.uint8),
        line_width=2.0,
    )

    vector_origin_to_p_origin = server.scene.add_line_segments(
        "/vectors/origin_to_p_origin",
        points=np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]], dtype=np.float32),
        colors=np.array([80, 200, 120], dtype=np.uint8),
        line_width=2.0,
    )

    label_offset = np.array([0.0, 0.0, -0.03])
    i3_base_frame_label = server.scene.add_label(
        "/labels/i3_base_frame",
        text="i3_base",
        position=label_offset,
        anchor="bottom-left",
        depth_test=True,
        font_size_mode="scene",
        font_scene_height=0.01,
    )
    i3_origin_frame_label = server.scene.add_label(
        "/labels/i3_origin_frame",
        text="i3_origin",
        position=label_offset,
        anchor="bottom-left",
        depth_test=True,
        font_size_mode="scene",
        font_scene_height=0.01,
    )
    robot_frame_label = server.scene.add_label(
        "/labels/robot_frame",
        text="robot_frame",
        position=(initial_robot_position + label_offset),
        anchor="bottom-left",
        depth_test=True,
        font_size_mode="scene",
        font_scene_height=0.02,
    )

    force_display = server.gui.add_text("Current force cmd", "[0.0, 0.0, 0.0]")
    pos_i3_base_display = server.gui.add_text("In Base", "\t[0.0, 0.0, 0.0]")
    pos_i3_origin_display = server.gui.add_text("In Origin", "\t[0.0, 0.0, 0.0]")
    pos_robot_display = server.gui.add_text("pos (robot frame)", "\t[0.0, 0.0, 0.0]")

    # Frame poses
    i3_base_frame_position = np.zeros(3)
    i3_origin_frame_position = i3.origin_transform[:3, 3]
    i3_origin_frame_orientation = i3.origin_transform[:3, :3]
    robot_orientation = R.from_quat(i3.config.orientation_default, scalar_first=True).as_quat(scalar_first=True)

    robot_origin_frame_position = i3.robot_transform[:3, 3]
    robot_origin_frame_orientation = i3.robot_transform[:3, :3]

    _set_frame_pose(i3_base_frame, i3_base_frame_position, np.array([1.0, 0.0, 0.0, 0.0]))
    _set_frame_pose(
        i3_origin_frame, i3_origin_frame_position, R.from_matrix(i3_origin_frame_orientation).as_quat(scalar_first=True)
    )
    _set_frame_pose(
        robot_frame,
        robot_origin_frame_position,
        R.from_matrix(robot_origin_frame_orientation).as_quat(scalar_first=True),
    )

    i3_base_frame_label.position = label_offset
    i3_origin_frame_label.position = i3_origin_frame_position + label_offset
    robot_frame_label.position = robot_origin_frame_position + label_offset

    # --- Start
    i3.start()
    print("Connected. Open Viser UI in browser.")

    dt = 1.0 / max(args.hz, 1.0)
    t0 = time.time()
    try:
        while True:
            # Get I3 state
            i3.update_device_state()
            p_base = i3.position_base
            p_origin = i3.position_origin
            p_robot = i3.position_robot

            # p_origin is expressed in the i3_origin frame; convert to base/world for drawing.
            p_origin_in_base = i3_origin_frame_orientation @ p_origin

            vector_base_to_p_base.points = np.array([[i3_base_frame_position, p_base]], dtype=np.float32)
            vector_origin_to_p_origin.points = np.array([[i3_origin_frame_position, p_base]], dtype=np.float32)

            print(f"{p_base}\t\t{p_origin}")
            # Update vis
            _set_frame_pose(position_i3_origin_frame, p_base, robot_orientation)
            _set_frame_pose(position_robot_frame, p_robot, robot_orientation)

            # i3_pos_i3_label.position = i3_pos + label_offset
            # i3_pos_robot_label.position = (robot_origin[0] + i3_pos_robot) + label_offset

            pos_i3_base_display.value = np.array2string(p_base, precision=3)
            pos_i3_origin_display.value = np.array2string(p_origin, precision=3)
            pos_robot_display.value = np.array2string(p_robot, precision=3)

            # --- Apply force
            # force_cmd = np.zeros(3)
            # if args.sine_force:
            #     if args.sine_axis == "cycle":
            #         phase_idx = int(math.floor(t / args.axis_hold_seconds)) % 3
            #         axis = phase_idx
            #     else:
            #         axis = {"x": 0, "y": 1, "z": 2}[args.sine_axis]

            #     force_cmd[axis] = args.sine_amp * math.sin(2.0 * math.pi * args.sine_freq * t)
            #     i3.set_device_force(force_cmd)
            # else:
            #     i3.set_device_force(np.zeros(3))

            # force_display.value = np.array2string(force_cmd, precision=3)

            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    finally:
        i3.apply_force(np.zeros(3))
        i3.stop()


if __name__ == "__main__":
    main()
