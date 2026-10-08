"""Check: switching back to osc_controller after a joint-trajectory move does not replay a stale target.

Sequence: hold P0 on osc_controller -> switch to the JTC and move joint 1 by --dq1 (P1) with the JTC
client directly, which leaves Robot's Cartesian target at P0 -> switch back to osc_controller.
Every target_pose Robot publishes after the switch must be P1, and the arm must not move. Returns
to the start configuration at the end.

Usage:
    python switch_reseed.py [--dq1 0.12]
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from arm_client.robot import Robot
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_system_default

JTC = "joint_trajectory_controller"
OSC = "osc_controller"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dq1", type=float, default=0.12, help="Joint 1 offset for the JTC move (rad)")
    parser.add_argument("--move-time", type=float, default=3.0, help="Duration of each JTC move (s)")
    args = parser.parse_args()

    robot = Robot(namespace="fr3")
    robot.wait_until_ready()
    switcher, jtc = robot.controller_switcher_client, robot.joint_trajectory_controller_client

    sent = []  # (time, position) of every target_pose this Robot publishes
    robot.node.create_subscription(
        PoseStamped,
        robot.config.target_pose_topic,
        lambda m: sent.append((time.time(), np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]))),
        qos_profile_system_default,
    )

    switcher.switch_controller(OSC)
    p0 = robot.end_effector_pose.position.copy()
    q0 = robot.q.copy()
    time.sleep(1.0)

    # Joint move with the JTC client directly: nothing updates Robot's Cartesian target, it stays P0
    switcher.switch_controller(JTC)
    q1 = q0.copy()
    q1[0] += args.dq1
    jtc.send_joint_config(robot.config.joint_names, q1, args.move_time, blocking=True)
    time.sleep(0.5)
    p1 = robot.end_effector_pose.position.copy()
    print(f"P0 {np.round(p0, 4)} -> P1 {np.round(p1, 4)}: {np.linalg.norm(p1 - p0) * 1e3:.1f} mm apart")

    t_switch = time.time()
    switcher.switch_controller(OSC)
    t_done = time.time()
    time.sleep(1.5)
    p_after = robot.end_effector_pose.position.copy()

    after = [(t, p) for t, p in sent if t >= t_switch]
    dist_p1 = np.array([np.linalg.norm(p - p1) for _, p in after]) * 1e3
    dist_p0 = np.array([np.linalg.norm(p - p0) for _, p in after]) * 1e3
    moved = np.linalg.norm(p_after - p1) * 1e3
    print(f"switch took {(t_done - t_switch) * 1e3:.0f} ms; {len(after)} target_pose messages since the switch request")
    print(f"  distance to P1 (measured): max {dist_p1.max():.2f} mm   distance to P0 (stale): min {dist_p0.min():.1f} mm")
    print(f"  arm moved {moved:.2f} mm in the 1.5 s after the switch")
    ok = len(after) > 0 and dist_p1.max() < 2.0 and moved < 2.0
    print("PASS" if ok else "FAIL")

    # Back to the start configuration, holding it on osc_controller
    switcher.switch_controller(JTC)
    jtc.send_joint_config(robot.config.joint_names, q0, args.move_time, blocking=True)
    time.sleep(0.5)
    switcher.switch_controller(OSC)
    time.sleep(0.5)
    robot.shutdown()


if __name__ == "__main__":
    main()
