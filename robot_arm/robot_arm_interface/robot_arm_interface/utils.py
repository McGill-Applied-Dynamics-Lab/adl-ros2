import rclpy

from geometry_msgs.msg import PoseStamped

import numpy as np
from spatialmath.pose3d import SO3, SE3
from spatialmath.base.quaternions import q2r, r2q
from spatialmath import UnitQuaternion


def SE32PoseStamped(se3: SE3) -> PoseStamped:
    pose_stamped = PoseStamped()

    # pose_stamped.header.frame_id = "base_link"
    pose_stamped.header.stamp = rclpy.time.Time().to_msg()

    pose_stamped.pose.position.x = se3.t[0]
    pose_stamped.pose.position.y = se3.t[1]
    pose_stamped.pose.position.z = se3.t[2]

    q = r2q(se3.R, order="sxyz")
    pose_stamped.pose.orientation.x = q[1]
    pose_stamped.pose.orientation.y = q[2]
    pose_stamped.pose.orientation.z = q[3]
    pose_stamped.pose.orientation.w = q[0]

    return pose_stamped


def PoseStamped2SE3(pose_stamped_msg: PoseStamped) -> SE3:
    """
    Convert a PoseStamped message to an SE3 object
    """
    t = np.array(
        [
            pose_stamped_msg.pose.position.x,
            pose_stamped_msg.pose.position.y,
            pose_stamped_msg.pose.position.z,
        ]
    )

    q = UnitQuaternion(
        [
            pose_stamped_msg.pose.orientation.w,
            pose_stamped_msg.pose.orientation.x,
            pose_stamped_msg.pose.orientation.y,
            pose_stamped_msg.pose.orientation.z,
        ]
    )
    T = SE3.Trans(t) * q.SE3()

    return T
