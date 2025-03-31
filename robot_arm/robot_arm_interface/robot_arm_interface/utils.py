import rclpy

from geometry_msgs.msg import PoseStamped, Pose, Twist

import numpy as np
import pinocchio as pin


def array2pose(array: np.ndarray) -> Pose:
    pose = Pose()

    pose.position.x = array[0]
    pose.position.y = array[1]
    pose.position.z = array[2]

    # pose_stamped.pose.orientation.x = array[3]
    # pose_stamped.pose.orientation.y = array[4]
    # pose_stamped.pose.orientation.z = array[5]
    # pose_stamped.pose.orientation.w = array[6]

    return pose


def motion2rostwist(motion: pin.Motion) -> Twist:
    twist = Twist()

    lin = motion.linear
    ang = motion.angular

    twist.linear.x = lin[0]
    twist.linear.y = lin[1]
    twist.linear.z = lin[2]

    twist.angular.x = ang[0]
    twist.angular.y = ang[1]
    twist.angular.z = ang[2]

    return twist


def rostwist2motion(twist: Twist) -> pin.Motion:
    lin = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    ang = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

    motion = pin.Motion(lin, ang)

    return motion


def se32rospose(se3: pin.SE3) -> Pose:
    pose = Pose()

    pose.position.x = se3.translation[0]
    pose.position.y = se3.translation[1]
    pose.position.z = se3.translation[2]

    q = pin.Quaternion(se3.rotation).coeffs()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def rospose2se3(pose: Pose) -> pin.SE3:
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    R = pin.Quaternion(q).matrix()

    se3 = pin.SE3(R, t)

    return se3
