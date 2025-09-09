# ROS imports
from typing import Literal

from geometry_msgs.msg import (
    Vector3,
    Point,
    Quaternion,
)

# Other
import numpy as np


def np2ros(array: np.ndarray, msg_type: Literal["Vector3", "Point", "Quaternion"] = "Vector3") -> Vector3:
    """
    Converts a numpy array to a Vector 3 ROS message.

    Args:
        array (np.ndarray): Numpy array to convert.

    Returns:
        Vector3: ROS message.
    """
    # if len(array) != 3:
    #     raise ValueError("Array must have 3 elements.")
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a numpy array.")

    if msg_type == "Vector3":
        msg = Vector3()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
    elif msg_type == "Point":
        msg = Point()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
    elif msg_type == "Quaternion":
        msg = Quaternion()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
        msg.w = array[3]
    else:
        raise ValueError("Invalid message type.")

    return msg


def ros2np(msg: Vector3 | Point) -> np.ndarray:
    """
    Converts a Vector 3 ROS message to a numpy array.

    Args:
        msg (Vector3): ROS message to convert.

    Returns:
        np.ndarray: Numpy array.
    """
    array = np.array([msg.x, msg.y, msg.z])

    return array
