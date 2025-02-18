import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node

from geometry_msgs.msg import WrenchStamped, Vector3
from teleop_interfaces.msg import Inverse3State

# libraries needed for Inverse3
# from tracemalloc import start
import HaplyHardwareAPI
import serial
import serial.tools.list_ports
# import time
# import math

import numpy as np


#! Inverse3 helper functions
def close_to_point(point_position, device_pos, thres):
    """Helper function checks if the end effector is close to a point.

    Args:
        point_position (np.ndarray): Positon of the points.
        device_pos (np.ndarray): Current position of the device.
        thres (float): Radius of the dead zone.

    Returns:
        bool: True if the end effector is whithin `thres` to the center, False otherwise
    """
    # distance = np.sqrt(sum([(device_pos[i] - center[i])**2 for i in range(3)]))
    distance = np.linalg.norm(device_pos - point_position)

    return distance < thres


def force_restitution(center, radius, device_pos, stiffness):
    """
    Helper function to calculate the restitution force at the boundary of the pos-ctl and vel-ctl regions

    Args:
        center (np.ndarray): Center of the physical workspace.
        radius (float): Radius of the pos-ctl region.
        device_pos (np.ndarray): Current position of the device.
        stiffness (float): Stiffness of the restitution force.
    """
    force = np.zeros(3)
    # distance = math.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
    distance = np.linalg.norm(device_pos - center)

    if distance < radius:
        return force
    else:
        direction = (device_pos - center) / distance
        force = direction * (distance - radius) * -stiffness

        # direction = [(device_pos[i] - center[i]) / distance for i in range(3)]
        # force = [direction[i] * (distance - radius) * -stiffness for i in range(3)]
        return force


# helper function calculates the velocity to apply to the drone if the device is in the vel-ctrl region
def velocity_applied(center, radius, device_pos, Kd):
    distance = math.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
    direction = [(device_pos[i] - center[i]) / distance for i in range(3)]
    velocity = [direction[i] * ((distance - radius) ** 3) * Kd for i in range(3)]
    return velocity


# # helper function checks if the end effector is within the pos-ctrl region
# def pos_check(center, device_pos, r_pos):
#     distance = math.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
#     if distance < r_pos:
#         return True
#     else:
#         return False


def np2ros(array: np.ndarray) -> Vector3:
    """
    Converts a numpy array to a Vector 3 ROS message.

    Args:
        array (np.ndarray): Numpy array to convert.

    Returns:
        Vector3: ROS message.
    """
    if len(array) != 3:
        raise ValueError("Array must have 3 elements.")
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a numpy array.")

    msg = Vector3()
    msg.x = array[0]
    msg.y = array[1]
    msg.z = array[2]

    return msg


def ros2np(msg: Vector3) -> np.ndarray:
    """
    Converts a Vector 3 ROS message to a numpy array.

    Args:
        msg (Vector3): ROS message to convert.

    Returns:
        np.ndarray: Numpy array.
    """
    array = np.array([msg.x, msg.y, msg.z])

    return array


class Inverse3(Node):
    def __init__(self):
        super().__init__("inverse3_node")
        self.get_logger().info("Initializing inverse3_node...")

        #! Attributes
        self._i3: HaplyHardwareAPI.Inverse3 = None

        self._positions = np.zeros(3)
        self._raw_positions = np.zeros(3)
        self._velocities = np.zeros(3)
        self._forces = np.zeros(3)

        self._des_ee_position = np.zeros(3)
        self._contact_forces = np.zeros(3)  # contact forces, from the ROS topic

        # Parameters
        self._R = 0.05  # radius of the pos-ctl region
        self._ks = 50  # stiffness for the restitution force
        self._scale = 10  # scales the movement of the end effector
        self._velocity_ball_center = [0, 0, 0]  # the center of velocity_ball
        self._workspace_center = [0.04, -0.17, 0.16]  # center of the physical workspace
        self._maxVa = 0.0005  # maximum velocity in the vel-ctl region
        self._force_cap = 1

        #! Init functions
        self._init_i3()

        self._init_subscribers()
        self._init_publishers()

    def _init_i3(self):
        """
        Initialize the Inverse3 device.
        """
        #! Connect to the Inverse3 device
        ports = serial.tools.list_ports.comports()
        candidates = []
        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
            if "USB Serial" in desc:
                candidates.append(port)
        # ask the user for the port to use:
        if len(candidates) == 1:
            port = candidates[0]
        elif len(candidates) == 0:
            print("No USB Serial ports found.")
            # raise ValueError("No USB Serial ports found.")
        else:
            print("More than one USB Serial port found.")
            port = input("Enter the port to use: ")

        try:
            com = HaplyHardwareAPI.SerialStream(port)

        except ValueError:
            print("Error opening port: {}".format(ValueError))

        Inverse3 = HaplyHardwareAPI.Inverse3(com)
        Response = Inverse3.device_wakeup()
        print(Response)
        print("Made handshake with device")

        #! Move the device to the center of the pos-ctl region
        self.get_logger().info("Moving device to the center of the pos-ctl region...")
        # loop until the device end effector is close enough to the center of the pos-ctl region
        i3_at_center = close_to_point(self._workspace_center, self._raw_positions, 0.03)

        while not i3_at_center:
            self._raw_positions, self._velocities = Inverse3.end_effector_force(self._forces)
            self._forces = force_restitution(self._workspace_center, 0.01, self._raw_positions, 10)
            i3_at_center = close_to_point(self._workspace_center, self._raw_positions, 0.03)

            # rclpy.spin_once()  #TODO: Test w/ this

        self.get_logger().info("Inverse3 initialized!")

    def _init_subscribers(self):
        """
        To initialize the subscribers.
        """
        self.get_logger().info("Initializing subscribers...")

        # --- forces ---
        forces_topic = "/inverse3/wrench_des"
        self._forces_sub = self.create_subscription(WrenchStamped, forces_topic, self._wrench_topic_callback, 10)

    def _init_publishers(self):
        """
        To initialize the publishers.
        """
        self.get_logger().info("Initializing publishers...")

        # Inverse3 state publisher
        self._i3_state_topic = "/inverse3/state"
        self._i3_state_pub = self.create_publisher(Inverse3State, self._i3_state_topic, 10)
        self._i3_state_msg = Inverse3State()
        self._ee_cmd_timer = self.create_timer(0.01, self._pub_i3_state)

    #! Callbacks
    def _wrench_topic_callback(self, msg: ...):
        """
        Callback for the '/inverse3/wrench_des' topic.
        """
        self._contact_forces = ros2np(msg.wrench.force)
        self._contact_forces = np.clip(self._contact_forces, -self._force_cap, self._force_cap)

    def _pub_i3_state(self):
        """
        Publishes the state of the Inverse3.
        """
        #! Position
        self._raw_positions, self._velocities = Inverse3.end_effector_force(self._forces)

        # map of the workspace into the virtual space
        # positions = [self._scale * (self._raw_positions[i] - self._workspace_center[i]) for i in range(3)]
        positions = self._scale * (self._raw_positions - self._workspace_center)

        # position of the end effector relative to the velocity_ball
        # abs_positions = [positions[i] + self._velocity_ball_center[i] for i in range(3)]
        self._des_ee_position = positions + self._velocity_ball_center

        #! Forces
        # Update the forces for the next iteration

        # set the force in the next iteration on the end effector to be the published forces from Vortex
        # (capped since contact forces generated in Vortex can be very large)
        # restitution_forces = force_restitution(workspace_center, R, raw_positions, ks)
        restitution_forces = [0, 0, 0]
        # contact_forces = [
        #     max(min(force_cap, pub_sub.fx), -force_cap),
        #     max(min(force_cap, pub_sub.fy), -force_cap),
        #     max(min(force_cap, pub_sub.fz), -force_cap),
        # ]

        # total force on the end effector, applied next iteration
        # self._forces = [restitution_forces[i] + contact_forces[i] for i in range(3)]
        self._forces = restitution_forces + self._contact_forces

        in_pos_ctl_region = close_to_point(self._workspace_center, self._raw_positions, self._R)

        # vel-ctl region
        if not in_pos_ctl_region:
            # get the velocity of movement in vel-ctl region
            Va = velocity_applied(self._workspace_center, self._R, self._raw_positions, 100)

            # magVa = math.sqrt(Va[0] * Va[0] + Va[1] * Va[1] + Va[2] * Va[2])
            Va_norm = np.linalg.norm(Va)

            if Va_norm > self._maxVa:
                # Va = [(self._maxVa / Va_norm) * Va[0], (self._maxVa / Va_norm) * Va[1], (self._maxVa / Va_norm) * Va[2]]

                Va = self._maxVa / Va_norm * Va

            # self._velocity_ball_center = [(self._velocity_ball_center[i] + Va[i]) for i in range(3)]
            self._velocity_ball_center += Va

        #! Publish the Inverse3 state
        self._i3_state_msg.header.stamp = self.get_clock().now().to_msg()

        self._i3_state_msg.pose.position = np2ros(self._des_ee_position)
        self._i3_state_msg.wrench.force = np2ros(self._forces)

        self._i3_state_pub.publish(self._i3_state_msg)


def main(args=None):
    node_name = "inverse3_node"

    rclpy.init(args=args)
    node = Inverse3()

    try:
        node.get_logger().info(f"{node_name} launched, end with CTRL-C")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
