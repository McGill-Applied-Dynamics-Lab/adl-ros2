from typing import Literal
import rclpy
from rclpy.node import Node  # node class makes a ROS2 Node
# from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import WrenchStamped, Vector3, Point
from teleop_interfaces.msg import Inverse3State

# libraries needed for Inverse3
from inverse3_ros2.websocket_inverse3 import Inverse3
import numpy as np

from adg_ros2_utils.debug_utils import wait_for_debugger

NODE_NAME = "inverse3_node"

APPLY_CONTACT_FORCES = True
DEBUG = False

np.set_printoptions(precision=2)


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
    distance = np.linalg.norm(device_pos - center)

    if distance < radius:
        return force
    else:
        direction = (device_pos - center) / distance
        force = direction * (distance - radius) * -stiffness
        return force


def velocity_applied(center, radius, device_pos, Kd):
    # helper function calculates the velocity to apply to the drone if the device is in the vel-ctrl region
    distance = np.sqrt(sum([(device_pos[i] - center[i]) ** 2 for i in range(3)]))
    direction = [(device_pos[i] - center[i]) / distance for i in range(3)]
    velocity = [direction[i] * ((distance - radius) ** 3) * Kd for i in range(3)]
    return velocity


def np2ros(array: np.ndarray, msg_type: Literal["Vector3", "Point"] = "Vector3") -> Vector3 | Point:
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

    if msg_type == "Vector3":
        msg = Vector3()
        msg.x = float(array[0])
        msg.y = float(array[1])
        msg.z = float(array[2])
    elif msg_type == "Point":
        msg = Point()
        msg.x = array[0]
        msg.y = array[1]
        msg.z = array[2]
    else:
        raise ValueError("msg_type must be 'Vector3' or 'Point'.")

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


class Inverse3Node(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.get_logger().info("Initializing inverse3_node...")

        #! Declare parameters
        self.declare_parameter("pos_radius", 0.04)  # radius of the pos-ctl region
        self.declare_parameter("restitution_stiffness", 1.0)  # stiffness for restitution force
        self.declare_parameter("force_cap", 1.0)  # maximum force cap
        self.declare_parameter("scale", 1.0)  # scale between i3 position and published position
        self.declare_parameter("max_velocity", 0.0005)  # maximum velocity in vel-ctl region
        self.declare_parameter("websocket_uri", "ws://localhost:10001")  # WebSocket URI

        #! Attributes
        self._i3: Inverse3 = None
        self._is_initialized = False

        self._ws_position = np.zeros(3)  # position of the I3, in the ws
        self._raw_position = np.zeros(3)  # raw position from the Inverse3 device

        self._velocity = np.zeros(3)

        self._forces = np.zeros(3)
        self._contact_forces = np.zeros(3)  # contact forces, from the ROS topic

        # Parameters
        self._apply_contact_forces = APPLY_CONTACT_FORCES

        self._workspace_center = np.array([0.0, -0.17, 0.16])  # center of the ws
        self._pos_radius = self.get_parameter("pos_radius").get_parameter_value().double_value
        self._ks = self.get_parameter("restitution_stiffness").get_parameter_value().double_value

        self._scale = self.get_parameter("scale").get_parameter_value().double_value
        self._velocity_ball_center = np.array([0, 0, 0])  # the center of velocity_ball
        self._maxVa = self.get_parameter("max_velocity").get_parameter_value().double_value
        self._force_cap = self.get_parameter("force_cap").get_parameter_value().double_value
        self._websocket_uri = self.get_parameter("websocket_uri").get_parameter_value().string_value

        # Log parameter values
        self.get_logger().info(
            f"Parameters\n"
            f"- pos_radius: {self._pos_radius}, \n"
            f"- restitution_stiffness: {self._ks}, \n"
            f"- force_cap: {self._force_cap}, \n"
            f"- scale: {self._scale}, \n"
            f"- max_velocity: {self._maxVa}, \n"
            f"- websocket_uri: {self._websocket_uri}"
        )

        #! Init functions
        self._init_i3()

        self._init_subscribers()
        self._init_publishers()

        self._ee_cmd_timer = self.create_timer(0.01, self._pub_i3_state)

    def _init_i3(self):
        """
        Initialize the Inverse3 device via WebSocket.
        """
        self.get_logger().info("Connecting to Inverse3 device via WebSocket...")

        # Create and start the Inverse3 websocket connection
        self._i3 = Inverse3(uri=self._websocket_uri)

        if not self._i3.start():
            self.get_logger().error("Failed to connect to Inverse3 device!")
            return

        self.get_logger().info("Connected to Inverse3 device!")

        # Get device information
        device_info = self._i3.device_wakeup_dict()
        self.get_logger().info(f"Device ID: {device_info.get('device_id')}")
        self.get_logger().info(f"Handedness: {device_info.get('handedness')}")

        #! Move the device to the center of the pos-ctl region
        self.get_logger().info("Moving device to the center of the pos-ctl region...")

        # Get initial position
        self._raw_position, self._velocity = self._i3.get_state()

        # loop until the device end effector is close enough to the center of the pos-ctl region
        i3_at_center = close_to_point(self._workspace_center, self._raw_position, 0.03)

        while not i3_at_center:
            self._raw_position, self._velocity = self._i3.end_effector_force(self._forces.tolist())
            self._raw_position = np.array(self._raw_position)
            self._velocity = np.array(self._velocity)

            self._forces = force_restitution(self._workspace_center, 0.01, self._raw_position, 10)
            i3_at_center = close_to_point(self._workspace_center, self._raw_position, 0.03)

            # ? Log
            self._ws_position = self._scale * (self._raw_position - self._workspace_center)
            self.get_logger().info("Move device to the center of the pos-ctl region...", throttle_duration_sec=2.0)

        self._is_initialized = True
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
        i3_state_topic = "/inverse3/state"
        self._i3_state_pub = self.create_publisher(Inverse3State, i3_state_topic, 10)
        self._i3_state_msg = Inverse3State()

    #! Callbacks
    def _wrench_topic_callback(self, msg: WrenchStamped):
        """
        Callback for the '/inverse3/wrench_des' topic.
        """
        if not self._apply_contact_forces:
            self._contact_forces = np.zeros(3)
            return

        self._contact_forces = ros2np(msg.wrench.force)
        self._contact_forces = np.clip(self._contact_forces, -self._force_cap, self._force_cap)

    def _pub_i3_state(self):
        """
        Publishes the state of the Inverse3.
        """
        if not self._i3 or not self._i3.is_connected():
            return

        #! Position
        self._raw_position, self._velocity = self._i3.end_effector_force(self._forces.tolist())
        self._raw_position = np.array(self._raw_position)
        self._velocity = np.array(self._velocity)

        # print(f"Raw position: {self._raw_position}")

        # map of the workspace into the virtual space
        self._ws_position = self._scale * (self._raw_position - self._workspace_center)

        # TODO: Remove - Only y used
        self._ws_position[0] = 0.0
        self._ws_position[2] = 0.0

        #! Forces
        # Update the forces for the next iteration

        # set the force in the next iteration on the end effector to be the published forces from Vortex
        # (capped since contact forces generated in Vortex can be very large)
        restitution_forces = force_restitution(self._workspace_center, self._pos_radius, self._raw_position, self._ks)

        # total force on the end effector, applied next iteration
        self._forces = restitution_forces + self._contact_forces

        #! Publish the Inverse3 state
        self._i3_state_msg.header.stamp = self.get_clock().now().to_msg()

        self._i3_state_msg.pose.position = np2ros(self._ws_position, msg_type="Point")
        self._i3_state_msg.twist.linear = np2ros(self._velocity, msg_type="Vector3")

        self._i3_state_pub.publish(self._i3_state_msg)
        if DEBUG:
            self._log_infos()

    def _log_infos(self):
        """
        Log the information of the Inverse3.
        """
        self.get_logger().info(
            f"Init: {self._is_initialized}  Position: [{self._ws_position[0]:<5.4f}, {self._ws_position[1]:<5.4f}, {self._ws_position[2]:<5.4f}] | Forces: [{self._forces[0]:<4.2f}, {self._forces[1]:<4.2f}, {self._forces[2]:<4.2f}]"
        )

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if self._i3:
            self._i3.stop()
        super().destroy_node()

    def _log_infos(self):
        """
        Log the information of the Inverse3.
        """
        self.get_logger().info(
            f"Init: {self._is_initialized}  Position: [{self._ws_position[0]:<5.4f}, {self._ws_position[1]:<5.4f}, {self._ws_position[2]:<5.4f}] | Forces: [{self._forces[0]:<4.2f}, {self._forces[1]:<4.2f}, {self._forces[2]:<4.2f}]"
        )


def main(args=None):
    wait_for_debugger(NODE_NAME)  # Wait for debugger if env variables is set

    rclpy.init(args=args)
    node = Inverse3Node()

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    try:
        node.get_logger().info(f"{NODE_NAME} launched, end with CTRL-C")
        rclpy.spin(node)
        # executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.\n")

    finally:
        node.destroy_node()

    print("Shutting down...")


if __name__ == "__main__":
    main()
