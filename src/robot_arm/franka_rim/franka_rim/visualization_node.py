import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import ColorRGBA
import numpy as np


class VisualizationNode(Node):
    """Node for visualizing I3 and RIM positions in RViz"""

    def __init__(self):
        super().__init__("visualization_node")
        self.get_logger().info("Initializing VisualizationNode")

        # Parameters
        self.declare_parameter("i3_sphere_size", 0.05)
        self.declare_parameter("rim_sphere_size", 0.05)

        self.i3_sphere_size = self.get_parameter("i3_sphere_size").get_parameter_value().double_value
        self.rim_sphere_size = self.get_parameter("rim_sphere_size").get_parameter_value().double_value

        # State storage
        self._i3_position = None  # Postion of the I3 in it's own frame
        self._i3_position_robot = None  # Position of the I3 in the robot's workspace

        self._rim_position = None
        self._goal_position = None

        # Subscribers
        self._i3_pose_sub = self.create_subscription(PoseStamped, "/i3/pose", self._i3_pose_callback, 10)
        self._haptic_pose_sub = self.create_subscription(PoseStamped, "/haptic_pose", self._haptic_pose_callback, 10)

        self._rim_pose_sub = self.create_subscription(PoseStamped, "/rim/pose", self._rim_pose_callback, 10)
        # self._rim_twist_sub = self.create_subscription(TwistStamped, "/rim/twist", self._rim_twist_callback, 10)

        # Publishers
        self._marker_pub = self.create_publisher(MarkerArray, "/delayrim_visualization", 10)

        # Timer for publishing markers
        self._vis_timer = self.create_timer(0.01, self._publish_markers)  # 100Hz

        self.get_logger().info(
            f"VisualizationNode started - I3 sphere: {self.i3_sphere_size}m, RIM sphere: {self.rim_sphere_size}m"
        )

    # ---- Callbacks ----
    def _i3_pose_callback(self, msg: PoseStamped):
        """Callback for I3 state updates"""
        # print("I3 STATE")
        # Extract I3 position (same coordinate transform as DelayRIM)
        self._i3_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # # Experiment offset
        # position[0] += 0.4253  # Same offset as DelayRIM

        # # Home offset
        # position[0] += 0.3067
        # position[2] += 0.4828

        # self._i3_position = position

    def _haptic_pose_callback(self, msg: PoseStamped):
        """Callback for haptic device pose in robot frame"""
        # Extract haptic position (same coordinate transform as DelayRIM)
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        self._i3_position_robot = position

    def _rim_pose_callback(self, msg: PoseStamped):
        """Callback for RIM state updates"""

        self._rim_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    # --- Methods
    def _publish_markers(self):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()

        # I3 position marker (red sphere)
        if self._i3_position_robot is not None:
            i3_marker = Marker()
            i3_marker.header.frame_id = "base"
            i3_marker.header.stamp = self.get_clock().now().to_msg()
            i3_marker.ns = "delayrim_viz"
            i3_marker.id = 0
            i3_marker.type = Marker.SPHERE
            i3_marker.action = Marker.ADD

            i3_marker.pose.position.x = float(self._i3_position_robot[0])
            i3_marker.pose.position.y = float(self._i3_position_robot[1])
            i3_marker.pose.position.z = float(self._i3_position_robot[2])

            # # I3 Marker at same y, z as RIM for visualization
            # i3_marker.pose.position.x = float(self._i3_position_robot[0])
            # i3_marker.pose.position.y = float(self._rim_position[1])
            # i3_marker.pose.position.z = float(self._rim_position[2])

            i3_marker.pose.orientation.w = 1.0

            i3_marker.scale.x = self.i3_sphere_size
            i3_marker.scale.y = self.i3_sphere_size
            i3_marker.scale.z = self.i3_sphere_size

            # Red color for I3
            i3_marker.color.r = 1.0
            i3_marker.color.g = 0.0
            i3_marker.color.b = 0.0
            i3_marker.color.a = 0.8
            marker_array.markers.append(i3_marker)

        # RIM position marker (green sphere)
        if self._rim_position is not None:
            rim_marker = Marker()
            rim_marker.header.frame_id = "base"
            rim_marker.header.stamp = self.get_clock().now().to_msg()
            rim_marker.ns = "delayrim_viz"
            rim_marker.id = 1
            rim_marker.type = Marker.SPHERE
            rim_marker.action = Marker.ADD

            rim_marker.pose.position.x = float(self._rim_position[0])
            rim_marker.pose.position.y = float(self._rim_position[1])
            rim_marker.pose.position.z = float(self._rim_position[2])
            rim_marker.pose.orientation.w = 1.0

            rim_marker.scale.x = self.rim_sphere_size
            rim_marker.scale.y = self.rim_sphere_size
            rim_marker.scale.z = self.rim_sphere_size

            # Green color for RIM
            rim_marker.color.r = 0.0
            rim_marker.color.g = 1.0
            rim_marker.color.b = 0.0
            rim_marker.color.a = 0.8

            marker_array.markers.append(rim_marker)

        # # Connection line between I3 and RIM (if both available)
        # if self._i3_position is not None and self._rim_position is not None:
        #     line_marker = Marker()
        #     line_marker.header.frame_id = "base"
        #     line_marker.header.stamp = self.get_clock().now().to_msg()
        #     line_marker.ns = "delayrim_viz"
        #     line_marker.id = 2
        #     line_marker.type = Marker.LINE_STRIP
        #     line_marker.action = Marker.ADD

        #     # Add points for the line
        #     from geometry_msgs.msg import Point

        #     p1 = Point()
        #     p1.x = float(self._i3_position[0])
        #     p1.y = float(self._i3_position[1])
        #     p1.z = float(self._i3_position[2])

        #     p2 = Point()
        #     p2.x = float(self._rim_position[0])
        #     p2.y = float(self._rim_position[1])
        #     p2.z = float(self._rim_position[2])

        #     line_marker.points = [p1, p2]

        #     line_marker.scale.x = 0.005  # Line width

        #     # Green color for connection line
        #     line_marker.color.r = 0.0
        #     line_marker.color.g = 1.0
        #     line_marker.color.b = 0.0
        #     line_marker.color.a = 0.6

        #     marker_array.markers.append(line_marker)

        # Publish markers
        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

    try:
        node.get_logger().info("VisualizationNode launched, end with CTRL-C")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt occurred, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
