
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('joint_vel_test_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self._joint_vels_cmd_msg = Float64MultiArray()
        self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_vels_cmd_period = 0.01  # Hz
        self._joint_vels_cmd_pub_timer = self.create_timer(self._joint_vels_cmd_period , self.timer_callback)

        self._start_time = self.get_clock().now()

    def timer_callback(self):
        time = self.get_clock().now()
        if time > Duration(seconds=2) + self._start_time:
            self._joint_vels_cmd_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        
        self.publisher_.publish(self._joint_vels_cmd_msg)
        self.get_logger().info('Publishing: "%s"' % self._joint_vels_cmd_msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()