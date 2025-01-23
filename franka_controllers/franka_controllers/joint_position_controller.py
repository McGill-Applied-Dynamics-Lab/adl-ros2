import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointPositionController(Node):
    def __init__(self):
        super().__init__("joint_position_controller")

        joint_states_topic = "/joint_states"

        controller_name = "velocity_controller"
        controller_command_topic = f"/{controller_name}/commands"

        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, 10
        )

        self._commmand_pub = self.create_publisher(
            Float64MultiArray, controller_command_topic, 10
        )
        self._command_msg = Float64MultiArray()
        self._command_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _joint_state_callback(self, joint_msg):
        # TODO: Process joint_state to update the robot

        self._commmand_pub.publish(self._command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
