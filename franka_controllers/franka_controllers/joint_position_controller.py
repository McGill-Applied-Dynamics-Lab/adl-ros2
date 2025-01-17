import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointPositionController(Node):
    def __init__(self):
        super().__init__("isaac_joint_state_remapper")

        joint_states_topic = "/joint_states"

        controller_name = "velocity_controller"
        controller_command_topic = f"/{controller_name}/command"

        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, 10
        )

        # TODO: Change command msg type
        self._commmand_pub = self.create_publisher(
            JointState, controller_command_topic, 10
        )

    def _joint_state_callback(self, msg):
        # TODO: Process joint_state to update the robot

        self._commmand_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
