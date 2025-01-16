import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

"""
Class to remap the joint names from the controller manager to the ones used by the Isaac Sim robot.
"""


class IsaacJointStateRemapper(Node):
    def __init__(self):
        super().__init__("isaac_joint_state_remapper")

        # Command remapper
        command_input_topic = "/isaac_joint_commands"
        command_output_topic = "/isaac_joint_commands_remapped"

        self.command_subscription = self.create_subscription(
            JointState, command_input_topic, self.command_joint_state_callback, 10
        )

        self.commmand_publisher = self.create_publisher(
            JointState, command_output_topic, 10
        )

        # State remapper
        state_input_topic = "/isaac_joint_states"
        state_output_topic = "/isaac_joint_states_remapped"

        self.state_subscription = self.create_subscription(
            JointState, state_input_topic, self.state_joint_state_callback, 10
        )

        self.state_publisher = self.create_publisher(JointState, state_output_topic, 10)

    def command_joint_state_callback(self, msg):
        msg.name = [name.replace("fr3_joint", "panda_joint") for name in msg.name]

        self.commmand_publisher.publish(msg)

    def state_joint_state_callback(self, msg):
        msg.name = [name.replace("panda_joint", "fr3_joint") for name in msg.name]

        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacJointStateRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
