"""
To control a robotic arm using an xbox controller.
"""

import rclpy
from rclpy.node import Node

import serial
import serial.tools.list_ports


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        # self.publisher_ = self.create_publisher(Joystick, "joystick", 10)
        timer_period = 0.5

        # setup the Inverse3
        ports = serial.tools.list_ports.comports()
        candidates = []
        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
            if "USB Serial" in desc:
                candidates.append(port)

        # ask the user for the port to use:
        if len(candidates) == 1:
            port = candidates[0]
        else:
            port = input("Please enter the port to use: ")
        try:
            # com = HaplyHardwareAPI.SerialStream(port)
            ...

        except ValueError:
            print("Error opening port: {}".format(ValueError))

        ...


def main(args=None):
    # initialize the library
    rclpy.init(args=args)

    node = Joystick()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        ...

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
