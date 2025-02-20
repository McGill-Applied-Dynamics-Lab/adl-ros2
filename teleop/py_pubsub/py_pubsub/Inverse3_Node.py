# libraries needed for publishing/subscribing through ROS2
import rclpy
from rclpy.node import Node # node class makes a ROS2 Node
from std_msgs.msg import Float64 # type of message

# libraries needed for Inverse3
from tracemalloc import start
import HaplyHardwareAPI
import serial
import serial.tools.list_ports
import time
import math

# create a ROS2 node
class PubSub(Node):

    # Constructor for this class
    def __init__(self):
        super().__init__('Inverse3')

        # messages to publish : Float64-message type, 'topic'-positions/velocities, 10-how many previous messages to hold onto
        self.publisher_x = self.create_publisher(Float64, 'posx', 10)
        self.publisher_y = self.create_publisher(Float64, 'posy', 10)
        self.publisher_z = self.create_publisher(Float64, 'posz', 10)

        # messages to subscribe Float64-message type, 'topic'-force, 10-how many previous messages to hold onto
        self.subscriber_fx = self.create_subscription(Float64, 'fx', self.listener_callback_fx, 10)
        self.subscriber_fy = self.create_subscription(Float64, 'fy', self.listener_callback_fy, 10)
        self.subscriber_fz = self.create_subscription(Float64, 'fz', self.listener_callback_fz, 10)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # counter

        self.positions = [0,0,0]
        self.velocities = [0,0,0]
        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0

    # creates a message with the counter value appended
    def timer_callback(self):
        # Note the position change so the the z-axis is vertical
        posxMsg = Float64()
        posxMsg.data = self.positions[0]
        posyMsg = Float64()
        posyMsg.data = self.positions[1]
        poszMsg = Float64()
        poszMsg.data = self.positions[2]
        self.publisher_x.publish(posxMsg)
        self.publisher_y.publish(posyMsg)
        self.publisher_z.publish(poszMsg)
        self.i += 1

    # used to publish the position of the end effector
    def set_positions(self, positions):
        self.positions = positions

    # used to publish the velocity of the end effector
    def set_velocities(self, velocities):
        self.velocities = velocities

    # callback function for the force on the end effector in the x direction
    def listener_callback_fx(self, msg):
        self.fx = msg.data
        pass

    # callback function for the force on the end effector in the y direction
    def listener_callback_fy(self, msg):
        self.fy = msg.data
        pass

    # callback function for the force on the end effector in the z direction
    def listener_callback_fz(self, msg):
        self.fz = msg.data
        pass

# helper function to calculate the restitution force at the boundary of the pos-ctl and vel-ctl regions
def force_restitution(center, radius, device_pos, stiffness):
    distance = math.sqrt(sum([(device_pos[i] - center[i])**2 for i in range(3)]))
    if distance < radius:
        return [0,0,0]
    else:
        direction = [(device_pos[i] - center[i])/distance for i in range(3)]
        force = [direction[i]*(distance-radius) * -stiffness for i in range(3)]
        return force

# helper function calculates the velocity to apply to the drone if the device is in the vel-ctrl region
def velocity_applied(center, radius, device_pos, Kd):
    distance = math.sqrt(sum([(device_pos[i] - center[i])**2 for i in range(3)]))
    direction = [(device_pos[i] - center[i])/distance for i in range(3)]
    velocity = [direction[i]*((distance-radius)**3) * Kd for i in range(3)]
    return velocity

# helper function checks if the end effector is close to the workstation center
def start_check(center, device_pos, r_dead):
    distance = math.sqrt(sum([(device_pos[i] - center[i])**2 for i in range(3)]))
    if (distance > r_dead):
        return True
    else:
        return False

# helper function checks if the end effector is within the pos-ctrl region
def pos_check(center, device_pos, r_pos):
    distance = math.sqrt(sum([(device_pos[i] - center[i])**2 for i in range(3)]))
    if (distance < r_pos):
        return True
    else:
        return False

def main(args=None):
    # initialize the library
    rclpy.init(args=args)

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
        com = HaplyHardwareAPI.SerialStream(port)
    except ValueError:
        print("Error opening port: {}".format(ValueError))
    Inverse3 = HaplyHardwareAPI.Inverse3(com)
    Response = Inverse3.device_wakeup()
    print(Response)
    print("Made handshake with device")

    # initial values
    R = 0.05  # radius of the pos-ctl region
    ks = 50  # stiffness for the restitution force
    scale = 10  # scales the movement of the end effector
    velocity_ball_center = [0, 0, 0]  # the center of velocity_ball
    workspace_center = [0.04, -0.17, 0.16]  # center of the physical workspace
    maxVa = 0.0005  # maximum velocity in the vel-ctl region
    raw_positions = [0, 0, 0]
    velocities = [0, 0, 0]
    forces = [0, 0, 0]
    force_cap = 1

    # while loop which runs until the device end effector is close enough to the center of the pos-ctl region
    flag = start_check(workspace_center, raw_positions, 0.03)
    while flag:
        raw_positions, velocities = Inverse3.end_effector_force(forces)
        forces = force_restitution(workspace_center, 0.01, raw_positions, 10)
        flag = start_check(workspace_center, raw_positions, 0.03)

    # creates the node - make an example of the Inverse3_Node class
    pub_sub = PubSub()

    # simulation loop - finds the positions and velocities of the end effector from Inverse3, publishes them on ROS2
    # to be used by Vortex, and subscribes to the forces on the end effector generated in Vortex
    while True:
        raw_positions, velocities = Inverse3.end_effector_force(forces)
        positions = [scale * (raw_positions[i] - workspace_center[i]) for i in range(3)]  # map of the workspace into the virtual space
        abs_positions = [positions[i] + velocity_ball_center[i] for i in range(3)]  # position of the end effector relative to the velocity_ball

        pub_sub.set_positions(abs_positions)
        pub_sub.set_velocities(velocities)
        rclpy.spin_once(pub_sub)

        # set the force in the next iteration on the end effector to be the published forces from Vortex
        # (capped since contact forces generated in Vortex can be very large)
        # restitution_forces = force_restitution(workspace_center, R, raw_positions, ks)
        restitution_forces = [0,0,0]
        contact_forces = [max(min(force_cap,pub_sub.fx),-force_cap), max(min(force_cap,pub_sub.fy),-force_cap), max(min(force_cap,pub_sub.fz),-force_cap)]
        forces = [restitution_forces[i] + contact_forces[i] for i in range(3)]  # total force on the end effector, applied next iteration

        if pos_check(workspace_center, raw_positions, R) is False:  # end effector is in the vel-ctl region
            Va = velocity_applied(workspace_center, R, raw_positions, 100)  # get the velocity of movement in vel-ctl region
            magVa = math.sqrt(Va[0] * Va[0] + Va[1] * Va[1] + Va[2] * Va[2])
            if (magVa > maxVa):
                Va = [(maxVa / magVa) * Va[0], (maxVa / magVa) * Va[1], (maxVa / magVa) * Va[2]]
            velocity_ball_center = [(velocity_ball_center[i] + Va[i]) for i in range(3)]

    # destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    pub_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()