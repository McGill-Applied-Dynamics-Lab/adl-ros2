import rclpy
from rclpy.node import Node
import heapq
import threading
from rclpy.clock import Clock
from ament_index_python.packages import get_package_share_directory

import pandas as pd
from pathlib import Path
import importlib

from arm_interfaces.msg import Delayed

from rcl_interfaces.msg import ParameterDescriptor


class DelayedMessage:
    def __init__(self, msg, current_time: rclpy.time.Time, scheduled_time: rclpy.time.Time, latency_ns: float):
        self.msg = msg
        self.scheduled_time = scheduled_time

        if isinstance(msg, Delayed):
            self.msg.time_received_netsim = current_time.to_msg()
            self.msg.latency = rclpy.time.Duration(nanoseconds=latency_ns).to_msg()

    def __lt__(self, other):
        return self.scheduled_time < other.scheduled_time


class DelayNode(Node):
    def __init__(self):
        super().__init__("network_sim_node")
        self.get_logger().info("Network simulation node started")

        #! Declare parameters
        # Declare as string parameter but handle both string and potential integer inputs
        delay_param_desc = ParameterDescriptor(
            description="Delay parameter: '5g' for 5G model or integer for fixed delay in ms", dynamic_typing=True
        )
        self.declare_parameter(
            "delay", 100, delay_param_desc
        )  # Delay: either '5g' for 5G model or integer for fixed delay in ms

        # Get the parameter value and handle type conversion
        delay_param_value = self.get_parameter("delay").get_parameter_value()

        # Check if string is '5g'
        if delay_param_value.string_value.lower() == "5g":
            self.delay_type = "5g"
            self.delay = None  # Will be sampled from dataset
            self.network_delay = 0.5  # Delay in ms, of the network itself, not the latency
            self.get_logger().info("Using 5G delay model")

        else:
            try:
                self.delay = int(delay_param_value.string_value)
                self.delay_type = "fixed"
                self.get_logger().info(f"Using fixed delay: {self.delay}ms")

            except ValueError:
                self.delay = int(delay_param_value.integer_value)
                self.delay_type = "fixed"
                self.get_logger().info(f"Using fixed delay: {self.delay}ms")

        self.declare_parameter("message_type", "arm_interfaces/msg/Delayed")  # default message type
        self.message_type_str = self.get_parameter("message_type").get_parameter_value().string_value

        self.declare_parameter("input_topic", "input_topic")
        self._input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "output_topic")
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # Dynamic message type loading
        self.message_class = self._load_message_type(self.message_type_str)
        if self.message_class is None:
            self.get_logger().error(f"Failed to load message type: {self.message_type_str}")
            return

        #! Publishers and subscribers
        # Create publishers and subscribers with dynamic message type
        self.publisher = self.create_publisher(self.message_class, self._output_topic, 10)
        self.subscription = self.create_subscription(self.message_class, self._input_topic, self._input_callback, 10)

        #! Load 5G data
        # Load 5G data from CSV file only if using 5g delay type
        if self.delay_type == "5g":
            package_share_path = Path(get_package_share_directory("network_sim"))
            csv_file_path = package_share_path / "data/5G-data.csv"
            self.data_5g = pd.read_csv(csv_file_path)
            self.get_logger().info("5G dataset loaded")
        else:
            self.data_5g = None

        #! Attributes
        # Create message queue and lock
        self.msg_queue = []
        self.queue_lock = threading.Lock()

        # Create timer to check for delayed messages
        self.timer = self.create_timer(0.0001, self.timer_callback)  # check every 1ms

        self.get_logger().info(
            f"Delay Simulator started. message_type: {self.message_type_str}, delay_type: {self.delay_type}, delay: {self.delay}ms from {self._input_topic} to {self._output_topic}"
        )

    def _load_message_type(self, message_type_str: str):
        """Dynamically load message type from string like 'geometry_msgs/msg/PoseStamped'"""
        try:
            # Parse the message type string
            parts = message_type_str.split("/")
            if len(parts) != 3 or parts[1] != "msg":
                raise ValueError(f"Invalid message type format: {message_type_str}")

            package_name, _, message_name = parts

            # Import the message module
            module_name = f"{package_name}.msg"
            message_module = importlib.import_module(module_name)

            # Get the message class
            message_class = getattr(message_module, message_name)

            self.get_logger().info(f"Successfully loaded message type: {message_type_str}")
            return message_class

        except Exception as e:
            self.get_logger().error(f"Failed to load message type {message_type_str}: {e}")
            return None

    def _input_callback(self, msg):
        current_time = self.get_clock().now()

        if self.delay_type == "5g":
            latency_ms = self.data_5g.sample(n=1, replace=True).latency.values[
                0
            ]  # Randomly sample latency from the data
            latency_ns = int(latency_ms * 1e6)  # Convert to nanoseconds
        else:  # fixed delay
            latency_ns = int(self.delay * 1e6)  # Convert milliseconds to nanoseconds

        scheduled_time = current_time + rclpy.time.Duration(nanoseconds=latency_ns)

        with self.queue_lock:
            heapq.heappush(self.msg_queue, DelayedMessage(msg, current_time, scheduled_time, latency_ns))

    def timer_callback(self):
        current_time = self.get_clock().now()

        with self.queue_lock:
            while self.msg_queue:
                # Check if the earliest message is ready to publish
                if self.msg_queue[0].scheduled_time <= current_time:
                    delayed_msg = heapq.heappop(self.msg_queue)
                    msg = delayed_msg.msg

                    # Only set netsim timestamp for Delayed messages
                    if isinstance(msg, Delayed):
                        msg.time_sent_netsim = current_time.to_msg()

                    self.publisher.publish(msg)
                else:
                    break


def main(args=None):
    rclpy.init(args=args)
    node = DelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        if "node" in locals() and node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()

        except Exception as e:
            # This will catch the "rcl_shutdown already called" error
            print(f"Note: ROS context already shut down: {e}")
            # Just continue execution - error can be safely ignored
            pass


if __name__ == "__main__":
    main()
