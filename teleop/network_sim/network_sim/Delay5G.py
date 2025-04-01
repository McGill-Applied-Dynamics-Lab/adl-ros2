import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq
import threading
from rclpy.clock import Clock


class DelayedMessage:
    def __init__(self, msg, scheduled_time):
        self.msg = msg
        self.scheduled_time = scheduled_time

    def __lt__(self, other):
        return self.scheduled_time < other.scheduled_time


class Delay5G(Node):
    def __init__(self):
        super().__init__("network_sim_5g")

        #! Declare parameters
        self.declare_parameter("delay", 0.1)  # default 100ms
        self.delay = self.get_parameter("delay").get_parameter_value().double_value

        self.declare_parameter("input_topic", "input_topic")
        self._input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "output_topic")
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        #! Publishers and subscribers
        # Create publishers and subscribers
        self.publisher = self.create_publisher(String, self._output_topic, 10)
        self.subscription = self.create_subscription(String, self._input_topic, self._input_callback, 10)

        #! Attributes
        # Create message queue and lock
        self.msg_queue = []
        self.queue_lock = threading.Lock()

        # Create timer to check for delayed messages
        self.timer = self.create_timer(0.001, self.timer_callback)  # check every 1ms

        self.get_logger().info(
            f"5G Delay Simulator started with {self.delay} second delay from {self._input_topic} to {self._output_topic}"
        )

    def _input_callback(self, msg):
        current_time = self.get_clock().now()
        scheduled_time = current_time + rclpy.time.Duration(seconds=self.delay)

        with self.queue_lock:
            heapq.heappush(self.msg_queue, DelayedMessage(msg, scheduled_time))

    def timer_callback(self):
        current_time = self.get_clock().now()

        with self.queue_lock:
            while self.msg_queue:
                # Check if the earliest message is ready to publish
                if self.msg_queue[0].scheduled_time <= current_time:
                    delayed_msg = heapq.heappop(self.msg_queue)
                    self.publisher.publish(delayed_msg.msg)
                else:
                    break


def main(args=None):
    rclpy.init(args=args)
    node = Delay5G()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
