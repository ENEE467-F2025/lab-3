import rclpy
from rclpy.node import Node

import random

from std_msgs.msg import Float32MultiArray


class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [random.random(), random.random()]

## end SimplePublisher

def main(args=None):
    rclpy.init(args=args)
    simple_publisher_node = SimplePublisher()

    try:
        rclpy.spin(simple_publisher_node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nExiting...")
        simple_publisher_node.destroy_node()

if __name__ == "__main__":
    main()
