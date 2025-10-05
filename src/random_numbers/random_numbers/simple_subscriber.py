import rclpy
from rclpy.node import Node

from math import log, sqrt, cos, pi

from std_msgs.msg import Float32MultiArray


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

    def sub_callback(self, msg: Float32MultiArray):
        u1, u2 = msg.data

## end SimpleSubscriber

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber_node = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber_node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nExiting...")
        simple_subscriber_node.destroy_node()

if __name__ == "__main__":
    main()
