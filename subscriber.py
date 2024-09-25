import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class SimpleSubscriber(Node):
    def __init__(self, device_name):
        super().__init__('simple_subscriber')
        self.device_name = device_name
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'{self.device_name} received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: python publisher.py <device_name>")
        sys.exit(1)

    device_name = sys.argv[1]
    simple_subscriber = SimpleSubscriber(device_name)

    rclpy.spin(simple_subscriber)

    # Shutdown when done
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()