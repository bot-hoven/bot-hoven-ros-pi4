import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class SimplePublisher(Node):
    def __init__(self, device_name):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.device_name = device_name
        self.count = 0
        timer_period = 1  # Frequency to publish
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        self.count += 1
        msg.data = f'Message {self.count} from {self.device_name}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'{self.device_name} sent: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: python publisher.py <device_name>")
        sys.exit(1)

    device_name = sys.argv[1]
    simple_publisher = SimplePublisher(device_name)

    rclpy.spin(simple_publisher)

    # Shutdown when done
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()