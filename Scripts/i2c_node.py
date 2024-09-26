import rclpy
from rclpy.node import Node
import smbus

class I2cNode(Node):
    def __init__(self):
        super().__init__('i2c_node')
        self.bus = smbus.SMBus(1)
        self.timer = self.create_timer(1.0, self.send_i2c_data)
        self.count = 0

    def send_i2c_data(self):
        self.count += 1
        self.bus.write_byte(0x08, self.count & 0xFF)
        self.get_logger().info(f'I2C Data Sent: {self.count & 0xFF}')

def main(args=None):
    rclpy.init(args=args)
    node = I2cNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
