import rclpy
from rclpy.node import Node
import spidev

class SpiNode(Node):
    def __init__(self):
        super().__init__('spi_node')
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 50000
        self.timer = self.create_timer(1.0, self.send_spi_data)
        self.count = 0

    def send_spi_data(self):
        self.count += 1
        data = [self.count & 0xFF]
        self.spi.xfer(data)
        self.get_logger().info(f'SPI Data Sent: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = SpiNode()
    rclpy.spin(node)
    node.spi.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
