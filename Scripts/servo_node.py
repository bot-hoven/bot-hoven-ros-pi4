import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
# import libregpio as GPIO # for Sweet Potato

class GpioNode(Node):
    def __init__(self):
        super().__init__('gpio_node')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(11, GPIO.OUT)
        # self.led_pin = 'GPIOX_17'
        # self.led = GPIO.OUT(self.led_pin)
        self.timer = self.create_timer(1.0, self.toggle_led)
        self.count = 0

    def toggle_led(self):
        self.count += 1
        GPIO.output(11, self.count % 2)
        # self.led.toggle()
        self.get_logger().info(f'GPIO LED Toggled. Count: {self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = GpioNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
