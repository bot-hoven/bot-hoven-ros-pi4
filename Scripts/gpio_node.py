# import rclpy
# from rclpy.node import Node
# import RPi.GPIO as GPIO
# # import libregpio as GPIO # for Sweet Potato

# class GpioNode(Node):
#     def __init__(self):
#         super().__init__('gpio_node')
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(11, GPIO.OUT)
#         # self.led_pin = 'GPIOX_17'
#         # self.led = GPIO.OUT(self.led_pin)
#         self.timer = self.create_timer(1.0, self.toggle_led)
#         self.count = 0

#     def toggle_led(self):
#         self.count += 1
#         GPIO.output(11, self.count % 2)
#         # self.led.toggle()
#         self.get_logger().info(f'GPIO LED Toggled. Count: {self.count}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GpioNode()
#     rclpy.spin(node)
#     GPIO.cleanup()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class GPIOToggleNode(Node):
    def __init__(self):
        super().__init__('gpio_toggle_node')

        # Set GPIO mode
        GPIO.setmode(GPIO.BOARD)

        # Define LED pins
        self.led_pins = {
            'g': 11,  # Green LED
            'y': 13,  # Yellow LED
            'r': 15   # Red LED
        }

        # Setup GPIOs as outputs
        for pin in self.led_pins.values():
            GPIO.setup(pin, GPIO.OUT)

        # Initialize LEDs as OFF
        self.led_states = {color: False for color in self.led_pins}

        # Subscriber for toggle commands
        self.subscription = self.create_subscription(
            String,
            'toggle_led',
            self.toggle_led_callback,
            10
        )

    def toggle_led_callback(self, msg):
        color = msg.data
        if color in self.led_pins:
            pin = self.led_pins[color]
            self.led_states[color] = not self.led_states[color]
            GPIO.output(pin, self.led_states[color])
            self.get_logger().info(f'{color.upper()} LED toggled {"ON" if self.led_states[color] else "OFF"}')

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOToggleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
