import lgpio
import time
from rclpy.node import Node
from std_msgs.msg import String
import rclpy

# Define GPIO Pins
PUL_PIN = 21  # Pulse
DIR_PIN = 20  # Direction
ENA_PIN = 24  # Enable (optional)
CW_DIR = 0    # Logical LOW
CCW_DIR = 1   # Logical HIGH

class StepperMotorController(Node):
    def __init__(self):
        super().__init__('stepper_motor_controller')

        # Initialize GPIO
        self.gpio_chip = lgpio.gpiochip_open(4)  # Open GPIO chip 4
        lgpio.gpio_claim_output(self.gpio_chip, PUL_PIN)
        lgpio.gpio_claim_output(self.gpio_chip, DIR_PIN)
        # Uncomment for enabling motor if needed
        # lgpio.gpio_claim_output(self.gpio_chip, ENA_PIN)
        lgpio.gpio_write(self.gpio_chip, DIR_PIN, CW_DIR)

        # Subscribe to topic for movement commands
        self.subscription = self.create_subscription(
            String,
            '/stepper_motor_cmd',
            self.move_motor_callback,
            10
        )
        self.get_logger().info('Stepper motor controller node has been started.')

    def move_motor_callback(self, msg):
        direction, steps, pulse_width = msg.data.split(',')
        steps = int(steps)
        pulse_width = float(pulse_width)

        # Set direction
        lgpio.gpio_write(self.gpio_chip, DIR_PIN, CW_DIR if direction == 'CW' else CCW_DIR)

        # Generate pulses to move the motor to the target position
        for _ in range(steps):
            lgpio.gpio_write(self.gpio_chip, PUL_PIN, 1)  # HIGH
            time.sleep(pulse_width)
            lgpio.gpio_write(self.gpio_chip, PUL_PIN, 0)  # LOW
            time.sleep(pulse_width / 2)

    def destroy(self):
        lgpio.gpiochip_close(self.gpio_chip)  # Clean up GPIO
        self.get_logger().info('Stepper motor controller node has been stopped.')

def main(args=None):
    rclpy.init(args=args)
    stepper_motor_controller = StepperMotorController()

    try:
        rclpy.spin(stepper_motor_controller)
    except KeyboardInterrupt:
        pass  # Allows Ctrl+C to gracefully stop the node
    finally:
        stepper_motor_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
