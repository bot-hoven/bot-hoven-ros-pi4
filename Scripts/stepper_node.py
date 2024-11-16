import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Define GPIO Pins
PUL_PIN = 21  # Pulse
DIR_PIN = 20  # Direction
ENA_PIN = 24  # Enable (optional)
CW_DIR = GPIO.LOW
CCW_DIR = GPIO.HIGH

class StepperMotorController(Node):
    def __init__(self):
        super().__init__('stepper_motor_controller')
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUL_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        # GPIO.setup(ENA_PIN, GPIO.OUT)
        GPIO.output(DIR_PIN, CW_DIR)
        
        # Subscribe to topic for movement commands
        self.subscription = self.create_subscription(
            String,
            '/stepper_motor_cmd',
            self.move_motor_callback,
            10)
        self.get_logger().info('Stepper motor controller node has been started.')
        
    def move_motor_callback(self, msg):
        direction, steps, pulse_width = msg.data.split(',')
        steps = int(steps)
        pulse_width = float(pulse_width)
        
        # Set direction
        GPIO.output(DIR_PIN, CW_DIR if direction == 'CW' else CCW_DIR)
        
        # Enable motor
        # GPIO.output(ENA_PIN, GPIO.HIGH)
        
        # Generate pulses to move the motor to the target position
        for _ in range(steps):
            GPIO.output(PUL_PIN, GPIO.HIGH)
            time.sleep(pulse_width)  
            GPIO.output(PUL_PIN, GPIO.LOW)
            time.sleep(pulse_width/2)
        
        # Stop sending pulses after the desired steps are reached.
        # Motor will stay at the last position after the pulses are stopped.
        
        # Optionally disable motor (depends on your motor driver)
        # GPIO.output(ENA_PIN, GPIO.LOW)  # Uncomment if you want to disable after move
        # If your motor driver requires the motor to be continuously energized
        # to hold the position, you can leave it enabled.

    def destroy(self):
        GPIO.cleanup()  # Clean up GPIO pins
        self.get_logger().info('Stepper motor controller node has been stopped.')
        
def main(args=None):
    rclpy.init(args=args)
    stepper_motor_controller = StepperMotorController()
    # rclpy.spin(stepper_motor_controller)
    # stepper_motor_controller.destroy()
    # rclpy.shutdown()

    try:
        rclpy.spin(stepper_motor_controller)
    except KeyboardInterrupt:
        pass  # Allows Ctrl+C to gracefully stop the node
    finally:
        stepper_motor_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
