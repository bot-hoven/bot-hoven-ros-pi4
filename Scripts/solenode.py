import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Define GPIO Pins
SOL_PIN = 21
PUSH = GPIO.HIGH
PULL = GPIO.LOW

class SolenoidController(Node):
    def __init__(self):
        super().__init__('solenoid_controller')
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SOL_PIN, GPIO.OUT)
        GPIO.output(SOL_PIN, GPIO.LOW)
        
        # Subscribe to topic for movement commands
        self.subscription = self.create_subscription(
            String,
            '/solenoid_cmd',
            self.move_solenoid_callback,
            10)
        self.get_logger().info('Solenoid controller node has been started.')
        
    def move_solenoid_callback(self, msg):        
        # Set direction
        GPIO.output(SOL_PIN, PUSH if msg.data == 'press' else PULL)

    def destroy(self):
        GPIO.cleanup()  # Clean up GPIO pins
        self.get_logger().info('Solenoid controller node has been stopped.')
        
def main(args=None):
    rclpy.init(args=args)
    solenoid_controller = SolenoidController()

    try:
        rclpy.spin(solenoid_controller)
    except KeyboardInterrupt:
        pass  # Allows Ctrl+C to gracefully stop the node
    finally:
        solenoid_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
