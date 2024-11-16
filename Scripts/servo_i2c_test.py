import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Define GPIO Pins
SERVO_PIN = 11
PUSH = GPIO.HIGH
PULL = GPIO.LOW

class ServoMotorController(Node):
    def __init__(self):
        super().__init__('servo_motor_controller')
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, 50)
        self.pwm.start(0)
        
        # Subscribe to topic for movement commands
        self.subscription = self.create_subscription(
            String,
            '/servo_motor_cmd',
            self.move_servo_callback,
            10)
        self.get_logger().info('Servo Motor controller node has been started.')
        
    def move_servo_callback(self, msg):   
        angle = int(msg.data)     
        duty_cycle = (angle / 18) + 2
        self.pwm.ChangeDutyCycle(duty_cycle)

    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()  # Clean up GPIO pins
        self.get_logger().info('Servo Motor controller node has been stopped.')
        
def main(args=None):
    rclpy.init(args=args)
    servo_motor_controller = ServoMotorController()

    try:
        rclpy.spin(servo_motor_controller)
    except KeyboardInterrupt:
        pass  # Allows Ctrl+C to gracefully stop the node
    finally:
        servo_motor_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

