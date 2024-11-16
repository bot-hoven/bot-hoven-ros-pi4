# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import RPi.GPIO as GPIO
# import time

# # Define GPIO Pins
# SERVO_PIN = 12

# class ServoMotorController(Node):
#     def __init__(self):
#         super().__init__('servo_motor_controller')
        
#         # Initialize GPIO
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(SERVO_PIN, GPIO.OUT)
#         self.pwm = GPIO.PWM(SERVO_PIN, 50)
#         self.pwm.start(0)
        
#         # Subscribe to topic for movement commands
#         self.subscription = self.create_subscription(
#             String,
#             '/servo_motor_cmd',
#             self.move_servo_callback,
#             10)
#         self.get_logger().info('Servo Motor controller node has been started.')
        
#     def move_servo_callback(self, msg):   
#         angle = int(msg.data)     
#         self.get_logger().info(f"Moving to angle: {angle}")
#         duty_cycle = (angle / 18) + 2
#         self.pwm.ChangeDutyCycle(duty_cycle)

#     def destroy(self):
#         self.pwm.stop()
#         GPIO.cleanup()  # Clean up GPIO pins
#         self.get_logger().info('Servo Motor controller node has been stopped.')
        
# def main(args=None):
#     rclpy.init(args=args)
#     servo_motor_controller = ServoMotorController()

#     try:
#         rclpy.spin(servo_motor_controller)
#     except KeyboardInterrupt:
#         pass  # Allows Ctrl+C to gracefully stop the node
#     finally:
#         servo_motor_controller.destroy()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# Define GPIO Pin for Servo PWM
SERVO_PIN = 12  # Choose the appropriate GPIO pin for PWM control
MIN_ANGLE = 0
MAX_ANGLE = 180

# PWM parameters for a typical servo
PWM_FREQUENCY = 50  # 50Hz is common for servos
MIN_DUTY_CYCLE = 2.5  # Adjust based on your servo's requirements
MAX_DUTY_CYCLE = 12.5

class ServoMotorController(Node):
    def __init__(self):
        super().__init__('servo_motor_controller')
        
        # Initialize GPIO and PWM
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY)
        self.pwm.start(0)  # Start PWM with 0 duty cycle (servo in neutral position)

        # Subscribe to topic for angle commands
        self.subscription = self.create_subscription(
            String,
            '/servo_motor_cmd',
            self.move_servo_callback,
            10)
        self.get_logger().info('Servo motor controller node has been started.')

    def move_servo_callback(self, msg):
        try:
            # Parse the angle from the message and convert it to an integer
            angle = int(msg.data)
            
            # Clamp the angle to the 0-180 range
            angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
            
            # Calculate duty cycle based on angle
            duty_cycle = self.angle_to_duty_cycle(angle)
            
            # Move the servo to the specified angle
            self.pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.5)  # Allow time for the servo to reach position
            self.pwm.ChangeDutyCycle(0)  # Stop sending signal to hold position
            
            self.get_logger().info(f'Moved servo to {angle} degrees.')
        
        except ValueError:
            self.get_logger().error('Invalid angle received; please provide an integer between 0 and 180.')

    def angle_to_duty_cycle(self, angle):
        # Linear interpolation between min and max duty cycle based on angle
        return MIN_DUTY_CYCLE + (angle / MAX_ANGLE) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)

    def destroy(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info('Servo motor controller node has been stopped.')
        
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
