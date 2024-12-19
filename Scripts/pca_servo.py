import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Servo Angle Range
MIN_ANGLE = -90
MAX_ANGLE = 90

class ServoController(Node):
    """
    ROS2 Node to control 3 servos connected to a PCA9685 via I2C.
    The node subscribes to the /servo_cmd topic and expects input in the format:
    "angle1,angle2,angle3" where each angle is between -90 and 90.
    """

    def __init__(self):
        super().__init__('servo_controller')

        # Initialize I2C and PCA9685
        self.get_logger().info("Initializing PCA9685 and servos...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 50Hz for servo control

        # Initialize servos on channels 0, 1, and 2
        self.servo1 = servo.Servo(self.pca.channels[0], min_pulse=1000, max_pulse=2000)
        self.servo2 = servo.Servo(self.pca.channels[1], min_pulse=1000, max_pulse=2000)
        self.servo3 = servo.Servo(self.pca.channels[2], min_pulse=1000, max_pulse=2000)

        # ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,
            '/servo_cmd',
            self.servo_cmd_callback,
            10
        )
        self.get_logger().info("Servo controller node initialized. Waiting for commands...")

    def servo_cmd_callback(self, msg):
        """
        Callback function for processing servo angle commands.
        :param msg: ROS2 String message in the format "angle1,angle2,angle3"
        """
        try:
            # Parse angles
            angles = msg.data.split(',')
            if len(angles) != 3:
                self.get_logger().error("Invalid input. Please send exactly 3 angles separated by commas.")
                return

            # Convert to integers
            angle1, angle2, angle3 = [int(angle.strip()) for angle in angles]

            # Validate angles
            if not all(MIN_ANGLE <= angle <= MAX_ANGLE for angle in [angle1, angle2, angle3]):
                self.get_logger().error("Invalid angles. Angles must be between -90 and 90.")
                return

            # Set servo angles (center is at 90 degrees)
            self.servo1.angle = 90 + angle1
            self.servo2.angle = 90 + angle2
            self.servo3.angle = 90 + angle3

            self.get_logger().info(f"Servo angles updated: {angle1}°, {angle2}°, {angle3}°")

        except ValueError as e:
            self.get_logger().error(f"Error parsing angles: {e}")

    def destroy(self):
        """
        Cleanup method to deinitialize PCA9685 and shutdown safely.
        """
        self.get_logger().info("Shutting down servo controller...")
        self.pca.deinit()


def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()

    try:
        rclpy.spin(servo_controller)
    except KeyboardInterrupt:
        pass  # Gracefully exit on Ctrl+C
    finally:
        servo_controller.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
