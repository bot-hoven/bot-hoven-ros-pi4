import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz 

# Initialize 3 servos on channels 0, 1, and 2
servo1 = servo.Servo(pca.channels[0], min_pulse=1000, max_pulse=2000)
servo2 = servo.Servo(pca.channels[1], min_pulse=1000, max_pulse=2000)
servo3 = servo.Servo(pca.channels[2], min_pulse=1000, max_pulse=2000)

def set_servo_angles(angle1, angle2, angle3):
    try:
        servo1.angle = angle1  
        servo2.angle = angle2
        servo3.angle = angle3
        print(f"Servo1: {angle1}, Servo2: {angle2}, Servo3: {angle3}")
    except ValueError as e:
        print(f"Error: {e}. Angles must be between 0 and 180.")

if __name__ == "__main__":
    try:
        while True:
            user_input = input("Enter 3 angles (0 to 180) separated by commas: ")
            angles = user_input.split(",")
            if len(angles) != 3:
                print("Please enter exactly 3 angles.")
                continue

            angle1, angle2, angle3 = [int(angle.strip()) for angle in angles]
            set_servo_angles(angle1, angle2, angle3)

    except KeyboardInterrupt:
        print("Exiting...")
        pca.deinit()  
