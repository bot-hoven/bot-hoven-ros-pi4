import RPi.GPIO as GPIO
import time

# Setup GPIO for servo
servo_pin = 11
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servo_pin, GPIO.OUT)
# pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
# pwm.start(0)

# try:
#     while True:
#         # Move to 0 degrees
#         pwm.ChangeDutyCycle(2)  # Adjust duty cycle for 0 degrees
#         print(f"Potentiometer Value: 0 degrees")
#         time.sleep(1)
#         # Move to 90 degrees
#         pwm.ChangeDutyCycle(7)  # Adjust duty cycle for 90 degrees
#         print(f"Potentiometer Value: 90 degrees")
#         time.sleep(1)
#         # Move to 180 degrees
#         pwm.ChangeDutyCycle(12)  # Adjust duty cycle for 180 degrees
#         print(f"Potentiometer Value: 180 degrees")
#         time.sleep(1)
#         # Stop
#         pwm.ChangeDutyCycle(0)  # Turn off PWM
#         print(f"Potentiometer Value: Off")
#         time.sleep(1)

# except KeyboardInterrupt:
#     pass
# finally:
#     pwm.stop()
#     GPIO.cleanup()

GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
pwm.start(0)
print("Waiting for 2 seconds")
time.sleep(2)

duty = 2

print("Rotating 180 degrees")
while duty <= 12:
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    duty += 1

time.sleep(2)

print("Rotating 90 degrees")
pwm.ChangeDutyCycle(7)
time.sleep(2)

print("Rotating 0 degrees")
pwm.ChangeDutyCycle(2)
time.sleep(0.5)
pwm.ChangeDutyCycle(0)

pwm.stop()
GPIO.cleanup()


