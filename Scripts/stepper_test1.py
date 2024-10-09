import RPi.GPIO as GPIO
import time

# Set GPIO pins
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def step_motor(steps):
    for _ in range(steps):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(0.001)

        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(0.001)

        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        time.sleep(0.001)

        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        time.sleep(0.001)

try:
    while True:
        print(f"Stepper Test Cycle: Started")
        step_motor(2048)  # Number of steps for a full rotation (depends on the motor)
        print(f"Stepper Test Cycle: Finished")
        time.sleep(2)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
