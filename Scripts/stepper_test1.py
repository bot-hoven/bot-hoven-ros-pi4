import RPi.GPIO as GPIO
import time

# Define GPIO Pins
PUL_PIN = 21  # Pulse
DIR_PIN = 20  # Direction
ENA_PIN = 24  # Enable

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENA_PIN, GPIO.OUT)

# Enable the motor
GPIO.output(ENA_PIN, GPIO.HIGH)

# Set direction (CW)
GPIO.output(DIR_PIN, GPIO.HIGH)

# Generate pulses
for i in range(1600):  # 1600 pulses for one revolution (1.8-degree step angle)
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.001)  # Pulse width (adjust for motor speed)
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.001)  # Adjust for desired speed

# Disable the motor
GPIO.output(ENA_PIN, GPIO.LOW)

GPIO.cleanup()
