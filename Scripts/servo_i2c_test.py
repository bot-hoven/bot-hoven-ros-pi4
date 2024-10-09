# import smbus
# import time
# import RPi.GPIO as GPIO

# # Setup GPIO for servo
# servo_pin = 11
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(servo_pin, GPIO.OUT)
# pwm = GPIO.PWM(servo_pin, 50)
# pwm.start(0)

# # I2C setup
# I2C_SLAVE_ADDR = 0x08  # Same address as defined in ESP32 code
# bus = smbus.SMBus(1)   # I2C bus number (1 for Raspberry Pi)

# # def set_servo_angle(angle):
# #     duty = (angle / 18) + 2
# #     GPIO.output(servo_pin, True)
# #     pwm.ChangeDutyCycle(duty)
# #     time.sleep(0.1)
# #     GPIO.output(servo_pin, False)
# #     pwm.ChangeDutyCycle(0)

# def set_servo_angle(angle):
#     # Convert angle to duty cycle
#     duty = (angle / 18) + 2  # Adjust this formula based on your specific servo
#     pwm.ChangeDutyCycle(duty)
#     # Instead of blocking, you can set the duty cycle and let the main loop handle timing.


# # try:
# #     while True:
# #         # Request data from ESP32
# #         pot_value = bus.read_byte(I2C_SLAVE_ADDR)
# #         # print(f"Potentiometer Value: {pot_value}")
        
# #         # Control servo angle based on potentiometer value
# #         set_servo_angle(pot_value)
# #         time.sleep(0.03)  # 10ms delay 

# try:
#     while True:
#         # Request data from ESP32
#         pot_value = bus.read_byte(I2C_SLAVE_ADDR)
        
#         # Scale the potentiometer value if necessary
#         scaled_value = map(pot_value, 0, 4095, 0, 180)  # Assuming you're scaling it
#         set_servo_angle(scaled_value)  # Set the servo position
#         time.sleep(0.03)  # Adjust if needed


# except KeyboardInterrupt:
#     pass

# finally:
#     pwm.stop()
#     GPIO.cleanup()


# import smbus
# import time
# import RPi.GPIO as GPIO
# import threading

# # Setup GPIO for servo
# servo_pin = 11
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(servo_pin, GPIO.OUT)
# pwm = GPIO.PWM(servo_pin, 50)
# pwm.start(0)

# # I2C setup
# I2C_SLAVE_ADDR = 0x08  # Same address as defined in ESP32 code
# bus = smbus.SMBus(1)   # I2C bus number (1 for Raspberry Pi)

# def set_servo_angle(angle):
#     duty = (angle / 18) + 2
#     GPIO.output(servo_pin, True)
#     pwm.ChangeDutyCycle(duty)
#     # GPIO.output(servo_pin, False)
#     # pwm.ChangeDutyCycle(0)

# def read_potentiometer():
#     while True:
#         # Request data from ESP32
#         pot_value = bus.read_byte(I2C_SLAVE_ADDR)
        
#         # Control servo angle based on potentiometer value
#         set_servo_angle(pot_value)
#         # print(pot_value)
#         time.sleep(0.01)  # Adjust this delay if necessary

# # Start the potentiometer reading in a separate thread
# pot_thread = threading.Thread(target=read_potentiometer)
# pot_thread.daemon = True  # This will allow the thread to exit when the main program does
# pot_thread.start()

# try:
#     while True:
#         # Main loop can perform other tasks or just wait
#         # time.sleep(0.1)  # Reduce CPU usage
#         # x = 1
#         pass
# except KeyboardInterrupt:
#     pass

# finally:
#     pwm.stop()
#     GPIO.cleanup()



# import smbus
# import time
# import RPi.GPIO as GPIO

# # Setup GPIO for servo
# servo_pin = 11
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(servo_pin, GPIO.OUT)
# pwm = GPIO.PWM(servo_pin, 50)
# pwm.start(0)

# # I2C setup
# I2C_SLAVE_ADDR = 0x08  # Same address as defined in ESP32 code
# bus = smbus.SMBus(1)   # I2C bus number (1 for Raspberry Pi)

# def set_servo_angle(angle):
#     duty = (angle / 18) + 2
#     GPIO.output(servo_pin, True)
#     pwm.ChangeDutyCycle(duty)
#     # GPIO.output(servo_pin, False)
#     # pwm.ChangeDutyCycle(0)

# # Timing variables
# previous_time = time.time()
# interval = 0.5  # 50 milliseconds

# try:
#     while True:
#         current_time = time.time()
#         if current_time - previous_time >= interval:
#             # Request data from ESP32
#             pot_value = bus.read_byte(I2C_SLAVE_ADDR)
            
#             # Control servo angle based on potentiometer value
#             set_servo_angle(pot_value)
            
#             previous_time = current_time  # Update the last time we checked

# except KeyboardInterrupt:
#     pass

# finally:
#     pwm.stop()
#     GPIO.cleanup()




import smbus
import time
import RPi.GPIO as GPIO

# Setup GPIO for servo
servo_pin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# I2C setup
I2C_SLAVE_ADDR = 0x08  # Same address as defined in ESP32 code
bus = smbus.SMBus(1)   # I2C bus number (1 for Raspberry Pi)

def set_servo_angle(angle):
    duty = (angle / 18) + 2
    # GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    # Optional: time.sleep(0.1) if you need a small delay for the servo to reach position
    # GPIO.output(servo_pin, False)
    # pwm.ChangeDutyCycle(0)

try:
    while True:
        # Request data from ESP32
        pot_value = bus.read_byte(I2C_SLAVE_ADDR)
        # print(f"Potentiometer Value: {pot_value}")
        
        # Control servo angle based on potentiometer value
        set_servo_angle(pot_value)
        time.sleep(0.1)  # Keep a slight delay for the loop

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
