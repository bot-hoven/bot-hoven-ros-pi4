import smbus
import time

# MCP23017 I2C Address (default is 0x20 [A0=A1=A2=GND])
MCP23017_ADDRESS = 0x20

# Register Addresses
IODIRA = 0x00   # Direction register for GPA (0 = output, 1 = input)
GPIOA = 0x12    # Register to control GPA pins
OLATA = 0x14    # Output latch register for GPA

# Setup I2C communication
bus = smbus.SMBus(1)  # Use I2C bus 1 on Pi5

def init_mcp23017():
    bus.write_byte_data(MCP23017_ADDRESS, IODIRA, 0x00)  # Set all GPA pins to outputs
    bus.write_byte_data(MCP23017_ADDRESS, OLATA, 0x00)   # Set all outputs to 0V initially
    print("MCP23017 Initialized. GPA pins set to outputs.")

def control_solenoids(solenoid_states):
    """
    Update the solenoid states.
    :param solenoid_states: List of 3 values [0, 1, 0] where 0 = OFF, 1 = ON.
    """
    # Ensure input is valid
    if len(solenoid_states) != 3 or any(state not in [0, 1] for state in solenoid_states):
        print("Error: Input must be 3 binary values (0 or 1).")
        return
    
    output_value = (
        (solenoid_states[0] << 0) |  # GPA0
        (solenoid_states[1] << 1) |  # GPA1
        (solenoid_states[2] << 2)    # GPA2
    )
    
    bus.write_byte_data(MCP23017_ADDRESS, GPIOA, output_value)
    print(f"Solenoid states updated: {solenoid_states}")

def main():
    init_mcp23017()
    
    print("Enter solenoid states as 3 binary values (0 or 1), separated by commas:")
    try:
        while True:
            user_input = input("Enter states (0,1,0): ")
            solenoid_states = [int(x.strip()) for x in user_input.split(",")]
            if len(solenoid_states) != 3:
                print("Please enter exactly 3 angles.")
                continue
            control_solenoids(solenoid_states)
            
    except KeyboardInterrupt:
        print("\nExiting program.")
        bus.write_byte_data(MCP23017_ADDRESS, GPIOA, 0x00)  # Turn off all solenoids
        print("All solenoids turned off.")

if __name__ == "__main__":
    main()
