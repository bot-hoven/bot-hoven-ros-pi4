import smbus
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# MCP23017 I2C Address (default is 0x20 [A0=A1=A2=GND])
MCP23017_ADDRESS = 0x20

# Register Addresses
IODIRA = 0x00   # Direction register for GPA (0 = output, 1 = input)
GPIOA = 0x12    # Register to control GPA pins
OLATA = 0x14    # Output latch register for GPA

class SolenoidController(Node):
    def __init__(self):
        super().__init__('solenoid_controller')
        self.bus = smbus.SMBus(1)  # Use I2C bus 1
        self.init_mcp23017()

        # Subscribe to the solenoid command topic
        self.subscription = self.create_subscription(
            String,
            '/solenoid_cmd',
            self.solenoid_cmd_callback,
            10
        )
        self.get_logger().info('Solenoid controller node has been started.')

    def init_mcp23017(self):
        """Initialize MCP23017 and set GPA pins as outputs."""
        self.bus.write_byte_data(MCP23017_ADDRESS, IODIRA, 0x00)  # Set all GPA pins to outputs
        self.bus.write_byte_data(MCP23017_ADDRESS, OLATA, 0x00)   # Set all outputs to 0V initially
        self.get_logger().info("MCP23017 Initialized. GPA pins set to outputs.")

    def solenoid_cmd_callback(self, msg):
        """Callback function to handle incoming solenoid commands."""
        try:
            # Parse the input data (expected format: "0,1,0")
            solenoid_states = [int(x.strip()) for x in msg.data.split(',')]

            # Validate the input
            if len(solenoid_states) != 3 or any(state not in [0, 1] for state in solenoid_states):
                self.get_logger().error("Invalid input. Provide exactly 3 binary values (0 or 1).")
                return

            # Convert the binary states to a single byte
            output_value = (
                (solenoid_states[0] << 0) |  # GPA0
                (solenoid_states[1] << 1) |  # GPA1
                (solenoid_states[2] << 2)    # GPA2
            )

            # Write the output to GPA pins
            self.bus.write_byte_data(MCP23017_ADDRESS, GPIOA, output_value)
            self.get_logger().info(f"Solenoid states updated: {solenoid_states}")
        except Exception as e:
            self.get_logger().error(f"Error processing solenoid command: {e}")

    def destroy(self):
        """Clean up the MCP23017 outputs on shutdown."""
        self.bus.write_byte_data(MCP23017_ADDRESS, GPIOA, 0x00)  # Turn off all solenoids
        self.get_logger().info("All solenoids turned off. Solenoid controller node stopped.")

def main(args=None):
    rclpy.init(args=args)
    solenoid_controller = SolenoidController()

    try:
        rclpy.spin(solenoid_controller)
    except KeyboardInterrupt:
        pass  # Graceful exit on Ctrl+C
    finally:
        solenoid_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
