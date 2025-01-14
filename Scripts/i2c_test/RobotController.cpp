#include "RobotController.h"

#include <sstream>
#include <stdexcept>
#include <algorithm>

// Constructor
RobotController::RobotController(const std::string &i2c_bus_path, uint8_t pca_address, uint8_t mcp_address)
    : i2c_bus(std::make_shared<I2CPeripheral>(i2c_bus_path)),
      mcp(),
      pca() {

    try {
        pca.setup(i2c_bus, pca_address); // Initialize PCA9685
    } catch (const std::exception &e) {
        throw std::runtime_error("Failed to setup PCA: " + std::string(e.what()));
    }

    try {
        mcp.setup(i2c_bus, mcp_address); // Initialize MCP23017
    } catch (const std::exception &e) {
        throw std::runtime_error("Failed to setup MCP: " + std::string(e.what()));
    }
}

// Initialize hardware components
void RobotController::initialize() {
    try {
        pca.connect();
        pca.init();
        pca.set_pwm_freq(50); // Set PCA9685 frequency to 50Hz

        mcp.connect();
        mcp.init();
    } catch (const std::exception &e) {
        throw std::runtime_error("Failed to initialize hardware: " + std::string(e.what()));
    }
}

// Process user commands
void RobotController::process_commands() {
    std::string input;
    std::cout << "Enter commands in the format '<device> <value_0> <value_1> <value_2> ...'\n";
    std::cout << "Examples:\n";
    std::cout << "  servo 1.5 1.5 1.5 1.5 1.5 (to control servos 0 to 4)\n";
    std::cout << "  sol 1 0 1 1 0             (to control solenoids 0 to 4)\n";

    while (true) {
        std::cout << "Enter command: ";
        std::getline(std::cin, input);

        if (input.empty()) {
            std::cerr << "Empty input, try again.\n";
            continue;
        }

        std::istringstream ss(input);
        std::string device;
        ss >> device; // Extract the device type (e.g., "servo" or "sol")

        try {
            if (device == "servo") {
                std::vector<double> values = parse_servo_input(input.substr(device.size()));
                if (values.size() != 5) {
                    throw std::invalid_argument("Expected 5 values for servos.");
                }
                control_servos(values);
            } else if (device == "sol") {
                std::vector<int> values = parse_solenoid_input(input.substr(device.size()));
                if (values.size() != 5) {
                    throw std::invalid_argument("Expected 5 values for solenoids.");
                }
                control_solenoids(values);
            } else {
                std::cerr << "Unknown device: " << device << ". Use 'servo' or 'sol'.\n";
            }
        } catch (const std::exception &e) {
            std::cerr << "Error processing command: " << e.what() << "\n";
        }
    }
}

// Parse servo input into a vector of doubles
std::vector<double> RobotController::parse_servo_input(const std::string &input) const {
    std::vector<double> values;
    std::stringstream ss(input);
    std::string token;

    while (ss >> token) {
        try {
            double value = std::stod(token);
            if (value < 0.0 || value > 1.0) {
                throw std::out_of_range("Servo comman must be between 0.0 and 1.0.");
            }
            values.push_back(value);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("Invalid servo input: " + token);
        }
    }

    return values;
}

// Parse solenoid input into a vector of integers
std::vector<int> RobotController::parse_solenoid_input(const std::string &input) const {
    std::vector<int> values;
    std::stringstream ss(input);
    std::string token;

    while (ss >> token) {
        try {
            int value = std::stoi(token);
            if (value != 0 && value != 1) {
                throw std::invalid_argument("Solenoid state must be 0 (OFF) or 1 (ON).");
            }
            values.push_back(value);
        } catch (const std::invalid_argument &) {
            throw std::runtime_error("Invalid solenoid input: " + token);
        }
    }

    return values;
}

double RobotController::command_to_duty_cycle(double command) {
    double min_input = 0.0;
    double max_input = 1.0;

    double clamped_command = std::clamp(command, min_input, max_input);

    // Duty cycle limits from 1ms to 2ms as per the datasheet
    double min_duty_cycle = 1.0;
    double max_duty_cycle = 2.0;

    double slope = (max_duty_cycle - min_duty_cycle) / (max_input - min_input);
    double offset = (max_duty_cycle + min_duty_cycle) / 2;

    return slope * clamped_command + offset;
}

// Control servos using the PCA9685
void RobotController::control_servos(const std::vector<double> &servo_values) {
    for (size_t i = 0; i < servo_values.size(); ++i) {
        double duty_cycle = command_to_duty_cycle(servo_values[i]);

        // if (!pca.connected()) {
            pca.connect();
        // }
        pca.set_pwm_ms(i, duty_cycle);
    }
}

// Control solenoids using the MCP23017
void RobotController::control_solenoids(const std::vector<int> &solenoid_values) {
    uint8_t gpio_state = 0;
    for (size_t i = 0; i < solenoid_values.size(); ++i) {
        gpio_state |= (solenoid_values[i] & 0x1) << i;
    }
    // if (!mcp.connected()) {
        mcp.connect();
    // }
    mcp.set_gpio_state(gpio_state);
}
