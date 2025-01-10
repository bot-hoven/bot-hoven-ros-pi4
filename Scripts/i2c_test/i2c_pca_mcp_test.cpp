#include "include/I2CPeripheral.h"
#include "include/pca9685_comm.h"
#include "include/mcp23017_comm.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

int main() {
    try {
        auto i2c_bus = std::make_shared<I2CPeripheral>("/dev/i2c-1");

        PCA9685 pca(i2c_bus, 0x40); // I2C address 0x40
        MCP23017 mcp(i2c_bus, 0x20); // I2C address 0x20

        pca.set_pwm_freq(50); // Set PCA9685 frequency to 50Hz
        mcp.init();           

        // User instructions
        std::cout << "Enter commands in the format x,y,z,a,b,c\n";
        std::cout << "x, y, z: Servo angles (0-180 degrees)\n";
        std::cout << "a, b, c: Solenoid states (0 = OFF, 1 = ON)\n";

        std::string input;
        while (true) {
            std::cout << "Enter command: ";
            std::getline(std::cin, input);

            if (input.empty()) {
                std::cout << "Empty input, try again.\n";
                continue;
            }

            // Parse input
            std::vector<int> values;
            std::stringstream ss(input);
            std::string token;
            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stoi(token));
                } catch (const std::invalid_argument &e) {
                    std::cerr << "Invalid input: " << token << "\n";
                    values.clear();
                    break;
                }
            }

            if (values.size() != 6) {
                std::cerr << "Invalid input. Enter 6 values in the format x,y,z,a,b,c.\n";
                continue;
            }

            // Control servos
            try {
                pca.set_pwm_ms(0, values[0]); // Servo 1 (Channel 0)
                pca.set_pwm_ms(1, values[1]); // Servo 2 (Channel 1)
                pca.set_pwm_ms(2, values[2]); // Servo 3 (Channel 2)
            } catch (const std::exception &e) {
                std::cerr << "Error controlling servos: " << e.what() << "\n";
            }

            // Control solenoids
            try {
                uint8_t solenoid_values = 0;
                for (size_t i = 0; i < 3; ++i) {
                    solenoid_values |= (values[i + 3] & 0x1) << i;
                }
                mcp.set_gpio_state(solenoid_values);
            } catch (const std::exception &e) {
                std::cerr << "Error controlling solenoids: " << e.what() << "\n";
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Initialization error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
