#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "I2CPeripheral.h"
#include "pca9685_comm.h"
#include "mcp23017_comm.h"
#include <memory>
#include <string>
#include <vector>

class RobotController {
public:
    // Constructor
    RobotController(const std::string &i2c_bus_path, uint8_t pca_address, uint8_t mcp_address);

    // Public method to initialize components
    void initialize();

    // Public method to process user commands
    void process_commands();

private:
    // Private method to parse user input
    std::vector<double> parse_servo_input(const std::string &input) const;    

    std::vector<int> parse_solenoid_input(const std::string &input) const; 

    double command_to_duty_cycle(double command);

    // Private method to control servos
    void control_servos(const std::vector<double> &servo_values);

    // Private method to control solenoids
    void control_solenoids(const std::vector<int> &solenoid_values);

    // I2C Peripheral for communication
    std::shared_ptr<I2CPeripheral> i2c_bus;

    // PCA9685 and MCP23017 objects
    PCA9685 pca;
    MCP23017 mcp;
};

#endif // ROBOT_CONTROLLER_H
