#include <iostream>
#include "RobotController.h"

int main() {
    try {
        // Create a RobotController instance
        RobotController controller("/dev/i2c-1", 0x40, 0x20);

        // Initialize hardware
        controller.initialize();

        // Process user commands
        controller.process_commands();
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
