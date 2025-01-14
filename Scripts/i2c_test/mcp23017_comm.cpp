#include "mcp23017_comm.h"

#include <unistd.h>

#include <cmath>

// namespace mcp23017_hardware_interface {

    MCP23017::MCP23017() = default;

    MCP23017::~MCP23017() = default;

    void MCP23017::setup(std::shared_ptr<I2CPeripheral> i2c_bus, const int i2c_address) {
        i2c_dev = i2c_bus;
        address = i2c_address;
    }

    void MCP23017::connect() {
        i2c_dev->ConnectToPeripheral(address);
        is_connected = true;
    }

    void MCP23017::disconnect() {
        is_connected = false;
    }

    bool MCP23017::connected() const {
        return is_connected;
    }

    void MCP23017::init() {
        i2c_dev->WriteRegisterByte(IODIRA, 0x00);  // Set pins to outputs
        i2c_dev->WriteRegisterByte(OLATA, 0x00);   // Initial values of 0
    }

    void MCP23017::set_gpio_state(uint8_t gpio_value_) {
        i2c_dev->ConnectToPeripheral(address);
        i2c_dev->WriteRegisterByte(GPIOA, gpio_value_);
    }

// }  // namespace mcp23017_hardware_interface