#ifndef HARDWARE__PCA9685__PCA9685_COMM_H
#define HARDWARE__PCA9685__PCA9685_COMM_H

#include <string>
#include <memory>

#include "pca9685_constants.h"
#include "I2CPeripheral.h"

// namespace pca9685_hardware_interface {

class PCA9685 {
public:
        PCA9685();
        ~PCA9685();

        void setup(std::shared_ptr<I2CPeripheral> i2c_bus, const int i2c_address);

        void connect();

        void disconnect();

        bool connected() const;

        void init();

    void set_pwm_freq(const double freq_hz);

    void set_pwm(const int channel, const uint16_t on, const uint16_t off);

    void set_all_pwm(const uint16_t on, const uint16_t off);

    void set_pwm_ms(const int channel, const double ms);

private:
    private:
        bool is_connected = false;
        std::shared_ptr<I2CPeripheral> i2c_dev;
        double frequency = 200.0; // Default frequency pulled from PCA9685 datasheet.
        int address = 0x40;
    };

// }  // namespace pca9685_hardware_interface

#endif //HARDWARE__PCA9685__PCA9685_COMM_H