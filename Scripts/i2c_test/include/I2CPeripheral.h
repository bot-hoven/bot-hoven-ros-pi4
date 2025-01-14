#ifndef I2C_PERIPHERAL_H
#define I2C_PERIPHERAL_H

#include <cstdint>
#include <string>

// namespace hardware {

class I2CPeripheral {
public:
  I2CPeripheral(const std::string& device);
  ~I2CPeripheral();

  void WriteRegisterByte(const uint8_t register_address, const uint8_t value);

  uint8_t ReadRegisterByte(const uint8_t register_address);

  void ConnectToPeripheral(const uint8_t address);

private:
  int bus_fd;

  void OpenBus(const std::string& device);

};

// }  // namespace hardware

#endif  // I2C_PERIPHERAL_H