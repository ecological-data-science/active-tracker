/*
  An RP2040 Library which allows you to communicate seamlessly with u-blox GNSS
  modules using the Configuration Interface

  Adapted from SparkFun u-blox GNSS Arduino Library v3.0
  Original: https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  Modified for Raspberry Pi Pico RP2040 SDK by Colin Torney 2025
*/


#include "i2c_bus.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
//

namespace i2c {

i2cbus::i2cbus(void) : _i2cPort{nullptr}, _address{0} {}

//////////////////////////////////////////////////////////////////////////////////////////////////
// I2C init()
//
// Methods to init/setup this device.
// The caller can provide an I2C Port, or this class will use the default.
bool i2cbus::init(uint8_t address, i2c_inst_t *i2cPort) {
  _i2cPort = i2cPort;
  _address = address;
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// I2C init()
//
// Methods to init/setup this device with default i2c port.

//////////////////////////////////////////////////////////////////////////////////////////////////
// ping()
//
// Is a device connected?
bool i2cbus::ping() {
  if (!_i2cPort)
    return false;

  // const int MAX_SIZE = 32; // Maximum data size to send
  // uint8_t data[MAX_SIZE];

  // Try to write 0 bytes to the device address
  // If it responds with an ACK, it exists
  int result = i2c_write_blocking(_i2cPort, _address, NULL, 0, false);
  return result >= 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// available()
//
// Checks how many bytes are waiting in the I2C buffer
// It does this by reading registers 0xFD and 0xFE
uint16_t i2cbus::available() {
  if (!_i2cPort)
    return false;

  // Get the number of bytes available from the module
  uint16_t bytesAvailable = 0;
  uint8_t reg = 0xFD; // 0xFD (MSB) and 0xFE (LSB) are the registers that
                      // contain number of bytes available

  // Write the register address
  int result = i2c_write_blocking(_i2cPort, _address, &reg, 1,
                                  true); // Keep control of the bus
  if (result < 0) {
    return 0; // Sensor did not ACK
  }

  // Read the two bytes
  uint8_t data[2];
  result = i2c_read_blocking(_i2cPort, _address, data, 2, false);
  if (result != 2) {
    return 0; // Sensor did not return 2 bytes
  } else {
    bytesAvailable = (uint16_t)data[0] << 8 | data[1];
  }

  return bytesAvailable;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// writeBytes()

uint8_t i2cbus::writeBytes(uint8_t *dataToWrite, uint8_t length) {
  if (!_i2cPort)
    return 0;

  if (length == 0)
    return 0;

  int result =
      i2c_write_blocking(_i2cPort, _address, dataToWrite, length, false);

  if (result >= 0)
    return result;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// readBytes()

uint8_t i2cbus::readBytes(uint8_t *data, uint8_t length) {
  if (!_i2cPort)
    return 0;

  if (length == 0)
    return 0;

  int result = i2c_read_blocking(_i2cPort, _address, data, length, false);
  if (result >= 0)
    return result;

  return 0;
}

} // namespace i2c
