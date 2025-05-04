/*
  An RP2040 Library which allows you to communicate seamlessly with u-blox GNSS
  modules using the Configuration Interface

  Adapted from SparkFun u-blox GNSS Arduino Library v3.0
  Original: https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  Modified for Raspberry Pi Pico RP2040 SDK by Colin Torney 2025
*/

#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>
#include <cstdio>

namespace i2c {

class i2cbus {
public:
  i2cbus(void);

  bool init(uint8_t address, i2c_inst_t *i2cPort);

  bool ping();
  uint16_t available();
  uint8_t writeBytes(uint8_t *data, uint8_t length);

  uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData,
                         uint8_t length) {
    (void)data;
    (void)readData;
    (void)length;
    return 0;
  }

  void startWriteReadByte() {};
  void writeReadByte(const uint8_t *data, uint8_t *readData) {
    (void)data;
    (void)readData;
  }
  void writeReadByte(const uint8_t data, uint8_t *readData) {
    (void)data;
    (void)readData;
  }
  void endWriteReadByte() {};

  uint8_t readBytes(uint8_t *data, uint8_t length);

private:
  i2c_inst_t *_i2cPort;
  uint8_t _address;
};

}; // namespace i2c
