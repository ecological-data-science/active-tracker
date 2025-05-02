/*
  An RP2040 Library which allows you to communicate seamlessly with u-blox GNSS
  modules using the Configuration Interface

  Adapted from SparkFun u-blox GNSS Arduino Library v3.0
  Original: https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  Modified for Raspberry Pi Pico RP2040 SDK
*/

// sfe_bus.h

#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>
#include <cstdio>

namespace SparkFun_UBLOX_GNSS {

// The following abstract class is used an interface for upstream
// implementation.
class GNSSDeviceBus {
public:
  // For I2C, ping the _address
  // Not Applicable for SPI and Serial
  virtual bool ping() = 0;

  // For Serial, return available bytes
  // For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
  // Not Applicable for SPI
  virtual uint16_t available() = 0;

  // For Serial, write data
  // For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single
  // byte writes as these are illegal
  // For SPI, writing bytes will also read bytes simultaneously. Read data is
  // _ignored_ here.
  virtual uint8_t writeBytes(uint8_t *data, uint8_t length) = 0;

  // For SPI, writing bytes will also read bytes simultaneously. Read data is
  // returned in readData
  virtual uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData,
                                 uint8_t length) = 0;
  virtual void startWriteReadByte() = 0; // beginTransaction
  virtual void writeReadByte(const uint8_t *data,
                             uint8_t *readData) = 0; // transfer
  virtual void writeReadByte(const uint8_t data,
                             uint8_t *readData) = 0; // transfer
  virtual void endWriteReadByte() = 0;               // endTransaction

  // For Serial, attempt to read
  // For I2C, read from register 0xFF
  // For SPI, read the byte while writing 0xFF
  virtual uint8_t readBytes(uint8_t *data, uint8_t length) = 0;
};

// The SfeI2C device defines behavior for I2C implementation for RP2040
class SfeI2C : public GNSSDeviceBus {
public:
  SfeI2C(void);

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

}; // namespace SparkFun_UBLOX_GNSS
bool init(uint8_t address, bool bInit = false);
; // namespace SparkFun_UBLOX_GNSS
