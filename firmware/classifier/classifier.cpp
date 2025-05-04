
#include "classifier.h"

bool classifier::begin(i2c_inst_t *i2c) {
  // Initialize the system - return results
  // Initialization code
  _i2cPort = i2cPort;
  _address = ICM20649_I2CADDR_DEFAULT;
  _i2cBus.init(_address, _i2cPort);

  // Reset the device
  reset();

  // 3 will be the largest range for either sensor
  writeGyroRange(3);
  writeAccelRange(3);
  // 1100Hz/(1+10) = 100Hz
  setGyroRateDivisor(10);

  // # 1125Hz/(1+20) = 53.57Hz
  setAccelRateDivisor(20);
  if (!ping())
    return false; // Sensor did not ack
  return true;
}

bool classifier::update() {}

void classifier::activate() {
  imu_counter = 0;
  bit_counter = 0;
  segment_counter = 0;
  return; // Implement activation logic if needed
}

void classifier::deactivate() {
  return; // Implement deactivation logic if needed
}
