
#include "IMU.h"
#include "hardware/i2c.h"

IMU::IMU() {
  // Constructor implementation
  temperature = 0;
  accX = accY = accZ = 0;
  gyroX = gyroY = gyroZ = 0;
  magX = magY = magZ = 0;
}

bool IMU::begin(i2c_inst_t *i2cPort, uint8_t i2c_addr) {
  // Initialization code
  
  _i2cPort = i2cPort;
  _address = ICM20649_I2CADDR_DEFAULT;
  _i2cBus.init(_address, _i2cPort);

  _sfeBus = &_i2cBus;
  // Reset the device
  reset();
  
  // TODO: Add actual initialization code for the IMU
  // - Check device ID
  // - Configure default settings
  // - Set up default ranges for accelerometer and gyroscope
  
  if (!ping())
    return false; // Sensor did not ack
  return true;
}

bool IMU::ping() {
  bool ok = _sfeBus->ping();
  return ok;
}
bool IMU::enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq) {
  // Enable/disable accelerometer digital low pass filter
  // TODO: Implement register writes to configure DLPF
  
  return true;
}

bool IMU::enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq) {
  // Enable/disable gyroscope digital low pass filter
  // TODO: Implement register writes to configure DLPF
  
  return true;
}

uint8_t IMU::getGyroRateDivisor(void) {
  // Get the gyroscope sample rate divisor
  // TODO: Read from appropriate register
  
  return 0;
}

void IMU::setGyroRateDivisor(uint8_t new_gyro_divisor) {
  // Set the gyroscope sample rate divisor
  // TODO: Write to appropriate register
}

uint16_t IMU::getAccelRateDivisor(void) {
  // Get the accelerometer sample rate divisor
  // TODO: Read from appropriate register
  
  return 0;
}

void IMU::setAccelRateDivisor(uint16_t new_accel_divisor) {
  // Set the accelerometer sample rate divisor
  // TODO: Write to appropriate register
}

void IMU::reset(void) {
  // Reset the device
  // TODO: Write to reset register
}

bool IMU::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
  // Read sensor data and populate the provided event structures
  
  // TODO: Read raw sensor data from registers
  
  // Scale the raw values
  scaleValues();
  
  // Fill in the event structures if provided
  if (accel) {
    accel->version = 1;
    accel->sensor_id = 0; // TODO: Set appropriate sensor ID
    accel->type = 1; // Accelerometer type
    accel->timestamp = 0; // TODO: Set timestamp
    accel->acceleration.x = accX;
    accel->acceleration.y = accY;
    accel->acceleration.z = accZ;
  }
  
  if (gyro) {
    gyro->version = 1;
    gyro->sensor_id = 1; // TODO: Set appropriate sensor ID
    gyro->type = 4; // Gyroscope type
    gyro->timestamp = 0; // TODO: Set timestamp
    gyro->gyro.x = gyroX;
    gyro->gyro.y = gyroY;
    gyro->gyro.z = gyroZ;
  }
  
  if (temp) {
    temp->version = 1;
    temp->sensor_id = 2; // TODO: Set appropriate sensor ID
    temp->type = 7; // Temperature type
    temp->timestamp = 0; // TODO: Set timestamp
    temp->temperature = temperature;
  }
  
  return true;
}

void IMU::scaleValues(void) {
  // Scale raw sensor values to appropriate units
  // TODO: Apply calibration and scaling factors to raw values
  // to convert to m/s^2 for accelerometer and rad/s for gyroscope
}


