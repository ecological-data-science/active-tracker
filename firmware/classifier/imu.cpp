
#include "imu.h"
#include "hardware/i2c.h"

imu::imu() {
  // Constructor implementation
  temperature = 0;
  accX = accY = accZ = 0;
  gyroX = gyroY = gyroZ = 0;
  magX = magY = magZ = 0;
}



void imu::writeAccelRange(uint8_t new_accel_range) {
  _setBank(2);

  // Read current register value
  uint8_t reg_addr = ICM20X_B2_ACCEL_CONFIG_1;
  uint8_t reg_data[2] = {reg_addr, 0};
  
  _sfeBus->writeBytes(reg_data, 1);
  _sfeBus->readBytes(&reg_data[1], 1);
  
  // Modify only the bits we want (bits 1-2)
  reg_data[1] &= ~(0x03 << 1); // Clear bits 1-2
  reg_data[1] |= (new_accel_range & 0x03) << 1; // Set bits 1-2 to new_accel_range
  
  // Write back the modified value
  _sfeBus->writeBytes(reg_data, 2);
  
  current_accel_range = new_accel_range;
  _setBank(0);
}

void imu::writeGyroRange(uint8_t new_gyro_range) {
  _setBank(2);

  // Read current register value
  uint8_t reg_addr = ICM20X_B2_GYRO_CONFIG_1;
  uint8_t reg_data[2] = {reg_addr, 0};
  
  _sfeBus->writeBytes(reg_data, 1);
  _sfeBus->readBytes(&reg_data[1], 1);
  
  // Modify only the bits we want (bits 1-2)
  reg_data[1] &= ~(0x03 << 1); // Clear bits 1-2
  reg_data[1] |= (new_gyro_range & 0x03) << 1; // Set bits 1-2 to new_gyro_range
  
  // Write back the modified value
  _sfeBus->writeBytes(reg_data, 2);
  
  current_gyro_range = new_gyro_range;
  _setBank(0);
}

bool imu::ping() {
  bool ok = _sfeBus->ping();
  return ok;
}

bool imu::enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq) {
  _setBank(2);
  
  // Read current register value
  uint8_t reg_addr = ICM20X_B2_ACCEL_CONFIG_1;
  uint8_t reg_data[2] = {reg_addr, 0};
  
  _sfeBus->writeBytes(reg_data, 1);
  _sfeBus->readBytes(&reg_data[1], 1);
  
  if (enable) {
    // Set ACCEL_FCHOICE = 0 (enable DLPF)
    reg_data[1] &= ~(1 << 0);
    // Set DLPFCFG bits to cutoff_freq
    reg_data[1] &= ~(0x07 << 3); // Clear bits 3-5
    reg_data[1] |= (cutoff_freq & 0x07) << 3; // Set bits 3-5 to cutoff_freq
  } else {
    // Set ACCEL_FCHOICE = 1 (bypass DLPF)
    reg_data[1] |= (1 << 0);
  }
  
  // Write back the modified value
  _sfeBus->writeBytes(reg_data, 2);
  
  _setBank(0);
  return true;
}

bool imu::enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq) {
  _setBank(2);
  
  // Read current register value
  uint8_t reg_addr = ICM20X_B2_GYRO_CONFIG_1;
  uint8_t reg_data[2] = {reg_addr, 0};
  
  _sfeBus->writeBytes(reg_data, 1);
  _sfeBus->readBytes(&reg_data[1], 1);
  
  if (enable) {
    // Set GYRO_FCHOICE = 0 (enable DLPF)
    reg_data[1] &= ~(1 << 0);
    // Set DLPFCFG bits to cutoff_freq
    reg_data[1] &= ~(0x07 << 3); // Clear bits 3-5
    reg_data[1] |= (cutoff_freq & 0x07) << 3; // Set bits 3-5 to cutoff_freq
  } else {
    // Set GYRO_FCHOICE = 1 (bypass DLPF)
    reg_data[1] |= (1 << 0);
  }
  
  // Write back the modified value
  _sfeBus->writeBytes(reg_data, 2);
  
  _setBank(0);
  return true;
}

uint8_t imu::getGyroRateDivisor(void) {
  _setBank(2);
  
  uint8_t reg_addr = ICM20X_B2_GYRO_SMPLRT_DIV;
  uint8_t divisor = 0;
  
  _sfeBus->writeBytes(&reg_addr, 1);
  _sfeBus->readBytes(&divisor, 1);
  
  _setBank(0);
  return divisor;
}

void imu::setGyroRateDivisor(uint8_t new_gyro_divisor) {
  _setBank(2);
  
  uint8_t reg_data[2] = {ICM20X_B2_GYRO_SMPLRT_DIV, new_gyro_divisor};
  _sfeBus->writeBytes(reg_data, 2);
  
  _setBank(0);
}

uint16_t imu::getAccelRateDivisor(void) {
  _setBank(2);
  
  uint8_t reg_addr = ICM20X_B2_ACCEL_SMPLRT_DIV_1;
  uint8_t data[2];
  
  _sfeBus->writeBytes(&reg_addr, 1);
  _sfeBus->readBytes(data, 2); // Read two bytes
  
  uint16_t divisor = (uint16_t)data[0] << 8 | data[1];
  
  _setBank(0);
  return divisor;
}

void imu::setAccelRateDivisor(uint16_t new_accel_divisor) {
  _setBank(2);
  
  uint8_t reg_data[3] = {
    ICM20X_B2_ACCEL_SMPLRT_DIV_1,
    (uint8_t)((new_accel_divisor >> 8) & 0xFF), // MSB
    (uint8_t)(new_accel_divisor & 0xFF)         // LSB
  };
  
  _sfeBus->writeBytes(reg_data, 3);
  
  _setBank(0);
}

void imu::reset(void) {
  _setBank(0);
  
  // Write to PWR_MGMT_1 register, setting the reset bit
  uint8_t reg_data[2] = {ICM20X_B0_PWR_MGMT_1, 0x80}; // 0x80 = reset bit
  _sfeBus->writeBytes(reg_data, 2);
  
  // Wait for reset to complete
  sleep_ms(100);
  
  // Clear reset bit and set clock source
  reg_data[1] = 0x01; // Auto select best clock source
  _sfeBus->writeBytes(reg_data, 2);
  
  // Wait for device to stabilize
  sleep_ms(10);
}

bool imu::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
  // Read sensor data and populate the provided event structures

  // TODO: Read raw sensor data from registers
  _read();

  // Scale the raw values
  scaleValues();

  // Fill in the event structures if provided
  if (accel) {
    accel->version = 1;
    accel->sensor_id = 0; // TODO: Set appropriate sensor ID
    accel->type = 1; // Accelerometer type
    accel->timestamp = 0; // TODO: Set timestamp
    accel->acceleration.x = accX * SENSORS_GRAVITY_EARTH;
    accel->acceleration.y = accY * SENSORS_GRAVITY_EARTH;
    accel->acceleration.z = accZ * SENSORS_GRAVITY_EARTH;
  }

  if (gyro) {
    gyro->version = 1;
    gyro->sensor_id = 1; // TODO: Set appropriate sensor ID
    gyro->type = 4; // Gyroscope type
    gyro->timestamp = 0; // TODO: Set timestamp
    gyro->gyro.x = gyroX * SENSORS_DPS_TO_RADS;
    gyro->gyro.y = gyroY * SENSORS_DPS_TO_RADS;
    gyro->gyro.z = gyroZ * SENSORS_DPS_TO_RADS;
  }

  if (temp) {
    temp->version = 1;
    temp->sensor_id = 2; // TODO: Set appropriate sensor ID
    temp->type = 7; // Temperature type
    temp->timestamp = 0; // TODO: Set timestamp
    temp->temperature = (temperature / 333.87) + 21.0;
  }

  return true;
}
// The rest of the functions remain largely the same

void imu::scaleValues(void) {
  // Scale raw sensor values to appropriate units
  // TODO: Apply calibration and scaling factors to raw values
  // to convert to m/s^2 for accelerometer and rad/s for gyroscope
  icm20649_gyro_range_t gyro_range = (icm20649_gyro_range_t)current_gyro_range;
  icm20649_accel_range_t accel_range =
      (icm20649_accel_range_t)current_accel_range;
  float accel_scale = 1.0;
  float gyro_scale = 1.0;

  if (gyro_range == ICM20649_GYRO_RANGE_500_DPS)
    gyro_scale = 65.5;
  if (gyro_range == ICM20649_GYRO_RANGE_1000_DPS)
    gyro_scale = 32.8;
  if (gyro_range == ICM20649_GYRO_RANGE_2000_DPS)
    gyro_scale = 16.4;
  if (gyro_range == ICM20649_GYRO_RANGE_4000_DPS)
    gyro_scale = 8.2;

  if (accel_range == ICM20649_ACCEL_RANGE_4_G)
    accel_scale = 8192.0;
  if (accel_range == ICM20649_ACCEL_RANGE_8_G)
    accel_scale = 4096.0;
  if (accel_range == ICM20649_ACCEL_RANGE_16_G)
    accel_scale = 2048.0;
  if (accel_range == ICM20649_ACCEL_RANGE_30_G)
    accel_scale = 1024.0;

  gyroX = rawGyroX / gyro_scale;
  gyroY = rawGyroY / gyro_scale;
  gyroZ = rawGyroZ / gyro_scale;

  accX = rawAccX / accel_scale;
  accY = rawAccY / accel_scale;
  accZ = rawAccZ / accel_scale;
}


void imu::_setBank(uint8_t bank_number) {
  // Prepare the data to write
  uint8_t data[2];
  data[0] = ICM20X_B0_REG_BANK_SEL;
  data[1] = (bank_number & 0b11) << 4;
  
  // Write to the bank select register
  _sfeBus->writeBytes(data, 2);
}

void imu::_read(void) {
  _setBank(0);

  // reading 9 bytes of mag data to fetch the register that tells the mag we've
  // read all the data
  const uint8_t numbytes = 14 + 9; // Read Accel, gyro, temp, and 9 bytes of mag
  uint8_t buffer[numbytes];
  
  // First, write the register address we want to read from
  uint8_t reg = ICM20X_B0_ACCEL_XOUT_H;
  _sfeBus->writeBytes(&reg, 1);
  
  // Then read the data
  _sfeBus->readBytes(buffer, numbytes);

  rawAccX = buffer[0] << 8 | buffer[1];
  rawAccY = buffer[2] << 8 | buffer[3];
  rawAccZ = buffer[4] << 8 | buffer[5];

  rawGyroX = buffer[6] << 8 | buffer[7];
  rawGyroY = buffer[8] << 8 | buffer[9];
  rawGyroZ = buffer[10] << 8 | buffer[11];

  temperature = buffer[12] << 8 | buffer[13];

  rawMagX = ((buffer[16] << 8) |
             (buffer[15] & 0xFF)); // Mag data is read little endian
  rawMagY = ((buffer[18] << 8) | (buffer[17] & 0xFF));
  rawMagZ = ((buffer[20] << 8) | (buffer[19] & 0xFF));

  scaleValues();
}
//
// // The scaleValues and getEvent functions can remain unchanged
// imu::imu() {
//   // Constructor implementation
//   temperature = 0;
//   accX = accY = accZ = 0;
//   gyroX = gyroY = gyroZ = 0;
//   magX = magY = magZ = 0;
// }
//
// bool imu::begin(i2c_inst_t *i2cPort, uint8_t i2c_addr) {
//   // Initialization code
//
//   _i2cPort = i2cPort;
//   _address = ICM20649_I2CADDR_DEFAULT;
//   _i2cBus.init(_address, _i2cPort);
//
//   _sfeBus = &_i2cBus;
//   // Reset the device
//   reset();
//
//   // 3 will be the largest range for either sensor
//   writeGyroRange(3);
//   writeAccelRange(3);
//   // TODO: Add actual initialization code for the imu
//   // - Check device ID
//   // - Configure default settings
//   // - Set up default ranges for accelerometer and gyroscope
//
//   if (!ping())
//     return false; // Sensor did not ack
//   return true;
// }
//
//
// void imu::writeAccelRange(uint8_t new_accel_range) {
//   _setBank(2);
//
//   Adafruit_BusIO_Register accel_config_1 = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20X_B2_ACCEL_CONFIG_1);
//
//   Adafruit_BusIO_RegisterBits accel_range =
//       Adafruit_BusIO_RegisterBits(&accel_config_1, 2, 1);
//
//   accel_range.write(new_accel_range);
//   current_accel_range = new_accel_range;
//
//   _setBank(0);
// }
// void imu::writeGyroRange(uint8_t new_gyro_range) {
//   _setBank(2);
//
//   Adafruit_BusIO_Register gyro_config_1 = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20X_B2_GYRO_CONFIG_1);
//
//   Adafruit_BusIO_RegisterBits gyro_range =
//       Adafruit_BusIO_RegisterBits(&gyro_config_1, 2, 1);
//
//   gyro_range.write(new_gyro_range);
//   current_gyro_range = new_gyro_range;
//   _setBank(0);
// }
// bool imu::ping() {
//   bool ok = _sfeBus->ping();
//   return ok;
// }
// bool imu::enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq) {
//   // Enable/disable accelerometer digital low pass filter
//   // TODO: Implement register writes to configure DLPF
//
//   return true;
// }
//
// bool imu::enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq) {
//   // Enable/disable gyroscope digital low pass filter
//   // TODO: Implement register writes to configure DLPF
//
//   return true;
// }
//
// uint8_t imu::getGyroRateDivisor(void) {
//   // Get the gyroscope sample rate divisor
//   // TODO: Read from appropriate register
//
//   return 0;
// }
//
// void imu::setGyroRateDivisor(uint8_t new_gyro_divisor) {
//   // Set the gyroscope sample rate divisor
//   // TODO: Write to appropriate register
// }
//
// uint16_t imu::getAccelRateDivisor(void) {
//   // Get the accelerometer sample rate divisor
//   // TODO: Read from appropriate register
//
//   return 0;
// }
//
// void imu::setAccelRateDivisor(uint16_t new_accel_divisor) {
//   // Set the accelerometer sample rate divisor
//   // TODO: Write to appropriate register
// }
//
// void imu::reset(void) {
//   // Reset the device
//   // TODO: Write to reset register
// }
//


// void imu::_setBank(uint8_t bank_number) {
//   // Prepare the data to write
//   uint8_t data[2];
//   data[0] = ICM20X_B0_REG_BANK_SEL;
//   data[1] = (bank_number & 0b11) << 4;
//
//   // Write to the bank select register
//   _sfeBus->writeBytes(data, 2);
// }
//
// void imu::_read(void) {
//   _setBank(0);
//
//   // reading 9 bytes of mag data to fetch the register that tells the mag we've
//   // read all the data
//   const uint8_t numbytes = 14 + 9; // Read Accel, gyro, temp, and 9 bytes of mag
//   uint8_t buffer[numbytes];
//
//   // First, write the register address we want to read from
//   uint8_t reg = ICM20X_B0_ACCEL_XOUT_H;
//   _sfeBus->writeBytes(&reg, 1);
//
//   // Then read the data
//   _sfeBus->readBytes(buffer, numbytes);
//
//   rawAccX = buffer[0] << 8 | buffer[1];
//   rawAccY = buffer[2] << 8 | buffer[3];
//   rawAccZ = buffer[4] << 8 | buffer[5];
//
//   rawGyroX = buffer[6] << 8 | buffer[7];
//   rawGyroY = buffer[8] << 8 | buffer[9];
//   rawGyroZ = buffer[10] << 8 | buffer[11];
//
//   temperature = buffer[12] << 8 | buffer[13];
//
//   rawMagX = ((buffer[16] << 8) |
//              (buffer[15] & 0xFF)); // Mag data is read little endian
//   rawMagY = ((buffer[18] << 8) | (buffer[17] & 0xFF));
//   rawMagZ = ((buffer[20] << 8) | (buffer[19] & 0xFF));
//
//   scaleValues();
//
// }
