
#include "classifier.h"

bool Classifier::begin(i2c_inst_t *i2c) {
  // Initialize the system - return results
  // Initialization code
  _i2cPort = i2c;
  _address = ICM20649_I2CADDR_DEFAULT;
  _i2cBus.init(_address, _i2cPort);

  // Reset the device
  reset();

  // 3 will be the largest range for either sensor
  writeGyroRange(3);
  writeAccelRange(3);
  // 1100Hz/(1+10) = 100Hz
  // setGyroRateDivisor(10);

  // # 1125Hz/(1+20) = 53.57Hz
  // setAccelRateDivisor(20);
  if (!ping())
    return false; // Sensor did not ack

  setAccelRateDivisor(225);
  setGyroRateDivisor(225);


  enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);
  sleep_ms(500);

  getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
    
  acc_bias[2] = pow( ax*ax + ay*ay + az*az,0.5) ;
  filter.setup( ax,ay,az);     

  // initialize the tflite classifier
  tflite.begin();
  return true;
}

bool Classifier::update() {
  if (absolute_time_diff_us(imu_last_check_time, get_absolute_time()) < imu_interval_ms * 1000)
    return true;
  imu_last_check_time = get_absolute_time();
  segment_counter++;
  update_imu(&predict_data[segment_counter * N_CHANNELS]);
  if (segment_counter == SEG_LENGTH) {
    int activity = tflite.classify(predict_data);
    DEBUG_PRINT(("Activity classification: %d\n", activity));
    segment_counter = 0;

    // set so that the first entry goes into the most significant bit and we
    // read right to left in order of the temporal sequence
    int msb = 3 - bit_counter;

    switch (activity) {
    case 0:
      break;
    case 1:
      // bitWrite(latest_activity.activities[imu_counter], 2*msb, 1)
      latest_activity.activities[imu_counter] |= (1 << (2 * msb));
      break;
    case 2:
      // bitWrite(latest_activity.activities[imu_counter], 2*msb+1, 1)
      latest_activity.activities[imu_counter] |= (1 << (2 * msb + 1));
      break;
    case 3:
      // bitWrite(latest_activity.activities[imu_counter], 2*msb, 1)
      latest_activity.activities[imu_counter] |= (1 << (2 * msb));
      // bitWrite(latest_activity.activities[imu_counter], 2*msb+1, 1)
      latest_activity.activities[imu_counter] |= (1 << (2 * msb + 1));
      break;
    }

    bit_counter++;

    if (bit_counter == BIT_LENGTH) {
      bit_counter = 0;
      imu_counter++;
    }

    if (imu_counter == SERIES_LENGTH)
      return false;
  }
  return true;
}

void Classifier::update_imu(float *pdata) {

  getEvent(&accel, &gyro, &temp);
  float gx = float(gyro.gyro.x);
  float gy = float(gyro.gyro.y);
  float gz = float(gyro.gyro.z);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
  filter.update(gx, gy, gz, ax, ay, az);

  float pitch = float(filter.pitch());
  float roll = float(filter.roll());
  float v[3] = {ax, ay, az};

  filter.projectVector(true, v);

  acc_bias[0] = (1.0 - bias_gain) * acc_bias[0] + bias_gain * v[0];
  acc_bias[1] = (1.0 - bias_gain) * acc_bias[1] + bias_gain * v[1];
  acc_bias[2] = (1.0 - bias_gain) * acc_bias[2] + bias_gain * v[2];

  angle_bias[0] = (1.0 - bias_gain) * angle_bias[0] + bias_gain * pitch;
  angle_bias[1] = (1.0 - bias_gain) * angle_bias[1] + bias_gain * roll;

  pdata[0] = v[0] - acc_bias[0];
  pdata[1] = v[1] - acc_bias[1];
  pdata[2] = v[2] - acc_bias[2];
  pdata[3] = pitch - angle_bias[0];
  pdata[4] = roll - angle_bias[1];
}

activity_reading Classifier::get_activity() {
  return latest_activity;
}

void Classifier::activate(long unixtime){

  latest_activity.start_time = unixtime;

  memset(latest_activity.activities,0,sizeof(latest_activity.activities));
  // TODO - reset the IMU and reinitialize??
  imu_counter = 0;
  bit_counter = 0;
  segment_counter = 0;
  return; 
}

void Classifier::deactivate() {
  return; // Implement deactivation logic if needed
}
