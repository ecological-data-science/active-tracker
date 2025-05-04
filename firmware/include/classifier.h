#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "filter/imuFilter.h"
#include "imu.h"
#include <stdio.h>

// class for the classifier
#define N_CHANNELS 5
#define SEG_LENGTH 50


#define BIT_LENGTH 4        // we write 4 predictions to a single byte as we need 2 bits per prediction
#define SERIES_LENGTH 45    // each byte is 40 seconds (4 predictions x 10s per prediction) 30 minutes is 45 bytes

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

typedef struct
{
      long start_time;
      uint8_t activities[45];
}  activity_reading;




constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.


constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.
class Classifier : public imu {

public:
  // Classifier();
  // ~Classifier();

  bool begin(i2c_inst_t *i2c);
  bool update();
  void activate();

private:
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imuFilter <&GAIN> filter;

  float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  float angle_bias[2] = {0.0f, 0.0f};
  void deactivate();
  void update_imu(float *pdata);

  activity_reading latest_activity;

  absolute_time_t imu_last_check_time;
  uint32_t imu_interval_ms = 200; // 5hz interval

  unsigned int imu_counter = 0;
  unsigned int bit_counter = 0;
  unsigned int segment_counter = 0;

  float predict_data[N_INPUTS];
  float prediction[N_OUTPUTS];
};
