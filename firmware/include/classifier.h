#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

// class for the classifier
#define N_CHANNELS 5
#define SEG_LENGTH 50


#define BIT_LENGTH 4        // we write 4 predictions to a single byte as we need 2 bits per prediction
#define SERIES_LENGTH 45    // each byte is 40 seconds (4 predictions x 10s per prediction) 30 minutes is 45 bytes

#define N_INPUTS SEG_LENGTH*N_CHANNELS
#define N_OUTPUTS 4

class classifier : public imu {
public:
  classifier();
  ~classifier();

  bool begin(i2c_inst_t *i2c);
  bool update();
  void activate();

private:
  void deactivate();
  void update_imu(float *pdata);

  absolute_time_t imu_last_check_time;
  uint32_t imu_interval_ms = 1000; // 1 second interval

  unsigned int imu_counter = 0;
  unsigned int bit_counter = 0;
  unsigned int segment_counter = 0;

  float predict_data[N_INPUTS];
  float prediction[N_OUTPUTS];
};
