/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define PICO_DEFAULT_I2C 0
#define PICO_DEFAULT_I2C_SDA_PIN 4
#define PICO_DEFAULT_I2C_SCL_PIN 5

#include "hardware/i2c.h"
#include "lib/IMU.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "lib/filter/imuFilter.h"

IMU imu;


sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;


constexpr float GAIN = 0.75;     // Fusion gain determines response of heading correction with respect to gravity.
imuFilter <&GAIN> filter;

float acc_bias[3] = {0.0f, 0.0f, SENSORS_GRAVITY_EARTH};
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float angle_bias[2] = {0.0f, 0.0f};


constexpr float bias_gain = 0.00001;     // averaging decay rate - should be very low.

#define N_INPUTS 5





float predict_data[N_INPUTS];

void update_imu(float *pdata){
  imu.getEvent(&accel, &gyro, &temp);
  float gx = float(gyro.gyro.x);
  float gy = float(gyro.gyro.y); 
  float gz = float(gyro.gyro.z); 
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
  filter.update(gx, gy, gz, ax, ay, az);

  float pitch = float(filter.pitch());
  float roll = float(filter.roll());
  float v[3] = { ax, ay, az };

  filter.projectVector( true, v );

  acc_bias[0] = (1.0-bias_gain)*acc_bias[0] + bias_gain*v[0];
  acc_bias[1] = (1.0-bias_gain)*acc_bias[1] + bias_gain*v[1];
  acc_bias[2] = (1.0-bias_gain)*acc_bias[2] + bias_gain*v[2];

  angle_bias[0] = (1.0-bias_gain)*angle_bias[0] + bias_gain*pitch;
  angle_bias[1] = (1.0-bias_gain)*angle_bias[1] + bias_gain*roll;

  pdata[0] = v[0]-acc_bias[0];
  pdata[1] = v[1]-acc_bias[1];
  pdata[2] = v[2]-acc_bias[2];
  pdata[3] = pitch-angle_bias[0];
  pdata[4] = roll-angle_bias[1];

  
}
int main() {
  stdio_init_all();
  sleep_ms(5000);

  // Initialize I2C using the default pins
  i2c_init(i2c0, 100 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  sleep_ms(1000);

  printf("Starting IMU ...\n");
  sleep_ms(1000);
  // Now initialize the IMU
  if (imu.begin(i2c0) == false) {
    printf("IMU module not detected, check wiring\n");
    while (1) {
      sleep_ms(100);
    }
  }

  



  printf("IMU module initialized successfully\n");

  imu.setAccelRateDivisor(225);
  imu.setGyroRateDivisor(225);


  imu.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  imu.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);
  sleep_ms(500);

  imu.getEvent(&accel, &gyro, &temp);
  float ax = float(accel.acceleration.x);
  float ay = float(accel.acceleration.y);
  float az = float(accel.acceleration.z);
    
  acc_bias[2] = pow( ax*ax + ay*ay + az*az,0.5) ;
  filter.setup( ax,ay,az);     

  while (true) {
    // printf("IMU module is working\n");

    update_imu(predict_data);
    printf("Ax: %f Ay: %f Az: %f Pitch: %f Roll: %f\n", predict_data[0], predict_data[1], predict_data[2], predict_data[3], predict_data[4]);
    // bool success = imu.getEvent(&accel, &gyro, &temp);
    // printf("Accel: x: %f y: %f z: %f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    // printf("Gyro: x: %f y: %f z: %f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    // printf("Temp: %f\n", temp.temperature);
    sleep_ms(200 * 1);
  }
}


