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

 IMU imu;

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

  while (true) {
    printf("IMU module is working\n");

    sleep_ms(1000 * 10);
  }
}
