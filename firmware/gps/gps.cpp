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
#include "lib/gps_pico.h"
#include "pico/stdlib.h"
#include <stdio.h>

DevUBLOXGNSS gps;

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

  printf("Starting GPS ...\n");
  sleep_ms(1000);
  // Now initialize the GPS
  if (gps.begin(i2c0) == false) {
    printf("GPS module not detected, check wiring\n");
    while (1) {
      sleep_ms(100);
    }
  }

  printf("GPS module initialized successfully\n");

  while (true) {
    if (gps.getPVT() == true) {
      int32_t latitude = gps.getLatitude();
      printf("Lat: %ld", latitude);

      int32_t longitude = gps.getLongitude();
      printf(" Long: %ld", longitude);
      printf(" (degrees * 10^-7) Datetime:");

      printf("%d-%d-%d %d:%d:%d", gps.getYear(), gps.getMonth(), gps.getDay(),
             gps.getHour(), gps.getMinute(), gps.getSecond());

      bool fixOk = gps.getGnssFixOk();
      printf(" Fix ok: %d\n", fixOk);

      uint8_t pdop = gps.getPDOP();
      printf(" PDOP: %d\n", pdop);

      if (gps.getTimeFullyResolved() && gps.getTimeValid()) {
        printf("Time is valid and fully resolved\n");
      }
    }

    sleep_ms(1000 * 10);
  }
}
