/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include "lib/gps_pico.h"
#include "pico/stdlib.h"
#include <stdio.h>
// DevUBLOXGNSS gps;

int main() {
  stdio_init_all();
  // gps.begin();
  while (true) {
    printf("Hello, Griffin!\n");
    sleep_ms(1000);
  }
}
