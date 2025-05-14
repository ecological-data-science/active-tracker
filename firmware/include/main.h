
#pragma once

#define PICO_DEFAULT_I2C 0
#define PICO_DEFAULT_I2C_SDA_PIN 4
#define PICO_DEFAULT_I2C_SCL_PIN 5

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "pico/flash.h"

#include "debug.h"
#include "gps.h"
#include "classifier.h"
#include "lowpower.h"
#include "storage.h"
#include "lora.h"

bool setup();
void loop();

static void turn_off_all_leds();
void sleep_until_next_hour_boundary(absolute_time_t start_time);

absolute_time_t start_time;
