
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
#include "lora.h"

