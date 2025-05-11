#pragma once

#include "pico/stdlib.h"
#include <stdio.h>

#include "pico/sleep.h"

#include "uart_settings.h"
#include "debug.h"


void enter_low_power_mode_ms(uint32_t sleep_time);
static void alarm_sleep_callback(uint alarm_id);

