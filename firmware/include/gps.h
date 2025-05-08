#pragma once

#include "hardware/i2c.h"
#include "gps_pico.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "data_structs.h"
#include "hardware/rtc.h"
#include <time.h>
#include "debug.h"

#define WAKEUP_PIN 7

#define UTC_DAY_START 3
#define UTC_DAY_END 16


class GPS : public gps_pico
{
public:
  GPS() : gps_pico() {}
  ~GPS() {}

  bool begin(i2c_inst_t *i2c);
  bool update();
  void activate();
  location_reading get_location();
  bool getNightMode();

  // get the unix epoch time
  uint32_t getUnixEpochfromRTC();

private:
  void deactivate();
  location_reading latest_location;
  absolute_time_t gps_last_check_time;
  absolute_time_t gps_start_time;

  void setRTCfromUnixEpoch(uint32_t unix_time);
  void setNightMode();
  uint32_t gps_run_time = 1000 * 60 * 10; // GPS will run for up to 10 minutes to get a fix
  uint32_t gps_check_interval_ms = 1000 * 10; // 10 second interval
  bool nightmode = false;

};

