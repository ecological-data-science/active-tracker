#pragma once

#include "hardware/i2c.h"
#include "gps_pico.h"
#include "pico/stdlib.h"
#include <stdio.h>


// class for the GPS module

class GPS : public gps_pico
{
public:
  GPS() : gps_pico() {}
  ~GPS() {}

  bool update();
  bool activate();
private:
  bool deactivate();

  absolute_time_t gps_last_check_time;
  uint32_t gps_check_interval_ms = 1000; // 1 second interval

};

