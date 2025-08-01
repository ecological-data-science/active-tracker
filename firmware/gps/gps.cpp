
#include "gps.h"

bool GPS::begin(i2c_inst_t *i2c) {
  uint8_t deviceAddress = kUBLOXGNSSDefaultAddress;
  uint16_t maxWait = kUBLOXGNSSDefaultMaxWait;
  bool assumeSuccess = false;

  rtc_init();
  datetime_t t = {
        .year  = 2025,
        .month = 5,
        .day   = 23,
        .dotw  = 5, 
        .hour  = 12,
        .min   = 00,
        .sec   = 00
    };
  rtc_set_datetime(&t);

  gpio_init(WAKEUP_PIN);
  gpio_set_dir(WAKEUP_PIN, GPIO_OUT);
  gpio_put(WAKEUP_PIN, 0);

  activate();

  // Initialize the I2C buss class i.e. setup default Wire port
  _i2cBus.init(deviceAddress, i2c);

  // Initialize the system - return results
  return init(maxWait, assumeSuccess);
}

bool GPS::update() {

  bool fixValid = false; // fully resolved GPS fix - has to have pdop < 5, time fully resolved, and time valid
  bool fixOk = false; // GNSS fix ok - use if not fully resolved after gps_run_time

  if (absolute_time_diff_us(gps_last_check_time, get_absolute_time()) >= gps_check_interval_ms * 1000) {
    gps_last_check_time = get_absolute_time();
    if (getPVT() == true) {
      fixOk = getGnssFixOk();
      uint8_t pdop = getPDOP();

      DEBUG_PRINT(("%d-%d-%d %d:%d:%d ", getYear(), getMonth(), getDay(), getHour(), getMinute(), getSecond()));
      DEBUG_PRINT(("Lat: %f Lon: %f", latest_location.lat, latest_location.lon));
      DEBUG_PRINT(("PDOP: %d, Fix OK: %d", pdop, fixOk));
      if (getTimeFullyResolved() && getTimeValid() && fixOk && (pdop < MIN_PDOP_THRESHOLD)) {
          fixValid = true;
      }
         
    }

    if (fixValid) {
      latest_location.start_time = getUnixEpoch();
      setRTCfromUnixEpoch(latest_location.start_time);
      latest_location.lat = getLatitude() / 1e7;
      latest_location.lon = getLongitude() / 1e7;

      DEBUG_PRINT(("%d-%d-%d %d:%d:%d ", getYear(), getMonth(), getDay(), getHour(), getMinute(), getSecond()));
      DEBUG_PRINT(("Lat: %f Lon: %f", latest_location.lat, latest_location.lon));
      setNightMode();
      deactivate();
      return false;
    }

    if (absolute_time_diff_us(gps_start_time, get_absolute_time()) >= gps_run_time * 1000) {

      if (getTimeFullyResolved() && getTimeValid()) {
        latest_location.start_time = getUnixEpoch();
        setRTCfromUnixEpoch(latest_location.start_time);
      } else {
        latest_location.start_time = getUnixEpochfromRTC();
        DEBUG_PRINT(("No fully resolved GPS fix, using RTC unix epoch time: %u", latest_location.start_time));
      }

      if (fixOk) {
        latest_location.lat = getLatitude() / 1e7;
        latest_location.lon = getLongitude() / 1e7;
      } else {
        latest_location.lat = 0.0f;
        latest_location.lon = 0.0f;
      }

      setNightMode();
      deactivate();
      return false;
    }
  }
  return true;
}

void GPS::setNightMode() {
    // Set night mode based on the given Unix epoch time
    time_t time = (time_t)latest_location.start_time;
    struct tm *utc_time = gmtime(&time);  
    int hour = utc_time->tm_hour;
    
    // check if daylight
    if (hour >= UTC_DAY_START && hour < UTC_DAY_END) {
        nightmode = false;
    } else {
        nightmode = true;
    }
}

uint32_t GPS::getUnixEpochfromRTC() {
    // Get the current time from the RTC
    datetime_t dt;
    rtc_get_datetime(&dt);
    
    // Calculate Unix timestamp directly (UTC)
    uint32_t unix_time = 0;
    
    // Seconds from 1970 till start of year
    int year = dt.year;
    unix_time = (year - 1970) * 365 * 24 * 3600;
    
    // Add leap day seconds for previous years
    for (int y = 1970; y < year; y++) {
        if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
            unix_time += 24 * 3600;
        }
    }
    
    // Add seconds from months
    const uint16_t monthDays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unix_time += monthDays[dt.month - 1] * 24 * 3600;
    
    // Add leap day if needed for current year
    if (dt.month > 2 && 
        ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
        unix_time += 24 * 3600;
    }
    
    // Add days, hours, minutes, seconds
    unix_time += (dt.day - 1) * 24 * 3600;
    unix_time += dt.hour * 3600;
    unix_time += dt.min * 60;
    unix_time += dt.sec;
    
    return unix_time;
}
void GPS::setRTCfromUnixEpoch(uint32_t unix_time) {
    // Set the RTC time to the given Unix epoch time
    time_t time = (time_t)unix_time;
    struct tm *timeinfo = gmtime(&time);  
    
    // Convert to datetime_t structure
    datetime_t dt;
    dt.year = timeinfo->tm_year + 1900;
    dt.month = timeinfo->tm_mon + 1;
    dt.day = timeinfo->tm_mday;
    dt.hour = timeinfo->tm_hour;
    dt.min = timeinfo->tm_min;
    dt.sec = timeinfo->tm_sec;
    dt.dotw = timeinfo->tm_wday;  // 0 is Sunday
    
    // Set the RTC
    rtc_set_datetime(&dt);
}


location_reading GPS::get_location() {
  return latest_location;
}
bool GPS::getNightMode() {
  return nightmode; 
}
void GPS::activate() {
  DEBUG_PRINT(("Waking up the GPS module via pin %d", WAKEUP_PIN));

  gpio_put(WAKEUP_PIN, 0); // Set LOW
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 1); // Set HIGH
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 0); // Set LOW
  gps_last_check_time = 0; // Reset the last check time
  gps_start_time = get_absolute_time();
  return;           
}

void GPS::deactivate() {
  powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  return; 
}
