
#include "gps.h"

bool GPS::begin(i2c_inst_t *i2c) {
  uint8_t deviceAddress = kUBLOXGNSSDefaultAddress;
  uint16_t maxWait = kUBLOXGNSSDefaultMaxWait;
  bool assumeSuccess = false;

  rtc_init();

  gpio_init(WAKEUP_PIN);
  gpio_set_dir(WAKEUP_PIN, GPIO_OUT);
  gpio_put(WAKEUP_PIN, 0);

  // Initialize the I2C buss class i.e. setup default Wire port
  _i2cBus.init(deviceAddress, i2c);

  // Initialize the system - return results
  return init(maxWait, assumeSuccess);
}

bool GPS::update() {

  bool fixValid = false;
  if (absolute_time_diff_us(gps_last_check_time, get_absolute_time()) >= gps_check_interval_ms * 1000) {
    gps_last_check_time = get_absolute_time();
    if (getPVT() == true) {
      bool fixOk = getGnssFixOk();
      uint8_t pdop = getPDOP();

      if (getTimeFullyResolved() && getTimeValid() && fixOk && (pdop < 5)) {
          fixValid = true;
      }
         
    }
  }

  if (fixValid) {
    latest_location.start_time = getUnixEpoch();
    setRTCfromUnixEpoch(latest_location.start_time);
    latest_location.lat = getLatitude() / 1e7;
    latest_location.lon = getLongitude() / 1e7;

    DEBUG_PRINT(("%d-%d-%d %d:%d:%d ", getYear(), getMonth(), getDay(), getHour(), getMinute(), getSecond()));
    DEBUG_PRINT(("Lat: %f Lon: %f\n", latest_location.lat, latest_location.lon));
    setNightMode();
    deactivate();
    return false;
  }

  if (absolute_time_diff_us(gps_start_time, get_absolute_time()) >= gps_run_time * 1000) {
    latest_location.start_time = getUnixEpochfromRTC();
    latest_location.lat = 0.0f;
    latest_location.lon = 0.0f;
    DEBUG_PRINT(("No GPS fix, using RTC unix epoch time: %d\n", latest_location.start_time));

    setNightMode();
    deactivate();
    return false;
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
    
    // Convert to tm structure
    struct tm timeinfo;
    timeinfo.tm_year = dt.year - 1900;  // tm_year is years since 1900
    timeinfo.tm_mon = dt.month - 1;     // tm_mon is 0-11
    timeinfo.tm_mday = dt.day;
    timeinfo.tm_hour = dt.hour;
    timeinfo.tm_min = dt.min;
    timeinfo.tm_sec = dt.sec;
    timeinfo.tm_isdst = -1;             // Let mktime determine DST status
    
    // Convert to Unix epoch time
    time_t unix_time = mktime(&timeinfo);
    return (uint32_t)unix_time;
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
  DEBUG_PRINT(("Waking up the GPS module via pin %d \n", WAKEUP_PIN));

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
