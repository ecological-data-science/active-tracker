
#include "gps.h"

bool GPS::begin(i2c_inst_t *i2c) {
  uint8_t deviceAddress = kUBLOXGNSSDefaultAddress;
  uint16_t maxWait = kUBLOXGNSSDefaultMaxWait;
  bool assumeSuccess = false;

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
  if (absolute_time_diff_us(gps_last_check_time, get_absolute_time()) >=
      gps_check_interval_ms * 1000) {
    // Time to check GPS
    gps_last_check_time = get_absolute_time();
    // Your GPS checking code here
    if (getPVT() == true) {
      int32_t latitude = getLatitude();
      printf("Lat: %ld", latitude);

      int32_t longitude = getLongitude();
      printf(" Long: %ld", longitude);
      printf(" (degrees * 10^-7) Datetime:");

      printf("%d-%d-%d %d:%d:%d", getYear(), getMonth(), getDay(), getHour(),
             getMinute(), getSecond());

      bool fixOk = getGnssFixOk();
      printf(" Fix ok: %d\n", fixOk);

      uint8_t pdop = getPDOP();
      printf(" PDOP: %d\n", pdop);

      if (getTimeFullyResolved() && getTimeValid()) {
        printf("Time is valid and fully resolved\n");
        if (fixOk) {
          printf("GPS fix is valid\n");
          fixValid = true;
        } else {
          printf("GPS fix is not valid\n");
        }
      }
    }
    printf("No PVT data available\n");
  }

  if (absolute_time_diff_us(gps_start_time, get_absolute_time()) >=
      gps_run_time * 1000) {
    // Time to stop GPS
    printf("GPS run time exceeded, stopping GPS\n");
    deactivate();
    return false;
  }
  if (fixValid) {
    // TODO save the fix data to a file 
    latest_location.start_time = getUnixTime();
    latest_location.lat = getLatitude() / 1e7;
    latest_location.lon = getLongitude() / 1e7;
    int hour = getHour();
    // check if daylight
    if (hour >= 6 && hour < 18) {
      nightmode = false;
    } else {
      nightmode = true;
    }

    deactivate();
    return false;
  }
  return true;
}

location_reading GPS::get_location() {
  return latest_location;
}
bool GPS::getNightMode() {
  return nightmode; 
}
void GPS::activate() {
  printf("-- waking up GPS module via pin %d on your microcontroller --\n",
         WAKEUP_PIN);

  gpio_put(WAKEUP_PIN, 0); // Set LOW
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 1); // Set HIGH
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 0); // Set LOW
    gps_last_check_time = 0; // Reset the last check time
  gps_start_time = get_absolute_time();
  return;             // Implement activation logic if needed
}

void GPS::deactivate() {
  powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  return; // Implement deactivation logic if needed
}
