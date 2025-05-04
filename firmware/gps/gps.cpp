
#include "gps.h"

bool GPS::update() {

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
      }
      return true; // Successfully updated
    }
    printf("No PVT data available\n");
  }
  return true;
}

bool GPS::activate() {
  return true; // Implement activation logic if needed
}

bool GPS::deactivate() {
  return true; // Implement deactivation logic if needed
}
