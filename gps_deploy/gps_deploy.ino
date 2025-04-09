/*
 *
 * GPS requisite functionality code
 *
 *
 */

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS gps;

#include "gps_deploy.h"
#include "lora.h"
#include "storage.h"

#include "pico/stdlib.h"
// #include "pico/sleep.h"

#define PIN_LED_R (18u)
#define PIN_LED_G (19u)
#define PIN_LED_B (20u)

// correct max value to be determined
#define MAX_PDOP 4.0

// define a digital pin capable of driving HIGH and LOW
#define WAKEUP_PIN 7

Lora lora;
Storage storage;

bool GPS_ACTIVE = false;
bool LORA_ACTIVE = false;

// gps will run for up to 10 minutes to get a fix
unsigned long gps_run_time = 1000 * 60 * 10;
// lora will broadcast for up to 40 minutes every hour
unsigned long lora_run_time = 1000 * 60 * 40;

unsigned long gps_check_interval = 60 * 1000; // check the gps fix every minute
unsigned long gps_last_check_time = 0;

unsigned long start_time = 0;
unsigned long lora_start_time = 0;

void wake_gps() {

  Serial.print("-- waking up GPS module via pin " + String(WAKEUP_PIN));
  Serial.println(" on your microcontroller --");

  digitalWrite(WAKEUP_PIN, LOW);
  delay(1000);
  digitalWrite(WAKEUP_PIN, HIGH);
  delay(1000);
  digitalWrite(WAKEUP_PIN, LOW);
  gps_last_check_time = millis();
}

void wake_gps_old() {
  Serial.println("waking up gps");
  gps_last_check_time = millis();
  // digitalWrite(GPS_WAKE_PIN, HIGH);
  delay(1000);
  // digitalWrite(GPS_WAKE_PIN, LOW);

  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  // GPS.fix = false;
  // GPS.HDOP = 2.0*MAX_HDOP;
  // GPS.satellites = 0;
}

void setup_old() {
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage
  // level) Serial.begin(115200); delay(4000);
  // Serial.println("\n\n*****************\n*****************\n");
  //
  // if (!lora.begin())
  // {
  //   Serial.println("ERROR: lora");
  // }
  // delay(500);
  //
  //
  // delay(500);
  //
  //
  //
  //
  // pinMode(GPS_WAKE_PIN, OUTPUT);
  // digitalWrite(GPS_WAKE_PIN, HIGH);
  // delay(1000);
  // digitalWrite(GPS_WAKE_PIN, LOW);
  //
  // if (!GPS.begin(I2C_ADDRESS))
  // {
  //   Serial.println("ERROR: GPS");
  // }
  // delay(500);
  //
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  //
  // rtc.begin(); // initialize RTC
  // delay(500);
  //
}
void setup() {

  Wire.begin();
  Serial.begin(115200);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(10000);
  digitalWrite(PIN_LED_G, HIGH); // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_B, HIGH); // turn the LED on (HIGH is the voltage level)

  pinMode(WAKEUP_PIN, OUTPUT);
  digitalWrite(WAKEUP_PIN, LOW);

  if (!lora.begin()) {
    Serial.println("ERROR: lora");
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_B, LOW);
  }

  if (!storage.begin()) {
    Serial.println("ERROR: storage");
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_B, LOW);
  }

  if (!gps.begin()) {
    Serial.println(F("ERROR: gps"));
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_B, LOW);
  }

  delay(5000);

  for (int i = 0; i < 10; i++) {
    digitalWrite(PIN_LED_R, HIGH); // 10 flashes indicate success
    delay(200);
    digitalWrite(PIN_LED_G, HIGH); // 10 flashes indicate success
    delay(200);
    digitalWrite(PIN_LED_B, HIGH); // 10 flashes indicate success
    delay(200);
    digitalWrite(PIN_LED_R, LOW);
    delay(200);
    digitalWrite(PIN_LED_G, LOW);
    delay(200);
    digitalWrite(PIN_LED_B, LOW);
  }
}

// void pause_gps()
// {
//   GPS.sendCommand(PMTK_BACKUP_MODE);
// }

bool waiting_on_first_fix = true;

void update_time() {

  // if ((GPS.satellites > 10)||(waiting_on_first_fix))
  // {
  //   if((GPS.year > 2020) && (GPS.year < 2079)) {
  //     rtc.setTime(GPS.hour, GPS.minute, GPS.seconds);
  //     rtc.setDate(GPS.day, GPS.month, GPS.year);
  //     waiting_on_first_fix = false;
  //   }
  // }
}

unsigned int imu_count;

location_reading latest_location;

void loop() {

  if ((!GPS_ACTIVE) && (!LORA_ACTIVE)) {
    // if nothing active then we are at the top of the hour so wake up the gps
    // and imu and reset variables
    wake_gps();
    GPS_ACTIVE = true;
    start_time = millis();
    latest_location.lat = 0.0f;
    latest_location.lon = 0.0f;
    lora.session_success = false;

    // wdt.setup(WDT_SOFTCYCLE16M);  // initialize WDT-softcounter refesh cycle
    // on 16 minute interval
  }

  if (GPS_ACTIVE) {
    if ((millis() - gps_last_check_time) >= gps_check_interval) {
      if (gps.getPVT() == true) {
        float latitude = gps.getLatitude();
        latitude = latitude / 10000000;

        float longitude = gps.getLongitude();
        longitude = longitude / 10000000;

        Serial.print(F("Lat: "));
        Serial.print(latitude);
        Serial.print(F(" Long: "));
        Serial.print(longitude);
        Serial.print(F(" (degrees * 10^-7) Datetime:"));

        Serial.print(gps.getYear());
        Serial.print(F("-"));
        Serial.print(gps.getMonth());
        Serial.print(F("-"));
        Serial.print(gps.getDay());
        Serial.print(F(" "));
        Serial.print(gps.getHour());
        Serial.print(F(":"));
        Serial.print(gps.getMinute());
        Serial.print(F(":"));
        Serial.print(gps.getSecond());

        bool fixOk = gps.getGnssFixOk();
        Serial.print(F(" Fix ok: "));
        Serial.print(fixOk);
        Serial.println();

        uint8_t pdop = gps.getPDOP();

        Serial.print(F(" PDOP: "));
        Serial.print(pdop);
        Serial.println();

        if (gps.getTimeFullyResolved() && gps.getTimeValid()) {
          Serial.println(F("Time is valid and fully resolved"));
        }
        latest_location.lat = latitude;
        latest_location.lon = longitude;
        latest_location.start_time = gps.getUnixEpoch();
      }
      gps_last_check_time = millis();
    }

    if (millis() - start_time >= gps_run_time) {
      gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
      GPS_ACTIVE = false;
    }
    Serial.println(F("Going to sleep for 1 minute"));
    // sleep for 1 minute
    delay(1000 * 60);

    // wake the gps
    wake_gps();

    //       char c = GPS.read();
    //       if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
    //
    //       if ((millis() - gps_last_check_time) >= gps_check_interval) //
    //       Check fix quality every minute
    //       {
    //         if ( ( (GPS.fix) && (GPS.HDOP < MAX_HDOP) ) || ( millis() -
    //         start_time >= gps_run_time) )
    //         {
    //
    // //          Serial.print("\nTime: ");
    // //          if (GPS.hour < 10) { Serial.print('0'); }
    // //          Serial.print(GPS.hour, DEC); Serial.print(':');
    // //          if (GPS.minute < 10) { Serial.print('0'); }
    // //          Serial.print(GPS.minute, DEC); Serial.print(':');
    // //          if (GPS.seconds < 10) { Serial.print('0'); }
    // //          Serial.println(GPS.seconds, DEC);
    // //          Serial.print("Location: ");
    // //          Serial.print(GPS.latitudeDegrees); Serial.print(GPS.lat);
    // //          Serial.print(", ");
    // //          Serial.print(GPS.longitudeDegrees); Serial.println(GPS.lon);
    //
    //            // either we got a fix or we're out of time
    //            if (GPS.fix)
    //            {
    //               latest_location.lat = GPS.latitudeDegrees;
    //               latest_location.lon = GPS.longitudeDegrees;
    //               update_time();
    //            }
    //
    //
    //            latest_location.start_time = rtc.getEpoch();
    //            GPS_ACTIVE=false;
    //            pause_gps();
    //         }
    //         gps_last_check_time = millis();
    //
    //       }
  }

  if ((!LORA_ACTIVE) && (!GPS_ACTIVE)) {
    // add the readings to storage
    // storage.begin();
    storage.set_latest_message(latest_location);
    // storage.write_next_message(latest_location, classifier.latest_activity);
    // storage.sleep();

    // turn on lora
    LORA_ACTIVE = true;
    lora_start_time = millis();
  }

  if (LORA_ACTIVE) {
    LORA_ACTIVE = lora.update(&storage);
    if (millis() - lora_start_time >= lora_run_time)
      LORA_ACTIVE = false;
  }

  // wdt.clear();

  if ((!GPS_ACTIVE) && (!LORA_ACTIVE)) {

    if (storage.pending_archive) {
      storage.store_latest_message();
    }

    // wdt.setup(WDT_OFF);  //watchdog
    unsigned long run_time = millis() - start_time;
    Serial.println("going to sleep...");
    Serial.print("back in ");
    Serial.print(60 * 60 * 1000 - run_time);
    Serial.println(" milliseconds.");

    // LowPower.deepSleep(60*60*1000 - run_time);
  }
}
