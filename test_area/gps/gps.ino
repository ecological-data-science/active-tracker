/*
 *
 * GPS requisite functionality code
 *
 *
 */


#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS gps;

//#include "pico/stdlib.h"
//#include "pico/sleep.h"

#define PIN_LED_R (18u)
#define PIN_LED_G (19u)
#define PIN_LED_B (20u)

// correct max value to be determined
#define MAX_PDOP 4.0

// define a digital pin capable of driving HIGH and LOW
#define WAKEUP_PIN 7


void wake_gps() {

  Serial.print("-- waking up GPS module via pin " + String(WAKEUP_PIN));
  Serial.println(" on your microcontroller --");

  digitalWrite(WAKEUP_PIN, LOW);
  delay(1000);
  digitalWrite(WAKEUP_PIN, HIGH);
  delay(1000);
  digitalWrite(WAKEUP_PIN, LOW);
}


void setup() {

  //pinMode(WAKEUP_PIN, OUTPUT);
  //digitalWrite(WAKEUP_PIN, LOW);

  Wire.begin();
  Serial.begin(115200);
  // while (!Serial); //Wait for user to open terminal
  delay(5000);
  Serial.println("SparkFun u-blox Example");

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_G, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PIN_LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)

  //gps.enableDebugging(); // Enable debug messages

  if (gps.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("u-blox GNSS detected at default I2C address"));
  delay(5000);

}

void loop() {
  //Do nothing
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if (gps.getPVT() == true)
  {
    int32_t latitude = gps.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = gps.getLongitude();
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


    if (gps.getTimeFullyResolved() &&  gps.getTimeValid())
    {
      Serial.println(F("Time is valid and fully resolved"));
    }

  }
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  Serial.println(F("Going to sleep for 1 minute"));
  // sleep for 1 minute
  gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  delay(1000*60);

  // wake the gps
  wake_gps();
}

