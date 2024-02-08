#include "pico/stdlib.h"
#include "pico/sleep.h"

#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/vreg.h"
#include "hardware/structs/scb.h"

#define PIN_LED_R (18u)
#define PIN_LED_G (19u)
#define PIN_LED_B (20u)
#define UART_TX (0u)

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS gps;

static void sleep_callback(void) {
    uart_default_tx_wait_blocking();
    return;
}

static void rtc_sleep(int8_t minute_to_sleep_to, int8_t second_to_sleep_to) {

    datetime_t t_alarm = {
            .year  = 2020,
            .month = 06,
            .day   = 05,
            .dotw  = 5, // 0 is Sunday, so 5 is Friday
            .hour  = 15,
            .min   = minute_to_sleep_to,
            .sec   = second_to_sleep_to
    };

    uart_default_tx_wait_blocking();
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);
}


void recover_from_sleep(uint scb_orig, uint clock0_orig, uint clock1_orig){

    //Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    //reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    //reset clocks
    // set_sys_clock_khz(133 * 1000, true);
    clocks_init();
    // stdio_init_all();
    // vreg_set_voltage(VREG_VOLTAGE_DEFAULT);

    return;
}

void setup() {
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);  
  digitalWrite(PIN_LED_G, HIGH); 
  digitalWrite(PIN_LED_B, HIGH); 
  Wire.begin();
  // disable uart 0
  


  Serial.begin(9600);
  // while (!Serial)
     // delay(10);
  delay(1000);
  Serial.println("\nPiCowbell Proto Pico I2C Scanner");

  if (gps.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  
  Serial1.begin(9600);
  delay(5000);
  byte message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F, 0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75, 0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};

Serial1.write(message, sizeof(message));
  delay(1000);
  Serial1.println("AT+UART=BR, 9600");
  delay(1000);
  // read from serial1
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
    Serial.println("Finished LoRa connection \n\n\n");
  // delay(10000);
  gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  delay(1000);
  Serial1.println("AT+LOWPOWER=AUTOON");
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
    // uart_default_tx_wait_blocking();
  // pinMode(UART_TX, INPUT);
  // delay(100000);
  // Serial1.end();
  // delay(1000);
  // while (true) {
    // Serial.write(Serial1.read());
  // }
    Serial.println("Sleeping \n\n\n");
  // Serial1.end();
  delay(10000);
  // pinMode(UART_TX, OUTPUT);
  // Serial1.begin(9600);

Serial1.write(message, sizeof(message));
  delay(10);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  delay(1000);
  Serial1.println("AT");
  delay(10);
  // read from serial1
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
    Serial.println("awake\n");
}

void loop() {
  digitalWrite(PIN_LED_B, LOW); 
    delay(10000);
  digitalWrite(PIN_LED_B, HIGH); 
    delay(1000);
  Serial1.println("AT+LOWPOWER=AUTOON");

    //save values for later
    uint scb_orig = scb_hw->scr;
    uint clock0_orig = clocks_hw->sleep_en0;
    uint clock1_orig = clocks_hw->sleep_en1;

    datetime_t t = {
            .year  = 2020,
            .month = 06,
            .day   = 05,
            .dotw  = 5, // 0 is Sunday, so 5 is Friday
            .hour  = 15,
            .min   = 45,
            .sec   = 00
    };

    // Start the Real time clock
    rtc_init();


    uart_default_tx_wait_blocking();

    sleep_run_from_xosc();
    //Reset real time clock to a value
    rtc_set_datetime(&t);
    //sleep here, in this case for 1 min
    rtc_sleep(46,0);


    //reset processor and clocks back to defaults
    recover_from_sleep(scb_orig, clock0_orig, clock1_orig);
  byte message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F, 0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75, 0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};
Serial1.write(message, sizeof(message));
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
