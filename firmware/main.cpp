

#include "main.h"

// TODO: implement the following classes
GPS gps;
Storage storage;
Classifier classifier;
Lora lora;

bool GPS_ACTIVE = false;
bool IMU_ACTIVE = false;
bool LORA_ACTIVE = false;

// location_reading latest_location;

int main() {
  // begin the watchdog timer
  bool success = setup();
  if (!success) {
    printf("Setup failed\n");
    // sleep for 1 minute to allow the watchdog to reset
    sleep_ms(60 * 1000);
  }
  while (true) {
    main_loop();
  }
}

bool setup() {
  stdio_init_all();
  sleep_ms(5000);
  printf("Starting setup ...\n");

  // initialize I2C using the default pins
  i2c_init(i2c0, 400 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // initialize the GPS
  if (!gps.begin(i2c0)) {
    printf("GPS initialization failed\n");
    return false;
  }

  // initialize the classifier
  if (!classifier.begin(i2c0)) {
    printf("classifier initialization failed\n");
    return false;
  }

  if (!storage.begin()) {
    printf("storage initialization failed\n");
    return false;
  }


  // initialize the lora communication
  if (!lora.begin(&storage)) {
    printf("Lora initialization failed\n");
    return false;
  }

  // flash the LED to indicate success

  // turn off all LEDs
  turn_off_all_leds();

  printf("Setup complete\n");

  return true;
}

void main_loop() {

  printf("going to sleep for 10 seconds\n");
  enter_low_power_mode_ms(10000);
  if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {

    // // if nothing active then we are at the top of the hour so wake up the
    // gps and imu and start the watchdog 
    gps.activate(); 
    classifier.activate(100); // TODO - add the startup time to the classifier
    // wdt.start();

    GPS_ACTIVE = true;
    IMU_ACTIVE = true;
  }

  if (GPS_ACTIVE)
    GPS_ACTIVE = gps.update();

  if (IMU_ACTIVE)
    IMU_ACTIVE = classifier.update();

  if ((!IMU_ACTIVE) && (!LORA_ACTIVE)) {
    storage.set_latest_message(gps.get_location(), classifier.get_activity());
    lora.activate(gps.getNightMode());
    LORA_ACTIVE = true;
  }

  if (LORA_ACTIVE)
     LORA_ACTIVE = lora.update();

  // wdt.clear();

  if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {

    // wdt.setup(WDT_OFF);  //watchdog

    // lowPower.activate();
  }
}

static void turn_off_all_leds() {
  // Initialize the RGB LED pins as outputs
  gpio_init(18);
  gpio_init(19);
  gpio_init(20);

  // Set them as outputs
  gpio_set_dir(18, GPIO_OUT);
  gpio_set_dir(19, GPIO_OUT);
  gpio_set_dir(20, GPIO_OUT);

  // Turn off all LEDs (they are active low, so set them high to turn off)
  gpio_put(18, 1);
  gpio_put(19, 1);
  gpio_put(20, 1);

  printf("All LEDs turned off\n");
}
