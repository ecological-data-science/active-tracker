

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
    classifier.activate(gps.getUnixEpochfromRTC()); 
    watchdog_enable(0x2000, 0);

    GPS_ACTIVE = true;
    IMU_ACTIVE = true;
    start_time = get_absolute_time();
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
  watchdog_update();

  if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {

    watchdog_disable();
    sleep_until_next_hour_boundary(start_time);
    // wdt.setup(WDT_OFF);  //watchdog

    // lowPower.activate();
  }
}
void sleep_until_next_hour_boundary(absolute_time_t start_time) {
    // calculate the time since we started the GPS/IMU in milliseconds
    uint64_t elapsed_ms = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;

    // define the duration of one hour in milliseconds
    uint64_t one_hour_ms = 60ULL * 60ULL * 1000ULL; // Use ULL suffix for uint64_t literals

    // calculate how far into the current hour interval we are (relative to start_time)
    uint64_t time_into_current_hour_ms = elapsed_ms % one_hour_ms;

    // calculate how long until the next hour interval boundary
    uint64_t sleep_duration_ms = one_hour_ms - time_into_current_hour_ms;

    // If elapsed_ms is exactly a multiple of one_hour_ms, time_into_current_hour_ms is 0.
    // sleep_duration_ms becomes one_hour_ms, which correctly means sleeping for the next full hour.

    printf("Elapsed time relative to start: %llu ms\n", elapsed_ms);
    printf("Time into current hour interval: %llu ms\n", time_into_current_hour_ms);
    printf("Sleeping for %llu ms until next hour boundary relative to start_time\n", sleep_duration_ms);

    enter_low_power_mode_ms(sleep_duration_ms);
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
