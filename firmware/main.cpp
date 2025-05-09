#include "main.h"

GPS gps;
Storage storage;
Classifier classifier;
Lora lora;

bool GPS_ACTIVE = false;
bool IMU_ACTIVE = false;
bool LORA_ACTIVE = false;

int main() {


  if (!setup())
    while (true) {
      // setup failed - the watchdog will reset the board
      sleep_ms(1000);
    }

  // enter the main loop
  while (true)
    loop();
}

void core1_entry() {
  flash_safe_execute_core_init();
}

bool setup() {

  flash_safe_execute_core_init();
  multicore_launch_core1(core1_entry);

#if DEBUG
  stdio_init_all();
  sleep_ms(5000);
#endif

  // start the watchdog timer
  watchdog_enable(0x2000, 0);
  DEBUG_PRINT(("Staring setup\n"));

  // initialize I2C using the default pins
  i2c_init(i2c0, 400 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  DEBUG_PRINT(("Completed i2c setup\n"));

  watchdog_update();
  if (!storage.begin()) {
    DEBUG_PRINT(("storage initialization failed\n"));
    return false;
  }
  DEBUG_PRINT(("Completed storage setup\n"));

  watchdog_update();
  // initialize the GPS
  if (!gps.begin(i2c0)) {
    DEBUG_PRINT(("GPS initialization failed\n"));
    return false;
  }
  DEBUG_PRINT(("Completed gps setup\n"));

  watchdog_update();
  // initialize the classifier
  if (!classifier.begin(i2c0)) {
    DEBUG_PRINT(("classifier initialization failed\n"));
    return false;
  }
  DEBUG_PRINT(("Completed classifier setup\n"));


  watchdog_update();
  // initialize the lora communication
  if (!lora.begin(&storage)) {
    DEBUG_PRINT(("Lora initialization failed\n"));
    return false;
  }
  DEBUG_PRINT(("Completed lora setup\n"));

  // turn off all LEDs
  turn_off_all_leds();

  DEBUG_PRINT(("Setup complete\n"));

  return true;
}

void loop() {

  watchdog_update();
  if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {
    // if nothing active then we are at the top of the hour so wake up the gps and imu
    gps.activate();
    classifier.activate(gps.getUnixEpochfromRTC());

    GPS_ACTIVE = true;
    IMU_ACTIVE = true;
    start_time = get_absolute_time();
  }

  watchdog_update();
  if (GPS_ACTIVE)
    GPS_ACTIVE = gps.update();

  watchdog_update();
  if (IMU_ACTIVE)
    IMU_ACTIVE = classifier.update();

  watchdog_update();
  if ((!IMU_ACTIVE) && (!LORA_ACTIVE)) {
    storage.set_latest_message(gps.get_location(), classifier.get_activity());
    lora.activate(gps.getNightMode());
    LORA_ACTIVE = true;
  }

  watchdog_update();
  if (LORA_ACTIVE)
    LORA_ACTIVE = lora.update();


  watchdog_update();
  if ((!GPS_ACTIVE) && (!IMU_ACTIVE) && (!LORA_ACTIVE)) {

    watchdog_disable();
    sleep_until_next_hour_boundary(start_time);
    watchdog_enable(0x2000, 0);
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

  // If elapsed_ms is exactly a multiple of one_hour_ms, time_into_current_hour_ms is 0. sleep_duration_ms becomes one_hour_ms,
  // which correctly means sleeping for the next full hour.
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

  DEBUG_PRINT(("All LEDs turned off\n"));
}
