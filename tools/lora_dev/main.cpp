#include "main.h"

Lora lora;


int main() {






  stdio_init_all();
  sleep_ms(5000);


  // initialize I2C using the default pins
  i2c_init(i2c0, 400 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);





  // initialize the lora communication
  if (!lora.begin()) {
    DEBUG_PRINT(("Lora initialization failed"));
    return false;
  }
  DEBUG_PRINT(("Completed lora setup"));

  return true;
}



