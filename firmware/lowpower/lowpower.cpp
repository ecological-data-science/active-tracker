
#include "lowpower.h"

    
    
bool awake = true;

void enter_low_power_mode_ms(uint32_t sleep_time)
{
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);  
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    sleep_ms(100);
    uart_default_tx_wait_blocking();
    // Set the crystal oscillator as the dormant clock source, UART will be
    // reconfigured from here This is only really necessary before sending the
    // pico dormant but running from xosc while asleep saves power
    sleep_run_from_xosc();
    awake = false;
    // Go to sleep until the alarm interrupt is generated after 10 seconds
    uart_default_tx_wait_blocking();
    if (sleep_goto_sleep_for(sleep_time, &alarm_sleep_callback)) {
      // Make sure we don't wake
      while (!awake) {
        DEBUG_PRINT(("LowPower: Should be sleeping\n"));
      }
    }
    // Re-enabling clock sources and generators.
    sleep_power_up();

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);  // Switch back to UART
    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    uart_init(UART_ID, BAUD_RATE);
}



static void alarm_sleep_callback(uint alarm_id) {
  uart_default_tx_wait_blocking();
  awake = true;
  hardware_alarm_set_callback(alarm_id, NULL);
  hardware_alarm_unclaim(alarm_id);
}


