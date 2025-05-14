/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define PICO_DEFAULT_I2C 0
#define PICO_DEFAULT_I2C_SDA_PIN 4
#define PICO_DEFAULT_I2C_SCL_PIN 5

// define a digital pin capable of driving HIGH and LOW
#define WAKEUP_PIN 7
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/sleep.h"
#include "hardware/clocks.h"

#include "lib/gps_pico.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0u
#define UART_RX_PIN 1u



DevUBLOXGNSS gps;

static bool awake;
void sendmsg(const char *msg)
{
    uint64_t timeout_us = 1000000;  // Timeout period in microseconds (1 second)
    
    uart_puts(UART_ID, msg);
    uart_puts(UART_ID, "\r\n");
    
    uint64_t start_time = time_us_64();  // Record the starting time
    while (time_us_64() - start_time < timeout_us)
    {
        while (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            // putchar(ch);  // Output to USB/stdio
        }
    }
}


void sendbytes(const uint8_t *msg, int len)
{
    uint64_t timeout_us = 1000000;  // Timeout period in microseconds (1 second)

  uart_write_blocking(UART_ID, msg, len);
  
    uint64_t start_time = time_us_64();  // Record the starting time
    while (time_us_64() - start_time < timeout_us)
    {
        while (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            // putchar(ch);  // Output to USB/stdio
        }
    }
}

void wake_lora() {
  // printf("-- waking up Lora module \n" );

    uint8_t message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F,
                         0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75,
                         0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};
  sendbytes(message, sizeof(message));
}
void wake_gps() {
  // printf("-- waking up GPS module via pin %d on your microcontroller --\n", WAKEUP_PIN);

  gpio_put(WAKEUP_PIN, 0);  // Set LOW
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 1);  // Set HIGH
  sleep_ms(1000);
  gpio_put(WAKEUP_PIN, 0);  // Set LOW
}
// Function to turn off all LEDs on the Pimoroni Tiny RP2040
// The Tiny RP2040 has an RGB LED connected to GPIO 18 (red), 19 (green), and 20
// (blue)
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

  // printf("All LEDs turned off\n");
}

static void alarm_sleep_callback(uint alarm_id) {
  // printf("alarm woke us up\n");
  uart_default_tx_wait_blocking();
  awake = true;
  hardware_alarm_set_callback(alarm_id, NULL);
  hardware_alarm_unclaim(alarm_id);
}

int main() {

set_sys_clock_48mhz();
  sleep_ms(1000);
    stdio_init_all();
    sleep_ms(5000);
    // printf("Hello, Griffin!\n");
    sleep_ms(1000);

    // Initialize UART1
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
  wake_lora();
  // printf("UART initialized\n");
    sleep_ms(1000);
    sendmsg("AT+ID=DevEui");
  // printf("AT+ID=DevEui\n");

    sleep_ms(1000);
    sendmsg("AT+ID=AppEui");
    sleep_ms(1000);
    
    sendmsg("AT");
    
  // // stdio_init_all();
  //   // uart_puts(UART_ID, "AT+LOWPOWER=AUTOON\r\n");
  // // sleep_ms(1000*10);
  // sendmsg("AT+LOWPOWER=AUTOON");
  // sleep_ms(1000 * 60);
  // wake_lora();
  //   sendmsg("AT");
  // sleep_ms(1000 * 60);

  gpio_init(WAKEUP_PIN);
  gpio_set_dir(WAKEUP_PIN, GPIO_OUT);
  gpio_put(WAKEUP_PIN, 0);
  
  // printf("Hello Alarm Sleep!\n");
  sleep_ms(5000);

  // Initialize I2C using the default pins
  i2c_init(i2c0, 400 * 1000); // 100 kHz

  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  sleep_ms(1000);

  // printf("Starting GPS ...\n");
  sleep_ms(1000);
  wake_gps();
  // Now initialize the GPS
  if (gps.begin(i2c0) == false) {
    // printf("GPS module not detected, check wiring\n");
    while (1) {
      sleep_ms(100);
    }
  }

  // printf("GPS module initialized successfully\n");

    // // Initialize UART1
    // uart_init(UART_ID, BAUD_RATE);
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
  turn_off_all_leds();

  do {
    // printf("Awake for 10 seconds\n");
    sleep_ms(1000 * 10);

    // printf("Sleeping for 10 seconds\n");

    gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
    sendmsg("AT+LOWPOWER=AUTOON");
    // uart_puts(UART_ID, "AT+LOWPOWER=AUTOON\r\n");
    // while (uart_is_readable(UART_ID)) {
    //     putchar(uart_getc(UART_ID));
    // }

    // // Set UART_TX to input mode for low power
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);  // Switch from UART to SIO (GPIO)
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
    // printf("Switching to XOSC\n");
    sleep_ms(1000*10);

    // Wait for the fifo to be drained so we get reliable output
    uart_default_tx_wait_blocking();
    // Set the crystal oscillator as the dormant clock source, UART will be
    // reconfigured from here This is only really necessary before sending the
    // pico dormant but running from xosc while asleep saves power


set_sys_clock_khz(125000, true);
    sleep_run_from_xosc();
    awake = false;
    // Go to sleep until the alarm interrupt is generated after 10 seconds
    uart_default_tx_wait_blocking();
    if (sleep_goto_sleep_for(30000, &alarm_sleep_callback)) {
      // Make sure we don't wake
      while (!awake) {
        // printf("Should be sleeping\n");
      }
    }
    // Re-enabling clock sources and generators.
    sleep_power_up();

    wake_gps();
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);  // Switch back to UART
    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    // Re-initialize UART if needed
    uart_init(UART_ID, BAUD_RATE);
    wake_lora();
    sendmsg("AT");
    // uint8_t message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F,
    //                      0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75,
    //                      0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};
    // uart_write_blocking(UART_ID, message, sizeof(message));
    // sleep_ms(10);
    // while (uart_is_readable(UART_ID)) {
    //     putchar(uart_getc(UART_ID));
    // }
    // sleep_ms(1000);
    // uart_puts(UART_ID, "AT\r\n");
    // sleep_ms(10);
    // // Read from UART1
    // while (uart_is_readable(UART_ID)) {
    //     putchar(uart_getc(UART_ID));
    // }

  } while (true);

  return 0;
}
