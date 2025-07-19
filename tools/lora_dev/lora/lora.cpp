#include "lora.h"


bool Lora::begin() {

 
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  
  sleep_ms(500);
  sendCommand("AT+ID=DevEui");

  sleep_ms(500);
  sendCommand("AT+ID=AppEui");
  sleep_ms(500);
  sendCommand("AT+LW=VER");

  return true;
}

void Lora::sendCommand(const char *msg) {
  uint64_t timeout_us = 1000000;     // Overall timeout: 1 second
  uint64_t idle_timeout_us = 100000; // Stop after 100ms of no new data
  char responseBuffer[256] = {0};
  uint8_t bufferPos = 0;

  uart_puts(UART_ID, msg);
  uart_puts(UART_ID, "\r\n");

  uint64_t start_time = time_us_64(); // Record the starting time
  uint64_t last_read_time = start_time;

  while (time_us_64() - start_time < timeout_us) {
    bool got_data = false;

    while (uart_is_readable(UART_ID)) {
      uint8_t ch = uart_getc(UART_ID);
      got_data = true;

      // Store in buffer (prevent overflow)
      if (bufferPos < sizeof(responseBuffer) - 1) {
        responseBuffer[bufferPos++] = ch;
        responseBuffer[bufferPos] = '\0';
      }
    }

    if (got_data) {
      last_read_time = time_us_64();
    }
    // If we've received some data (buffer not empty) and had no new data for
    // idle_timeout_us, we can assume the response is complete
    else if (bufferPos > 0 &&
             (time_us_64() - last_read_time > idle_timeout_us)) {
      break;
    }

    // Small delay to prevent CPU hogging
    sleep_us(10);
  }

  // Print the collected response
  DEBUG_PRINT(("LoRa Response: %s", responseBuffer));
}

