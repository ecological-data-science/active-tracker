#include "lora.h"

static const char join_failed_msg[] = "+JOIN: Join failed";
static const char join_succeeded_msg[] = "+JOIN: NetID";
static const char join_attempt_complete_msg[] = "+JOIN: Done";
static const char already_joined_msg[] = "+JOIN: Joined already";
static const char not_joined_msg[] = "+CMSGHEX: Please join network first";
static const char send_msg_complete[] = "+CMSGHEX: Done";
static const char send_msg_ack[] = "+CMSGHEX: ACK Received";

bool Lora::begin(Storage *_storage) {

  storage = _storage;
 
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  
  wakeup();
  sleep_ms(500);
  sendCommand("AT+ID=DevEui");

  sleep_ms(500);
  sendCommand("AT+ID=AppEui");
  sendCommand("AT+MODE=LWOTAA");
  sendCommand("AT+DELAY=RX1,5000");
  sendCommand("AT+DELAY=RX2,6000");

  char key_message[128];  // Make sure this buffer is large enough to hold the concatenated message
  sprintf(key_message, "AT+KEY=APPKEY,%s", WBEEST_APP_KEY);
  sendCommand(key_message);  // Send the concatenated message
   
  sleep_ms(500);
  join_success = attempt_joins();

  deactivate();
  return true;
}

bool Lora::update() {

  // attempts to send a message and returns false if the lora comms
  // should be stopped. If true it will keep trying to send messages
  // Reasons to return true:
  // 1. we have sent a message and there's messages in the history
  // 2. we have not sent a message but we have successfully sent one this
  // session
  // 3. we have not sent a message because we are not joined

  if (!join_success) {
    join_success = attempt_joins();
    if (!join_success){
      storage->archive_latest_message();
      deactivate();
      return false;
    }
  }

  combined_reading current_reading = storage->get_current_reading();

  bool activity_sent = false;
  bool location_sent = false;

  if (current_reading.partially_sent)
    location_sent = true;
  else {
    _offset = 0;
    _intToBytes(location_buffer + _offset, current_reading.location.start_time,
                4);
    _offset += 4;
    int32_t lat = current_reading.location.lat * 1e6;
    int32_t lng = current_reading.location.lon * 1e6;
    _intToBytes(location_buffer + _offset, lat, 4);
    _offset += 4;
    _intToBytes(location_buffer + _offset, lng, 4);
    location_sent = sendPayload(location_buffer, location_buffer_len);
  }

  if (!join_success) {
    DEBUG_PRINT(("Not actually joined"));
    // if we get here then we thought we were joined but we are not
    // we return turn to keep lora active and in the next iteration
    // we will try to join again
    return true;
  }

  if (location_sent) {
    DEBUG_PRINT(("location sent successfully"));
    if (!current_reading.partially_sent) {
      attempt_counter = max_attempts;
      storage->location_send_successful();
    }

    _offset = 0;
    _intToBytes(activity_buffer + _offset, current_reading.activity.start_time,
                4);
    _offset += 4;
    for (int i = 0; i < NUM_CLASSIFICATIONS; i++) {
      _intToBytes(activity_buffer + _offset,
                  current_reading.activity.activities[i], 1);
      _offset += 1;
    }
    activity_sent = sendPayload(activity_buffer, activity_buffer_len);
  }

  DEBUG_PRINT(("activity sending attempted"));
  if (activity_sent) {
    DEBUG_PRINT(("activity sent successfully"));
    attempt_counter = max_attempts;
    storage->activity_send_successful();
  } else {
    DEBUG_PRINT(("activity send failed"));
    // if we get here then we have not sent a message
    // but we have successfully sent one this session
    attempt_counter--;
  }

  if (absolute_time_diff_us(lora_start_time, get_absolute_time()) >=
      lora_run_time * 1000) {
    DEBUG_PRINT(("lora run time exceeded, stopping lora"));
    if (!activity_sent) {
      // if we have not sent a message then we need to archive the
      // latest message
      storage->archive_latest_message();
    }
    deactivate();
    return false;
  }

  if (attempt_counter == 0) {
    DEBUG_PRINT(("lora attempts exceeded, stopping lora"));
    if (!activity_sent) {
      // if we have not sent a message then we need to archive the
      // latest message
      storage->archive_latest_message();
    }
    deactivate();
    return false;
  }

  bool send_needed = storage->anything_to_send(nightmode);

  if (!send_needed) {
    DEBUG_PRINT(("no messages to send, stopping lora"));
    deactivate();
    return false;
  }

  return true;
}


void Lora::sendCommand(const char *msg) {
  uint64_t timeout_us = 1000000;     // Overall timeout: 1 second
  uint64_t idle_timeout_us = 100000; // Stop after 100ms of no new data
  char responseBuffer[256] = {0};
  uint8_t bufferPos = 0;

  watchdog_update();
  uart_puts(UART_ID, msg);
  uart_puts(UART_ID, "\r\n");

  watchdog_update();
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
  watchdog_update();
}

void Lora::sendBytes(const uint8_t *msg, int len) {
  uint64_t timeout_us = 1000000;     // Overall timeout: 1 second
  uint64_t idle_timeout_us = 100000; // Stop after 100ms of no new data
  char responseBuffer[256] = {0};
  uint8_t bufferPos = 0;

  watchdog_update();
  uart_write_blocking(UART_ID, msg, len);

  watchdog_update();
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
  watchdog_update();
}

void Lora::wakeup() {
  watchdog_update();
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); // Switch back to UART
  gpio_set_dir(UART_TX_PIN, GPIO_OUT);
  // Re-initialize UART if needed
  uart_init(UART_ID, BAUD_RATE);
  watchdog_update();
  sleep_ms(1000);
  uint8_t message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F,
                       0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75,
                       0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};
  sendBytes(message, sizeof(message));
}


bool Lora::attempt_joins() {
    if (join()) {
        DEBUG_PRINT(("LoRa join succeeded."));
        return true;
    }
    for (int attempt = 1; attempt < max_join_attempts; attempt++) {
        

        DEBUG_PRINT(("Join failed, waiting 8 seconds before next attempt..."));
            
        // Sleep 8 seconds in small increments to keep watchdog alive
        const uint32_t pause_us = 8 * 1000 * 1000;
        const uint32_t step_us = 100 * 1000; // 100ms steps
        uint32_t waited_us = 0;
        while (waited_us < pause_us) {
            sleep_us(step_us);
            watchdog_update();
            waited_us += step_us;
        }
        DEBUG_PRINT(("Join attempt %d of %d", attempt+1, max_join_attempts));
        if (join()) {
            DEBUG_PRINT(("LoRa join succeeded on attempt %d", attempt + 1));
            return true;
        }
    }
    DEBUG_PRINT(("All join attempts failed"));
    return false;
}

bool Lora::join() {

  bool joined = false;

  DEBUG_PRINT(("attempting to join.."));

  uint64_t timeout_us = 100000000; // Overall timeout: 100 seconds
  char responseBuffer[256] = {0};
  uint8_t bufferPos = 0;

  watchdog_update();

  uart_puts(UART_ID, "AT+JOIN\r\n");

  watchdog_update();
  uint64_t start_time = time_us_64(); // Record the starting time

  while (time_us_64() - start_time < timeout_us) {

    while (uart_is_readable(UART_ID)) {
      uint8_t ch = uart_getc(UART_ID);
      if (ch == '\r') {
        continue;
      }
      if (ch == '\n') {
        // message is finished so print and clear the buffer
        DEBUG_PRINT(("LoRa Response: %s", responseBuffer));
        if (strstr(responseBuffer, join_failed_msg) != NULL) {
          DEBUG_PRINT(("Join failed"));
          joined = false;
        } else if (strstr(responseBuffer, join_succeeded_msg) != NULL) {
          DEBUG_PRINT(("Join succeeded"));
          joined = true;
        } else if (strstr(responseBuffer, already_joined_msg) != NULL) {
          DEBUG_PRINT(("Already joined"));
          joined = true;
        } else if (strstr(responseBuffer, join_attempt_complete_msg) != NULL) {
          DEBUG_PRINT(("Join attempt complete"));
          return joined;
        }

        responseBuffer[0] = '\0'; // Clear the buffer
        bufferPos = 0;            // Reset the buffer position
      }

      // Store in buffer (prevent overflow)
      if (bufferPos < sizeof(responseBuffer) - 1) {
        responseBuffer[bufferPos++] = ch;
        responseBuffer[bufferPos] = '\0';
      }
    }

    // Small delay to prevent CPU hogging
    sleep_us(10);
    watchdog_update();
  }

  return joined;
}

bool Lora::sendPayload(uint8_t *message, int len) {

  watchdog_update();
  DEBUG_PRINT(("sending message"));

  bool message_sent = false;
  uint64_t timeout_us = 100000000; // Overall timeout: 100 seconds

  // location messages are length 12 and go to port 3
  // activity messages are length 49 and go to port 5
  if (len == 12) {

    DEBUG_PRINT(("sending location message"));
    sendCommand("AT+PORT=3");
  } else {
    DEBUG_PRINT(("sending activity message"));
    sendCommand("AT+PORT=5");
  }

  // build the message - first create the hex string
  char hexstr[max_string_length];
  _bufferToHex(message, len, hexstr);


  // next create the AT command
  char at_msg[max_string_length + 32];
  snprintf(at_msg, sizeof(at_msg), "AT+CMSGHEX=\"%s\"\r\n", hexstr);

  // prepare the response buffer
  char responseBuffer[256] = {0};
  uint8_t bufferPos = 0;

  watchdog_update();

  // send the message
  uart_puts(UART_ID, at_msg);

  uint64_t start_time = time_us_64(); // Record the starting time

  watchdog_update();
  

  while (time_us_64() - start_time < timeout_us) {

    while (uart_is_readable(UART_ID)) {
      uint8_t ch = uart_getc(UART_ID);
      if (ch == '\r') {
        continue;
      }
      if (ch == '\n') {
        // message is finished so print and clear the buffer
        DEBUG_PRINT(("LoRa Response: %s", responseBuffer));
        if (strstr(responseBuffer, not_joined_msg) != NULL) {
          DEBUG_PRINT(("Not joined"));
          join_success = false;
        } else if (strstr(responseBuffer, send_msg_ack) != NULL) {
          DEBUG_PRINT(("Send succeeded"));
          message_sent = true;
        } else if (strstr(responseBuffer, send_msg_complete) != NULL) {
          DEBUG_PRINT(("Send attempt complete"));
          return message_sent;
        }

        responseBuffer[0] = '\0'; // Clear the buffer
        bufferPos = 0;            // Reset the buffer position
      }

      // Store in buffer (prevent overflow)
      if (bufferPos < sizeof(responseBuffer) - 1) {
        responseBuffer[bufferPos++] = ch;
        responseBuffer[bufferPos] = '\0';
      }
    }

    // Small delay to prevent CPU hogging
    sleep_us(10);
    watchdog_update();
  }

  return message_sent;
}

void Lora::activate(bool _nightmode) {

  attempt_counter = max_attempts;

  lora_start_time = get_absolute_time();
  nightmode = _nightmode;
  wakeup();
  sendCommand("AT");
}

void Lora::deactivate() {
  sendCommand("AT+LOWPOWER=AUTOON");

  // Set UART_TX to input mode for low power
  gpio_set_function(UART_TX_PIN,
                    GPIO_FUNC_SIO); // Switch from UART to SIO (GPIO)
  gpio_set_dir(UART_TX_PIN, GPIO_IN);
}

void Lora::_intToBytes(uint8_t *buf, int32_t i, uint8_t byteSize) {
  for (uint8_t x = 0; x < byteSize; x++) {
    buf[x] = (uint8_t)(i >> (x * 8));
  }
}

void Lora::_bufferToHex(const uint8_t* buffer, int len, char* hexstr) {
    for (int i = 0; i < len; ++i) {
        sprintf(&hexstr[i*2], "%02X", buffer[i]);
    }
    hexstr[len*2] = '\0';
}

