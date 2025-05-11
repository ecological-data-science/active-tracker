

#include "lora.h"

void Lora::sendCommand(const char *msg)
{
    uint64_t timeout_us = 1000000;  // Timeout period in microseconds (1 second)
    
  watchdog_update();
    uart_puts(UART_ID, msg);
    uart_puts(UART_ID, "\r\n");
    
  watchdog_update();
    uint64_t start_time = time_us_64();  // Record the starting time
    while (time_us_64() - start_time < timeout_us)
    {
        while (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            putchar(ch);  // Output to USB/stdio
        }
    }
  watchdog_update();
}
void Lora::sendBytes(const uint8_t *msg, int len)
{
  uint64_t timeout_us = 1000000;  // Timeout period in microseconds (1 second)

  watchdog_update();
  uart_write_blocking(UART_ID, msg, len);
  
  watchdog_update();
  uint64_t start_time = time_us_64();  // Record the starting time
  while (time_us_64() - start_time < timeout_us)
    {
        while (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            putchar(ch);  // Output to USB/stdio
        }
    }
  watchdog_update();
}



void Lora::wakeup() {
  watchdog_update();
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);  // Switch back to UART
    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    // Re-initialize UART if needed
    uart_init(UART_ID, BAUD_RATE);
  watchdog_update();
  sleep_ms(1000);
  uint8_t message[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x61, 0x74, 0x2B, 0x6C, 0x6F,
                         0x77, 0x70, 0x6F, 0x77, 0x65, 0x72, 0x3D, 0x61, 0x75,
                         0x74, 0x6F, 0x6F, 0x66, 0x66, 0x0D, 0x0A};
  sendBytes(message, sizeof(message));
  // SerialLoRa.println("AT+WAKEUP");
  // delay(500);
  // SerialLoRa.flush();
  // delay(500);
}

void Lora::sleep() {
  // SerialLoRa.println("AT+SLEEP");
  // delay(500);
  // SerialLoRa.flush();
  // delay(500);
    sendCommand("AT+LOWPOWER=AUTOON");
    // uart_puts(UART_ID, "AT+LOWPOWER=AUTOON\r\n");
    // while (uart_is_readable(UART_ID)) {
    //     putchar(uart_getc(UART_ID));
    // }

    // // Set UART_TX to input mode for low power
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_SIO);  // Switch from UART to SIO (GPIO)
    gpio_set_dir(UART_TX_PIN, GPIO_IN);
}

bool Lora::begin(Storage *_storage) {

  storage = _storage;

    // Initialize UART1
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // TODO setup uart
    wakeup();
    sleep_ms(500);
    sendCommand("AT+ID=DevEui");
  printf("AT+ID=DevEui\n");

    sleep_ms(500);
    sendCommand("AT+ID=AppEui");
    
  // TODO set the device keys etc
  //
  // sendQuery("AT+VER?");
  // sendQuery("AT+DEVEUI?");
  // sendQuery("AT+APPEUI?");
  //
  // sendCommand(String("AT+APPKEY=") + WBEEST_APP_KEY); // set appkey
  // sendQuery("AT+APPKEY?");
  //
  //
  //
  // sendCommand("AT+MODE=1");
  // sendCommand("AT+RTYNUM=8");
  // sendCommand("AT+DELAY=5000,6000,5000,6000");
  // sendCommand("AT+DUTYCYCLE=0");
  //
  //
  // sendQuery("AT+RTYNUM?");
  // sendQuery("AT+DELAY?");
  //
  join();

  sleep();
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
    join_success = join();
    if (!join_success)
      return false;
  }

  combined_reading current_reading = storage->get_current_reading();

  LoraMessage l_message;
  l_message.addUnixtime(current_reading.location.start_time);
  l_message.addLatLng(current_reading.location.lat,
                      current_reading.location.lon);

  bool activity_sent = false;
  bool location_sent = false;

  if (current_reading.partially_sent)
    location_sent = true;
  else
    location_sent = send_message(l_message);

  if (!join_success) {
      DEBUG_PRINT(("Not actually joined"));
    // if we get here then we thought we were joined but we are not
    // we return turn to keep lora active and in the next iteration
    // we will try to join again
    return true;
  }

  if (location_sent) {
    if (!current_reading.partially_sent)
    {
      attempt_counter = max_attempts;
      storage->location_send_successful();
    }
    LoraMessage a_message;

    a_message.addUnixtime(current_reading.activity.start_time);
    for (int i = 0; i < 45; i++)
      a_message.addUint8(current_reading.activity.activities[i]);

    activity_sent = send_message(a_message);
  }

  if (activity_sent) {
    attempt_counter = max_attempts;
    storage->activity_send_successful();
  } else {
    // if we get here then we have not sent a message
    // but we have successfully sent one this session
    attempt_counter--;
  }

  if (absolute_time_diff_us(lora_start_time, get_absolute_time()) >=
      lora_run_time * 1000) {
    DEBUG_PRINT(("lora run time exceeded, stopping lora\n"));
    if (!activity_sent) {
      // if we have not sent a message then we need to archive the
      // latest message
      storage->archive_latest_message();
    }
    deactivate();
    return false;
  }

  if (attempt_counter == 0){
    DEBUG_PRINT(("lora attempts exceeded, stopping lora\n"));
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
    DEBUG_PRINT(("no messages to send, stopping lora\n"));
    deactivate();
    return false;
  }

  return true;
}

bool Lora::join() {

    bool joined = false;

    DEBUG_PRINT(("attempting to join.."));

    // int modem_status = sendCommand("AT+JOIN");
    // if (modem_status==MODEM_OK) // join request sent
    // {
    //
    //   long lora_start_time = millis();
    //   long lora_timeout = 60*1000*5;  // break after 5 minutes
    //   String modem_ans;
    //   while (true)
    //   {
    //
    //     modem_ans = SerialLoRa.readStringUntil('\r\n');
    //
    //     if (modem_ans.startsWith("+EVENT=1,0")){
    //         Serial.println("join failed");
    //         break;
    //     }
    //     if (modem_ans.startsWith("+EVENT=1,1")){
    //         Serial.println("join succeeded");
    //         joined = true;
    //         break;
    //     }
    //     if (millis() - lora_start_time > lora_timeout){
    //       break;
    //     }
    //   }
    // }
    //
    // if (modem_status==ERR_ALREADY_JOINED) // already joined
    //   joined = true;
    //

    // TODO: implement join code
    joined = true;
    return joined;
}

bool Lora::send_message(LoraMessage message) {

    watchdog_update();
    DEBUG_PRINT(("sending message"));
    //
    // Serial.println(message.getLength());
    bool message_sent = false;

    // location messages are length 12 and go to port 3
    // activity messages are length 49 and go to port 5
  combined_reading current_reading = storage->get_current_reading(); // TODO remove and replace with lora send logic
    if (message.getLength()==12)
    {

      DEBUG_PRINT(("sending location message: time %lu, lat %f, lon %f\n",
                   current_reading.location.start_time, current_reading.location.lat, current_reading.location.lon));
    //   SerialLoRa.print("AT+PCTX 3,");
    }
    else
    {
      DEBUG_PRINT(("sending activity message: time %lu, activity %d\n",
                   current_reading.activity.start_time, current_reading.activity.activities[0]));
    //   SerialLoRa.print("AT+PCTX 5,");
    }
    // SerialLoRa.print(message.getLength());
    // SerialLoRa.print("\r");
    // SerialLoRa.write(message.getBytes(),message.getLength());
    //
    // int modem_status = 0;
    //
    // String answer;
    // while (true)
    // {
    //
    //   answer = SerialLoRa.readStringUntil('\r\n');
    //   if (answer.startsWith("+OK")){
    //     modem_status=MODEM_OK;
    //     break;
    //   }
    //   if (answer.startsWith("+ERR")){
    //     modem_status = answer.substring(answer.indexOf("=")+1).toInt();
    //     break;
    //   }
    // }
    //
    // if (modem_status==MODEM_OK)
    // {
    //   long lora_start_time = millis();
    //   long lora_timeout = 60*1000*2;  // break after 2 minutes
    //   String modem_ans;
    //   while (true)
    //   {
    //
    //     modem_ans = SerialLoRa.readStringUntil('\r\n');
    //     if (modem_ans.startsWith("+NOACK")){
    //         Serial.println("no ack recv");
    //         message_sent = false;
    //
    //         break;
    //     }
    //     if (modem_ans.startsWith("+ACK")){
    //         Serial.println("ack recv");
    //         message_sent = true;
    //         break;
    //     }
    //     if (millis() - lora_start_time > lora_timeout){
    //       break;
    //     }
    //   }
    // }
    //
    //
    // if (modem_status==ERR_NOT_JOINED) // not joined
    //   join_success = false;

    message_sent = true;
    return message_sent;
  }

  void Lora::activate(bool _nightmode) {

    attempt_counter = max_attempts;

    lora_start_time = get_absolute_time();
    nightmode = _nightmode;
    wakeup();
    sendCommand("AT");
    //   if (lora_active==true)
    //     return;
    //   SerialLoRa.begin(19200);
    //   long lora_start_time = millis();
    //   long lora_timeout = 10000;
    //   while(!SerialLoRa){
    //     if (millis() - lora_start_time > lora_timeout)
    //       return;
    //   }
    //
    //   digitalWrite(LORA_IRQ_DUMB, LOW);
    //   lora_active=true;
    //   delay(500);
    //   return;
  }
    //
     void Lora::deactivate() {
        sleep();
    //
    //   if (lora_active==false)
    //     return;
    //
    //   digitalWrite(LORA_IRQ_DUMB, HIGH);
    //
    //   delay(500);
    //   sendCommand("AT$DETACH"); // request UART to disconnect
    //
    //   lora_active=false;
    //   delay(500);
    //
    //   return;
     }
    //
  // void Lora::sendQuery(const char* atstring) {
  //   //
  //   //   Serial.print("Sending query: ");
  //   //   Serial.println(atstring);
  //   //
  //   //   SerialLoRa.println(atstring);
  //   //   String answer;
  //   //   while (true)
  //   //   {
  //   //
  //   //     answer = SerialLoRa.readStringUntil('\r\n');
  //   //     if (answer.startsWith("+OK")){
  //   //       break;
  //   //     }
  //   //     if (answer.startsWith("+ERR")){
  //   //       break;
  //   //     }
  //   //   }
  //   //   Serial.print("Response: ");
  //   //   Serial.println(answer);
  //   //   delay(500);
  // }

  // int Lora::sendCommand(const char* atstring) {
  //   int modem_status = 0;
  //
  //   // Serial.print("Sending command: ");
  //   // Serial.println(atstring);
  //   //
  //   // SerialLoRa.println(atstring);
  //   //
  //   // String answer;
  //   // while (true)
  //   // {
  //   //
  //   //   answer = SerialLoRa.readStringUntil('\r\n');
  //   //   if (answer.startsWith("+OK")){
  //   //     modem_status=MODEM_OK;
  //   //     break;
  //   //   }
  //   //   if (answer.startsWith("+ERR")){
  //   //     // get the error code
  //   //     modem_status = answer.substring(answer.indexOf("=")+1).toInt();
  //   //     break;
  //   //   }
  //   // }
  //   // Serial.print("Response: ");
  //   // Serial.println(answer);
  //   // delay(500);
  //   return modem_status;
  // }
