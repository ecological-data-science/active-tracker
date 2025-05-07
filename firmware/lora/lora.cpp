

#include "lora.h"

bool Lora::begin(Storage *_storage) {

  storage = _storage;

  // pinMode(LORA_IRQ_DUMB, OUTPUT);
  // activate();
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
  // join();

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
    printf("Not actually joined");
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
    printf("lora run time exceeded, stopping lora\n");
    if (!activity_sent) {
      // if we have not sent a message then we need to archive the
      // latest message
      storage->archive_latest_message();
    }
    deactivate();
    return false;
  }

  if (attempt_counter == 0){
    printf("lora attempts exceeded, stopping lora\n");
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
    printf("no messages to send, stopping lora\n");
    deactivate();
    return false;
  }

  return true;
}

bool Lora::join() {

    bool joined = false;

    printf("attempting to join..");

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

    joined = true;
    return joined;
}

bool Lora::send_message(LoraMessage message) {

  watchdog_update();
    printf("sending message");
    //
    // Serial.println(message.getLength());
    bool message_sent = false;
    //
    // // location messages are length 12 and go to port 3
    // // activity messages are length 49 and go to port 5
    // if (message.getLength()==12)
    // {
    //   SerialLoRa.print("AT+PCTX 3,");
    // }
    // else
    // {
    //   SerialLoRa.print("AT+PCTX 5,");
    // }
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
  void Lora::sendQuery(const char* atstring) {
    //
    //   Serial.print("Sending query: ");
    //   Serial.println(atstring);
    //
    //   SerialLoRa.println(atstring);
    //   String answer;
    //   while (true)
    //   {
    //
    //     answer = SerialLoRa.readStringUntil('\r\n');
    //     if (answer.startsWith("+OK")){
    //       break;
    //     }
    //     if (answer.startsWith("+ERR")){
    //       break;
    //     }
    //   }
    //   Serial.print("Response: ");
    //   Serial.println(answer);
    //   delay(500);
  }

  int Lora::sendCommand(const char* atstring) {
    int modem_status = 0;

    // Serial.print("Sending command: ");
    // Serial.println(atstring);
    //
    // SerialLoRa.println(atstring);
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
    //     // get the error code
    //     modem_status = answer.substring(answer.indexOf("=")+1).toInt();
    //     break;
    //   }
    // }
    // Serial.print("Response: ");
    // Serial.println(answer);
    // delay(500);
    return modem_status;
  }
