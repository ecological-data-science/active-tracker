#pragma once

#include "debug.h"
#include "hardware/watchdog.h"
#include "storage.h"
#include "uart_settings.h"

#define ERR_NOT_JOINED -5
#define ERR_ALREADY_JOINED -6
#define MODEM_OK 1

class Lora {
public:
  bool begin(Storage *storage);
  bool update();
  void wakeup();
  bool join();
  bool session_success = false;
  void activate(bool _nightmode);
  void deactivate();

private:
  void sendCommand(const char *atstring);
  void sendBytes(const uint8_t *msg, int len);
  bool sendPayload(uint8_t *message, int len);
  void _intToBytes(uint8_t *buf, int32_t i, uint8_t byteSize);
  void _bufferToHex(const uint8_t *buffer, int len, char *hexstr);
  int attempt_counter;
  int max_attempts = 10;
  Storage *storage;
  bool lora_active = false;
  bool join_success = false;
  absolute_time_t lora_start_time;
  uint8_t location_buffer[SIZE_OF_LOCATION];
  int location_buffer_len = SIZE_OF_LOCATION;
  uint8_t activity_buffer[SIZE_OF_ACTIVITY];
  int activity_buffer_len = SIZE_OF_ACTIVITY;
  int _offset;

  // this is for the buffer to convert bytes to hex
  const int max_string_length = 2 * SIZE_OF_ACTIVITY + 1;

  uint32_t lora_run_time = 1000 * 60 * 20; // LoRa will run for up to 20 minutes

  bool nightmode = false;
  const char *WBEEST_APP_KEY = "434F4C494E5357494C44454245455354";

};
