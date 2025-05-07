#pragma once

#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization
#include "storage.h"

#define ERR_NOT_JOINED -5
#define ERR_ALREADY_JOINED -6
#define MODEM_OK 1

class Lora {
public:
  bool begin(Storage *storage);
  bool update();

  bool join();
  // bool start(location_reading message);
  bool send_message(LoraMessage message);

  bool session_success = false;
  void activate();
  void deactivate();

private:
  void sendQuery(String atstring);
  int sendCommand(String atstring);
  int attempt_counter;
  int max_attempts = 10;
  Storage *storage;
  bool lora_active = false;
  bool join_success = false;
  absolute_time_t lora_start_time;

  uint32_t lora_run_time = 1000 * 60 * 20; // LoRa will run for up to 20 minutes

  bool nightmode = false;
  String WBEEST_APP_KEY = "434F4C494E5357494C44454245455354";
};

