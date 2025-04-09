#pragma once

#ifndef _LORA_H
#define _LORA_H
#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization
#include "storage.h"
#include <Arduino.h>

#define ERR_NOT_JOINED -5
#define ERR_ALREADY_JOINED -6
#define MODEM_OK 1

class Lora {
public:
  bool begin();
  bool update(Storage *storage);

  bool join();
  // bool start(location_reading message);
  bool send_message(LoraMessage message);

  bool session_success = false;
  void activate();
  void deactivate();

private:
  void sendQuery(String atstring);
  int sendCommand(String atstring);
  bool lora_active = false;
  bool join_success = false;
  bool location_sent = false;

  String WBEEST_APP_KEY = "434F4C494E5357494C44454245455354";
};

#endif
