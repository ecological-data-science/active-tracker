#pragma once

#include "debug.h"
#include "uart_settings.h"

#define ERR_NOT_JOINED -5
#define ERR_ALREADY_JOINED -6
#define MODEM_OK 1

class Lora {
public:
  bool begin();
  void wakeup();


private:
  void sendCommand(const char *atstring);
  void sendBytes(const uint8_t *msg, int len);
};
