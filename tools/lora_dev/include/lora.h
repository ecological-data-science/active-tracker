#pragma once

#include "debug.h"
#include "uart_settings.h"

#define ERR_NOT_JOINED -5
#define ERR_ALREADY_JOINED -6
#define MODEM_OK 1

class Lora {
public:
  bool begin();

private:
  void sendCommand(const char *atstring);

};
