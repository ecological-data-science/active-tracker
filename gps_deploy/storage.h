#include <cstdint>
/**************************************************************************/

/**************************************************************************/

#ifndef _STORAGE_H
#define _STORAGE_H
#include "gps_deploy.h"
#include <Arduino.h>
#include <LittleFS.h>

class Storage {
public:
  bool begin();
  void sleep();
  void set_latest_message(location_reading location);
  void store_latest_message();
  void send_successful();
  bool anything_to_send();
  bool pending_archive = false;
  uint8_t message_buffer[12];

private:
  const char *devNonce = "/counters/dev_nonce_counter.bin";
  const char *sendCounter = "/counters/send_counter.bin";
  const char *dataFile = "/data/telemetry.bin";
  int size_of_location = 12;

  int record_size = 12;

  uint8_t last_record_sent = 0;

  uint16_t currentSectorIndex;
  uint16_t currentRecordIndex;
};
/**************************************************************************/

#endif
