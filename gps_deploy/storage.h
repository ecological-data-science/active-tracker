/**************************************************************************/


/**************************************************************************/

#ifndef _STORAGE_H
#define _STORAGE_H
#include <Arduino.h>
#include "gps_deploy.h"
#include <LittleFS.h>
#include "LoraMessage.h" //https://github.com/thesolarnomad/lora-serialization


class Storage {
public:
  
  void begin();
  void sleep();
  LoraMessage read_next_message();
  void write_next_message(location_reading location);
  void send_successful();
  bool anything_to_send();

  uint8_t message_buffer[12];
  

private:

  const char *devNonce = "/counters/dev_nonce_counter.bin";
  const char *sendCounter = "/counters/send_counter.bin";
  const char *dataFile = "/data/telemetry.bin";
  int size_of_location = 12; 
  
  int record_size = 12;

  int last_record_sent = 0;

  uint16_t currentSectorIndex;
  uint16_t currentRecordIndex; 

};
/**************************************************************************/

#endif
