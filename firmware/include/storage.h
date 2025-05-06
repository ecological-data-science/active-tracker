
#pragma once

#include <pico/stdlib.h>
#include <stdio.h>

#include "lfs.h"
#include "pico_lfs_hal.h"
#include "pico_lfs.h"

#include "data_structs.h"

class Storage {
public:
  bool begin();
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

  // LittleFS instance and configuration
  lfs_t lfs;
  lfs_file_t file;
  struct lfs_config cfg = {
      // block device operations
      .read = pico_read_flash_block,
      .prog = pico_prog_flash_block,
      .erase = pico_erase_flash_block,
      .sync = pico_sync_flash_block,

      // block device configuration
      .read_size = 1,
      .prog_size = PICO_PROG_PAGE_SIZE,
      .block_size = PICO_ERASE_PAGE_SIZE,
      .block_count = FLASHFS_BLOCK_COUNT,
      .block_cycles = 500,
      .cache_size = PICO_PROG_PAGE_SIZE * 1,
      .lookahead_size = 16,
  };
};
