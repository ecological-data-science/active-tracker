
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
  void archive_latest_message();
  void set_latest_message(location_reading location, activity_reading activities);
  void location_send_successful();
  void activity_send_successful();
  bool anything_to_send(bool nightmode);
  bool new_message = false;
  combined_reading get_current_reading();

private:
  const char *devNonce = "/counters/dev_nonce_counter.bin";
  const char *sendCounter = "/counters/send_counter.bin";
  const char *dataFile = "/data/telemetry.bin";
  uint8_t message_buffer[SIZE_OF_RECORD];
  combined_reading current_reading;

  int record_size = SIZE_OF_RECORD;

  uint32_t last_record_sent = 0; 


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
