#include "storage.h"


bool Storage::begin() {

  // Initialize LittleFS
  int err = lfs_mount(&lfs, &cfg);

  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err) {
    lfs_format(&lfs, &cfg);
    err = lfs_mount(&lfs, &cfg);
  }
  if (err) {
    printf("Failed to mount LittleFS");
    return false;
  }

  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_RDONLY) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (lfs_file_open(&lfs, &file, dataFile, LFS_O_WRONLY | LFS_O_CREAT) !=
        LFS_ERR_OK) {
      printf("Failed to open file for writing");
      return false;
    }
    lfs_file_close(&lfs, &file);
  } else {
    lfs_file_close(&lfs, &file);
  }

  if (lfs_file_open(&lfs, &file, devNonce, LFS_O_RDONLY) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (lfs_file_open(&lfs, &file, devNonce, LFS_O_WRONLY | LFS_O_CREAT) !=
        LFS_ERR_OK) {
      printf("Failed to open file for writing");
      return false;
    }
    uint8_t dev_nonce = 0;
    lfs_file_write(&lfs, &file, &dev_nonce, sizeof(dev_nonce));
    lfs_file_close(&lfs, &file);
  } else {
    lfs_file_close(&lfs, &file);
  }

  if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_RDONLY) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_WRONLY | LFS_O_CREAT) !=
        LFS_ERR_OK) {
      printf("Failed to open file for writing");
      return false;
    }
    uint8_t send_counter = 0;
    lfs_file_write(&lfs, &file, &send_counter, sizeof(send_counter));
    lfs_file_close(&lfs, &file);
  } else {
    // Read the send counter
    if (lfs_file_read(&lfs, &file, &last_record_sent,
                      sizeof(last_record_sent)) != sizeof(last_record_sent)) {
      printf("Failed to read send counter");
      lfs_file_close(&lfs, &file);
      return false;
    }
    lfs_file_close(&lfs, &file);
    printf("Last record sent: %d\n", last_record_sent);
  }

  return true;
}

void Storage::set_latest_message(location_reading location) {
  memcpy(message_buffer, &location, size_of_location);
  pending_archive = true;
}

void Storage::store_latest_message() {
  // if we enter this function we have not successfully sent the message
  // and it is still stored in the message_buffer
  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_WRONLY | LFS_O_APPEND) !=
      LFS_ERR_OK) {
    printf("file open failed");
    return;
  }

  lfs_file_write(&lfs, &file, message_buffer, record_size);
  lfs_file_close(&lfs, &file);
  pending_archive = false;
}

bool Storage::anything_to_send() {
  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_RDONLY) != LFS_ERR_OK) {
    printf("file open failed");
    return false;
  }

  lfs_soff_t file_size = lfs_file_size(&lfs, &file);
  int total_records = file_size / record_size;

  if (total_records > last_record_sent) {
    lfs_file_seek(&lfs, &file, last_record_sent * record_size, LFS_SEEK_SET);
    lfs_file_read(&lfs, &file, message_buffer, record_size);
    lfs_file_close(&lfs, &file);
    last_record_sent = total_records;
    return true;
  }

  lfs_file_close(&lfs, &file);
  return false;
}

void Storage::send_successful() {
  if (pending_archive) {
    pending_archive = false;
    return;
  }

  last_record_sent++;
  if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_WRONLY | LFS_O_TRUNC) !=
      LFS_ERR_OK) {
    printf("Failed to open file for writing");
    return;
  }

  lfs_file_write(&lfs, &file, &last_record_sent, sizeof(last_record_sent));
  lfs_file_close(&lfs, &file);
}
