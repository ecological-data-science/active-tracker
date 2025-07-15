#include "storage.h"


bool Storage::begin() {


  // First try to mount the existing filesystem
  int err = lfs_mount(&lfs, &cfg);

  // If mounting fails, format and then mount
  if (err) {
    DEBUG_PRINT(("Failed to mount LittleFS, formatting..."));
    err = lfs_format(&lfs, &cfg);
    if (err) {
      DEBUG_PRINT(("Failed to format LittleFS"));
      return false;
    }
        
    err = lfs_mount(&lfs, &cfg);
    if (err) {
      DEBUG_PRINT(("Failed to mount LittleFS after formatting"));
      return false;
    }
  }

  DEBUG_PRINT(("LittleFS mounted successfully"));
  
  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_RDONLY) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (lfs_file_open(&lfs, &file, dataFile, LFS_O_WRONLY | LFS_O_CREAT) !=
        LFS_ERR_OK) {
        DEBUG_PRINT(("Failed to open file for writing"));
      return false;
    }
    lfs_file_close(&lfs, &file);
  } else {
    lfs_soff_t file_size = lfs_file_size(&lfs, &file);
    int total_records = file_size / record_size;
    DEBUG_PRINT(("Total records: %d", total_records));
    lfs_file_close(&lfs, &file);
  }
  DEBUG_PRINT(("dataFile opened successfully"));

  if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_RDONLY) != LFS_ERR_OK) {
    // File doesn't exist, create it
    if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_WRONLY | LFS_O_CREAT) !=
        LFS_ERR_OK) {
        DEBUG_PRINT(("Failed to open file for writing"));
      return false;
    }
    uint32_t send_counter = 0;
    lfs_file_write(&lfs, &file, &send_counter, sizeof(send_counter));
    lfs_file_close(&lfs, &file);
  } else {
    // Read the send counter
    if (lfs_file_read(&lfs, &file, &last_record_sent,
                      sizeof(last_record_sent)) != sizeof(last_record_sent)) {
        DEBUG_PRINT(("Failed to read send counter"));
      lfs_file_close(&lfs, &file);
      return false;
    }
    lfs_file_close(&lfs, &file);
        DEBUG_PRINT(("Last record sent: %d", last_record_sent));
  }
  DEBUG_PRINT(("sendCounter opened successfully"));

  return true;
}

void Storage::set_latest_message(location_reading location, activity_reading activities) {
  current_reading.location = location;
  current_reading.activity = activities;
  current_reading.partially_sent = false;
  new_message = true;
}

void Storage::archive_latest_message() {
  if (!new_message) {
    return;
  }
  // if we enter this function we have not successfully sent the message
  // and it is still stored in the message_buffer
  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_WRONLY | LFS_O_APPEND) !=
      LFS_ERR_OK) {
        DEBUG_PRINT(("file open failed"));
    return;
  }

    // Explicitly serialize each field to avoid padding issues
    uint8_t* ptr = message_buffer;
    
    // Location data
    memcpy(ptr, &current_reading.location.start_time, sizeof(long));
    ptr += sizeof(long);
    memcpy(ptr, &current_reading.location.lat, sizeof(float));
    ptr += sizeof(float);
    memcpy(ptr, &current_reading.location.lon, sizeof(float));
    ptr += sizeof(float);
    
    // Activity data
    memcpy(ptr, &current_reading.activity.start_time, sizeof(long));
    ptr += sizeof(long);
    memcpy(ptr, &current_reading.activity.activities, NUM_CLASSIFICATIONS * sizeof(uint8_t));
    ptr += NUM_CLASSIFICATIONS * sizeof(uint8_t);
    
    // Partially sent flag
    memcpy(ptr, &current_reading.partially_sent, sizeof(bool));
    
    lfs_file_write(&lfs, &file, message_buffer, record_size);
    lfs_file_close(&lfs, &file);
    new_message = false;
  // memcpy(message_buffer, &current_reading, sizeof(current_reading));
  // lfs_file_write(&lfs, &file, message_buffer, record_size);
  // lfs_file_close(&lfs, &file);
  // new_message = false;
}

combined_reading Storage::get_current_reading() {
  return current_reading;
}

  
bool Storage::anything_to_send(bool nightmode) {

  // always try to send the latest message
  if (new_message) {
    return true;
  }
  // if there's no new message and we're not in night mode we'll quit
  if (!nightmode) {
    return false;
  }

  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_RDONLY) != LFS_ERR_OK) {
        DEBUG_PRINT(("file open failed"));
    return false;
  }

  lfs_soff_t file_size = lfs_file_size(&lfs, &file);
  int total_records = file_size / record_size;

  if (total_records > last_record_sent) {
      lfs_file_seek(&lfs, &file, last_record_sent * record_size, LFS_SEEK_SET);
      lfs_file_read(&lfs, &file, message_buffer, record_size);
      lfs_file_close(&lfs, &file);
      
      // Explicitly deserialize each field
      uint8_t* ptr = message_buffer;
      
      // Location data
      memcpy(&current_reading.location.start_time, ptr, sizeof(long));
      ptr += sizeof(long);
      memcpy(&current_reading.location.lat, ptr, sizeof(float));
      ptr += sizeof(float);
      memcpy(&current_reading.location.lon, ptr, sizeof(float));
      ptr += sizeof(float);
      
      // Activity data
      memcpy(&current_reading.activity.start_time, ptr, sizeof(long));
      ptr += sizeof(long);
      memcpy(&current_reading.activity.activities, ptr, NUM_CLASSIFICATIONS * sizeof(uint8_t));
      ptr += NUM_CLASSIFICATIONS * sizeof(uint8_t);
      
      // Partially sent flag
      memcpy(&current_reading.partially_sent, ptr, sizeof(bool));
      
      return true;

    // lfs_file_seek(&lfs, &file, last_record_sent * record_size, LFS_SEEK_SET);
    // lfs_file_read(&lfs, &file, message_buffer, record_size);
    // lfs_file_close(&lfs, &file);
    // memcpy(&current_reading, message_buffer, sizeof(current_reading));
    // return true;
  }

  lfs_file_close(&lfs, &file);
  return false;
}

void Storage::location_send_successful() {
  current_reading.partially_sent = true;

  if (new_message) {
      return;
  }
  
  if (lfs_file_open(&lfs, &file, dataFile, LFS_O_WRONLY) != LFS_ERR_OK) {
      DEBUG_PRINT(("Failed to open data file for updating record %lu", last_record_sent));
      return;
  }

  // Seek to the position of the current archived record
  lfs_soff_t seek_pos = (lfs_soff_t)last_record_sent * record_size;
  if (lfs_file_seek(&lfs, &file, seek_pos, LFS_SEEK_SET) < 0) {
      DEBUG_PRINT(("Failed to seek to record %lu for update", last_record_sent));
      lfs_file_close(&lfs, &file);
      return;
  }

  // Explicitly serialize each field to avoid padding issues
  uint8_t* ptr = message_buffer;
  
  // Location data
  memcpy(ptr, &current_reading.location.start_time, sizeof(long));
  ptr += sizeof(long);
  memcpy(ptr, &current_reading.location.lat, sizeof(float));
  ptr += sizeof(float);
  memcpy(ptr, &current_reading.location.lon, sizeof(float));
  ptr += sizeof(float);
  
  // Activity data
  memcpy(ptr, &current_reading.activity.start_time, sizeof(long));
  ptr += sizeof(long);
  memcpy(ptr, &current_reading.activity.activities, NUM_CLASSIFICATIONS * sizeof(uint8_t));
  ptr += NUM_CLASSIFICATIONS * sizeof(uint8_t);
  
  // Partially sent flag - now set to true
  memcpy(ptr, &current_reading.partially_sent, sizeof(bool));

  // Write the buffer back to the file at the correct position
  if (lfs_file_write(&lfs, &file, message_buffer, record_size) != record_size) {
      DEBUG_PRINT(("Failed to write updated record %lu to archive", last_record_sent));
  }

  lfs_file_close(&lfs, &file);

}


void Storage::activity_send_successful() {
  if (new_message) {
    new_message = false;
    return;
  }

  last_record_sent++;

  // Save the updated send counter to the file
  if (lfs_file_open(&lfs, &file, sendCounter, LFS_O_WRONLY | LFS_O_TRUNC) != LFS_ERR_OK) {
        DEBUG_PRINT(("Failed to open send counter file for writing"));
    return;
  }

  if (lfs_file_write(&lfs, &file, &last_record_sent, sizeof(last_record_sent)) != sizeof(last_record_sent)) {
        DEBUG_PRINT(("Failed to write updated send counter"));
  }

  lfs_file_close(&lfs, &file);

}
