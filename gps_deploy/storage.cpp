#include "storage.h"

bool Storage::begin() {

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return false;
  }
  if (!LittleFS.exists(devNonce)) {
    File file = LittleFS.open(devNonce, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      return false;
    }
    uint8_t dev_nonce = 0;
    file.write(dev_nonce);
    file.close();
  }
  if (!LittleFS.exists(sendCounter)) {
    File file = LittleFS.open(sendCounter, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      return false;
    }

    uint8_t send_counter = 0;
    file.write(send_counter);
    file.close();
  } else {
    File file = LittleFS.open(sendCounter, "r");
    if (!file) {
      Serial.println("Failed to open file for reading");
      return false;
    }
    last_record_sent = file.read();
    file.close();
    Serial.print("Send counter: ");
    Serial.println(last_record_sent);
  }
  delay(500);
  return true;
}

void Storage::sleep() { delay(500); }

void Storage::set_latest_message(location_reading location) {

  memcpy(message_buffer, &location, size_of_location);
  pending_archive = true;
}

void Storage::store_latest_message() {

  // if we enter this function we have not successfully sent the message
  // and it is still stored in the message_buffer
  File file = LittleFS.open(dataFile, "a");
  if (!file) {
    Serial.println("file open failed");
    return;
  }

  file.write(message_buffer, record_size);
  file.close();
  pending_archive = false;
}

bool Storage::anything_to_send() {

  File file = LittleFS.open(dataFile, "r");
  if (!file) {
    Serial.println("file open failed");
    return false;
  }

  int total_records = file.size() / record_size;

  if (total_records > last_record_sent) {
    file.seek(last_record_sent * record_size, SeekSet);
    file.read(message_buffer, record_size);
    last_record_sent = total_records;
    return true;
  }
  return false;
}

void Storage::send_successful() {

  if (pending_archive) {
    pending_archive = false;
    return;
  }
  last_record_sent++;
  File file = LittleFS.open(sendCounter, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write(last_record_sent);
  file.close();
}
