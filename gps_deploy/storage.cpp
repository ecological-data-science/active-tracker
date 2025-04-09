#include "storage.h"

void Storage::begin() {

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return;
  }
  if (!LittleFS.exists(devNonce)) {
    File file = LittleFS.open(devNonce, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    file.write(0);
    file.close();
  }
  if (!LittleFS.exists(sendCounter)) {
    File file = LittleFS.open(sendCounter, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    file.write(0);
    file.close();
  } else {
    File file = LittleFS.open(sendCounter, "r");
    if (!file) {
      Serial.println("Failed to open file for reading");
      return;
    }
    send_counter = file.read();
    file.close();
    Serial.print("Send counter: ");
    Serial.println(send_counter);
  }
  delay(500);
}

void Storage::sleep() { delay(500); }

void Storage::write_next_message(location_reading location) {

  File file = LittleFS.open(dataFile, "a");
  if (!file) {
    Serial.println("file open failed");
    return;
  }

  uint8_t record_buffer[record_size];

  memcpy(record_buffer, &location, size_of_location);
  // memcpy(&record_buffer[size_of_location], &activity, size_of_activity);

  file.write(record_buffer, record_size);
  file.close();
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

  send_counter++;
  File file = LittleFS.open(sendCounter, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write(send_counter);
  file.close();
}
