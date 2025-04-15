
/*
  An Arduino Library which allows you to communicate seamlessly with u-blox GNSS
  modules using the Configuration Interface

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/16344
  https://www.sparkfun.com/products/18037
  https://www.sparkfun.com/products/18719
  https://www.sparkfun.com/products/18774
  https://www.sparkfun.com/products/19663
  https://www.sparkfun.com/products/17722

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020
  v3.0 rework by Paul Clark @ SparkFun Electronics, December 8th, 2022

  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  This library is an updated version of the popular SparkFun u-blox GNSS Arduino
  Library. v3 uses the u-blox Configuration Interface (VALSET and VALGET) to:
  detect the module (during begin); configure message intervals; configure the
  base location; etc..

  This version of the library will not work with older GNSS modules.
  It is specifically written for newer modules like the ZED-F9P, ZED-F9R and
  MAX-M10S. For older modules, please use v2 of the library:
  https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.19

  SparkFun code, firmware, and software is released under the MIT
  License(http://opensource.org/licenses/MIT). The MIT License (MIT) Copyright
  (c) 2018 SparkFun Electronics Permission is hereby granted, free of charge, to
  any person obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to permit
  persons to whom the Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

DevUBLOXGNSS::DevUBLOXGNSS(void) {
  // Constructor
  if (debugPin >= 0) {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0; // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA
                                                // messages to processNMEA
  _logRTCM.all = 0; // Default to passing no RTCM messages to the file buffer

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  rtcmInputStorage.init();
#endif
}

DevUBLOXGNSS::DevUBLOXGNSS(void) {
  // Constructor
  if (debugPin >= 0) {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0; // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA
                                                // messages to processNMEA
  _logRTCM.all = 0; // Default to passing no RTCM messages to the file buffer

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  rtcmInputStorage.init();
#endif
}

DevUBLOXGNSS::~DevUBLOXGNSS(void) {
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and
         // spiBuffer

  if (payloadCfg != nullptr) {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
  }

  if (payloadAuto != nullptr) {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = nullptr;
  }

  if (spiBuffer != nullptr) {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = nullptr;
  }
}

// Stop all automatic message processing. Free all used RAM
void DevUBLOXGNSS::end(void) {
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer !=
      nullptr) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.println(
          F("end: the file buffer has been deleted. You will need to call "
            "setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = nullptr;
    fileBufferSize = 0; // Reset file buffer size. User will have to call
                        // setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (cfgValgetValueSizes != nullptr) {
    delete[] cfgValgetValueSizes;
    cfgValgetValueSizes = nullptr;
  }

  if (moduleSWVersion != nullptr) {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = nullptr;
  }

  if (packetUBXNAVPVT != nullptr) {
    if (packetUBXNAVPVT->callbackData != nullptr) {
      delete packetUBXNAVPVT->callbackData;
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = nullptr;
  }

  deleteLock(); // Delete the lock semaphore - if required
}

// Return the number of free bytes remaining in packetCfgPayload
size_t DevUBLOXGNSS::getPacketCfgSpaceRemaining() {
  return (packetCfgPayloadSize - packetCfg.len);
}

// New in v3.0: hardware interface is abstracted
void DevUBLOXGNSS::setCommunicationBus(
    SparkFun_UBLOX_GNSS::GNSSDeviceBus &theBus) {
  _sfeBus = &theBus;
}
// For Serial, return Serial.available()
// For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
// Not Applicable for SPI
uint16_t DevUBLOXGNSS::available() { return _sfeBus->available(); }
// For I2C, ping the _address
// Not Applicable for SPI and Serial
bool DevUBLOXGNSS::ping() {
  if (!lock())
    return false;
  bool ok = _sfeBus->ping();
  unlock();
  return ok;
}

// TODO: here update for rp2040
//
//

// For Serial, do Serial.write
// For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single
// byte writes as these are illegal For SPI, writing bytes will also read bytes
// simultaneously. Read data is _ignored_ here. Use writeReadBytes
uint8_t DevUBLOXGNSS::writeBytes(uint8_t *data, uint8_t length) {
  return _sfeBus->writeBytes(data, length);
}
// For SPI, writing bytes will also read bytes simultaneously. Read data is
// returned in readData
uint8_t DevUBLOXGNSS::writeReadBytes(const uint8_t *data, uint8_t *readData,
                                     uint8_t length) {
  return _sfeBus->writeReadBytes(data, readData, length);
}
void DevUBLOXGNSS::startWriteReadByte() { _sfeBus->startWriteReadByte(); }
void DevUBLOXGNSS::writeReadByte(const uint8_t *data, uint8_t *readData) {
  _sfeBus->writeReadByte(data, readData);
}
void DevUBLOXGNSS::writeReadByte(const uint8_t data, uint8_t *readData) {
  _sfeBus->writeReadByte(data, readData);
}
void DevUBLOXGNSS::endWriteReadByte() { _sfeBus->endWriteReadByte(); }
// For Serial, attempt Serial.read
// For I2C, read from register 0xFF
// For SPI, read the byte while writing 0xFF
uint8_t DevUBLOXGNSS::readBytes(uint8_t *data, uint8_t length) {
  return _sfeBus->readBytes(data, length);
}

bool DevUBLOXGNSS::init(uint16_t maxWait, bool assumeSuccess) {
  createLock(); // Create the lock semaphore - if needed

  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if
                        // valid traffic is seen.

  setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE
  // poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected) {
    connected = isConnected(maxWait);
  }

  if (!connected) {
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess &&
      _signsOfLife) // Advanced users can assume success if required. Useful if
                    // the port is outputting messages at high navigation rate.
  {
    return (true);
  }

  return (connected);
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase
// the transaction size if the platform supports it Note: If the transaction
// size is set larger than the platforms buffer size, bad things will happen.
void DevUBLOXGNSS::setI2CTransactionSize(uint8_t transactionSize) {
  if (transactionSize < 8)
    transactionSize = 8; // Ensure transactionSize is at least 8 bytes otherwise
                         // sendI2cCommand will have problems!

  i2cTransactionSize = transactionSize;
}
uint8_t DevUBLOXGNSS::getI2CTransactionSize(void) {
  return (i2cTransactionSize);
}

// Returns true if I2C device ack's
bool DevUBLOXGNSS::isConnected(uint16_t maxWait) {
  if (!ping())
    return false; // Sensor did not ack

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles,
  // let's request the config for most appropriate port
  uint8_t en;
  return getVal8(UBLOX_CFG_I2CINPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
}

const char *DevUBLOXGNSS::statusString(sfe_ublox_status_e stat) {
  switch (stat) {
  case SFE_UBLOX_STATUS_SUCCESS:
    return "Success";
    break;
  case SFE_UBLOX_STATUS_FAIL:
    return "General Failure";
    break;
  case SFE_UBLOX_STATUS_CRC_FAIL:
    return "CRC Fail";
    break;
  case SFE_UBLOX_STATUS_TIMEOUT:
    return "Timeout";
    break;
  case SFE_UBLOX_STATUS_COMMAND_NACK:
    return "Command not acknowledged (NACK)";
    break;
  case SFE_UBLOX_STATUS_OUT_OF_RANGE:
    return "Out of range";
    break;
  case SFE_UBLOX_STATUS_INVALID_ARG:
    return "Invalid Arg";
    break;
  case SFE_UBLOX_STATUS_INVALID_OPERATION:
    return "Invalid operation";
    break;
  case SFE_UBLOX_STATUS_MEM_ERR:
    return "Memory Error";
    break;
  case SFE_UBLOX_STATUS_HW_ERR:
    return "Hardware Error";
    break;
  case SFE_UBLOX_STATUS_DATA_SENT:
    return "Data Sent";
    break;
  case SFE_UBLOX_STATUS_DATA_RECEIVED:
    return "Data Received";
    break;
  case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
    return "I2C Comm Failure";
    break;
  case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
    return "Data Packet Overwritten";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Check for the arrival of new I2C/Serial/SPI data
// Called regularly to check for available bytes on the user' specified port

bool DevUBLOXGNSS::checkUblox(uint8_t requestedClass, uint8_t requestedID) {
  return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}

// PRIVATE: Called regularly to check for available bytes on the user' specified
// port
bool DevUBLOXGNSS::checkUbloxInternal(ubxPacket *incomingUBX,
                                      uint8_t requestedClass,
                                      uint8_t requestedID) {
  if (!lock())
    return false;

  bool ok = false;
  ok = (checkUbloxI2C(incomingUBX, requestedClass, requestedID));

  unlock();

  return ok;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool DevUBLOXGNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass,
                                 uint8_t requestedID) {
  if (millis() - lastCheck >= i2cPollingWait) {
    // Get the number of bytes available from the module
    // From the u-blox integration manual:
    // "There are two forms of DDC read transfer. The "random access" form
    // includes a peripheral register
    //  address and thus allows any register to be read. The second "current
    //  address" form omits the register address. If this second form is used,
    //  then an address pointer in the receiver is used to determine which
    //  register to read. This address pointer will increment after each read
    //  unless it is already pointing at register 0xFF, the highest addressable
    //  register, in which case it remains unaltered."
    uint16_t bytesAvailable = available();

    if (bytesAvailable == 0) {
      // #ifndef SFE_UBLOX_REDUCED_PROG_MEM
      //       if (_printDebug == true)
      //       {
      //         _debugSerial.println(F("checkUbloxI2C: OK, zero bytes
      //         available"));
      //       }
      // #endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the
    // two 'data available' bytes as 1 then we have far too many bytes to check.
    // May be related to I2C setup time violations:
    // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15)) {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);
    }

    while (bytesAvailable) {
      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead >
          i2cTransactionSize) // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // Here it would be desireable to use a restart where possible /
      // supported, but only if there will be multiple reads. However, if an
      // individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t buf[i2cTransactionSize];
      uint8_t bytesReturned = readBytes(buf, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead) {
        for (uint16_t x = 0; x < bytesToRead; x++) {
          process(buf[x], incomingUBX, requestedClass,
                  requestedID); // Process this valid character
        }
      } else {
        // Something has gone very wrong. Sensor did not respond - or a bus
        // error happened...
        if (_resetCurrentSentenceOnBusError)
          currentSentence =
              SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being
                                            // looking for a new start char
        return (false);
      }

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()

// Processes NMEA, RTCM and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void DevUBLOXGNSS::process(uint8_t incoming, ubxPacket *incomingUBX,
                           uint8_t requestedClass, uint8_t requestedID) {
  // Update storedClass and storedID if either requestedClass or requestedID is
  // non-zero, otherwise leave unchanged. This allows calls of checkUblox()
  // (which defaults to checkUblox(0,0)) by other threads without overwriting
  // the requested / expected Class and ID.
  volatile static uint8_t storedClass = 0;
  volatile static uint8_t storedID = 0;
  static size_t payloadAutoBytes;
  if (requestedClass ||
      requestedID) // If either is non-zero, store the requested Class and ID
  {
    storedClass = requestedClass;
    storedID = requestedID;
  }

  _outputPort.write(incoming); // Echo this byte to the serial port

  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) ||
      (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA)) {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again
      // when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a stored class and ID
      // match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    } else if (incoming == '$') {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    } else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    } else {
      // This character is unknown or we missed the previous start of a sentence
      // Or it could be a 0xFF from a SPI transaction
    }
  }

  uint16_t maxPayload = 0;

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX) {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1)) // ISO 'μ'
      currentSentence =
          SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence =
          SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as
    // processUBX will also load information into packetBuf, but we'll do it
    // here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA =
          0; // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0; // Reset the packetBuf.counter (again)
      packetBuf.valid =
          SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity
                                                 // (redundant?)
      packetBuf.startingSpot =
          incomingUBX->startingSpot; // Copy the startingSpot
    } else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and
      // ID match
      if (packetBuf.cls != UBX_CLASS_ACK) {
        bool logBecauseAuto =
            autoLookup(packetBuf.cls, packetBuf.id, &maxPayload);
        bool logBecauseEnabled = logThisUBX(packetBuf.cls, packetBuf.id);

        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == storedClass) && (packetBuf.id == storedID)) {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into
                                            // incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter =
              packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own
        // storage defined
        else if (logBecauseAuto || logBecauseEnabled) {
          // This is not the message we were expecting but it has its own
          // storage and so we should process it anyway. We'll try to use
          // packetAuto to buffer the message (so it can't overwrite anything in
          // packetCfg). We need to allocate memory for the packetAuto payload
          // (payloadAuto) - and delete it once reception is complete.
          if (logBecauseAuto && (maxPayload == 0)) {
          }

          // Determine the payload length
          if ((!logBecauseAuto) && (logBecauseEnabled))
            maxPayload = SFE_UBX_MAX_LENGTH;

          // Increase the payloadAuto buffer size if necessary, by removing
          // the previous buffer
          if (payloadAuto && (payloadAutoBytes < maxPayload)) {
            delete[] payloadAuto; // Created with new[] below
            payloadAuto = nullptr;
            payloadAutoBytes = 0;
          }

          // Allocate the payloadAuto buffer if necessary
          if (payloadAuto == nullptr) {
            payloadAuto = new uint8_t[maxPayload];
            if (payloadAuto)
              payloadAutoBytes = maxPayload;
          }

          packetAuto.payload = payloadAuto;

          if (payloadAuto == nullptr) // Check if the alloc failed
          {
            // The RAM allocation failed so fall back to using incomingUBX
            // (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into
                                              // incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter =
                packetBuf.counter; // Copy over the .counter too
          } else {
            // The RAM allocation was successful so we start diverting data into
            // packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls =
                packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter =
                packetBuf.counter; // Copy over the .counter too
            packetAuto.startingSpot =
                packetBuf.startingSpot; // And the starting spot? (Probably
                                        // redundant)
          }
        } else {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the
          // payload
          ignoreThisPayload = true;
        }
      } else {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    } else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer ==
      // SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming;      // (Duplication)
    } else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer ==
      // SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    } else if (ubxFrameCounter == 6)  // This should be the first byte of the
                                      // payload unless .len is zero
    {
      if (packetBuf.len ==
          0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the first byte of the checksum so
        // record it
        packetBuf.checksumA = incoming;
      } else {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    } else if (ubxFrameCounter == 7) // This should be the second byte of the
                                     // payload unless .len is zero or one
    {
      if (packetBuf.len ==
          0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so
        // record it
        packetBuf.checksumB = incoming;
      } else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      } else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a
      // matching ACK/NACK
      if ((activePacketBuffer ==
           SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a
                                       // data packet
          && (packetBuf.cls == UBX_CLASS_ACK)      // and if this is an ACK/NACK
          && (packetBuf.payload[0] == storedClass) // and if the class matches
          && (packetBuf.payload[1] == storedID))   // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, storedClass, storedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, storedClass, storedID);

    // If user has assigned an output port then pipe the characters there,
    // but only if the port is different (otherwise we'll output each character
    // twice!)
    if (_outputPort != _ubxOutputPort)
      _ubxOutputPort.write(incoming); // Echo this byte to the serial port

    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
}

// PRIVATE: Return true if we should add this NMEA message to the file buffer
// for logging

// PRIVATE: Return true if the NMEA header is valid

// PRIVATE: Return true if we should pass this NMEA message to processNMEA

// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and
// passes or fails CRC The payload portion of the packet can be 100s of bytes
// but the max array size is packetCfgPayloadSize bytes. startingSpot can be set
// so we only record a subset of bytes within a larger packet.
void DevUBLOXGNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX,
                              uint8_t requestedClass, uint8_t requestedID) {
  // If incomingUBX is a user-defined custom packet, then the payload size could
  // be different to packetCfgPayloadSize. TO DO: update this to prevent an
  // overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than
  //        packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO) {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be
    // SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{

    bool logBecauseAuto =
        autoLookup(incomingUBX->cls, incomingUBX->id, &maximum_payload_size);
    bool logBecauseEnabled = logThisUBX(incomingUBX->cls, incomingUBX->id);
    if ((!logBecauseAuto) && (logBecauseEnabled))
      maximum_payload_size = SFE_UBX_MAX_LENGTH;
    if (maximum_payload_size == 0) {
    }
    //}
    // else
    //  maximum_payload_size = 2;
  } else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to
  // the rolling checksum
  if (incomingUBX->counter < (incomingUBX->len + 4))
    addToChecksum(incoming);

  if (incomingUBX->counter == 0) {
    incomingUBX->cls = incoming;
  } else if (incomingUBX->counter == 1) {
    incomingUBX->id = incoming;
  } else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  } else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  } else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  } else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence =
        SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being
                                      // looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) &&
        (incomingUBX->checksumB == rollingChecksumB)) {
      incomingUBX->valid =
          SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife =
          true; // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and
      // requestedID Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) &&
          (incomingUBX->id == requestedID)) {
        incomingUBX->classAndIDmatch =
            SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the
                                             // classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the
      // requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) &&
               (incomingUBX->id == UBX_ACK_ACK) &&
               (incomingUBX->payload[0] == requestedClass) &&
               (incomingUBX->payload[1] == requestedID)) {
        incomingUBX->classAndIDmatch =
            SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the
                                             // classAndIDmatch flag to valid
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if ((autoLookup(incomingUBX->cls, incomingUBX->id)) ||
               (logThisUBX(incomingUBX->cls, incomingUBX->id))) {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
      }

      // We've got a valid packet, now do something with it but only if
      // ignoreThisPayload is false
      if (ignoreThisPayload == false) {
        processUBXpacket(incomingUBX);
      }
    } else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and
      // requestedID. This is potentially risky as we are saying that we saw the
      // requested Class and ID but that the packet checksum failed. Potentially
      // it could be the class or ID bytes that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) &&
          (incomingUBX->id == requestedID)) {
        incomingUBX->classAndIDmatch =
            SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the
                                                 // classAndIDmatch flag to not
                                                 // valid
      }
      // If this is an ACK then let's check if the class and ID match the
      // requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) &&
               (incomingUBX->payload[0] == requestedClass) &&
               (incomingUBX->payload[1] == requestedID)) {
        incomingUBX->classAndIDmatch =
            SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the
                                                 // classAndIDmatch flag to not
                                                 // valid
      }

      // Now that the packet is complete and has been processed, 'free' the
      // memory for packetAuto but leave payloadAuto allocated (See #75)
      if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
        packetAuto.payload = nullptr;
    }
    else // Load this byte into the payload array
    {
      // If an automatic packet comes in asynchronously, we need to fudge the
      // startingSpot
      uint16_t startingSpot = incomingUBX->startingSpot;
      if (autoLookup(incomingUBX->cls, incomingUBX->id))
        startingSpot = 0;
      // Check if this is payload data which should be ignored
      if (ignoreThisPayload == false) {
        // Begin recording if counter goes past startingSpot
        if ((incomingUBX->counter - 4) >= startingSpot) {
          // Check to see if we have room for this byte
          if (((incomingUBX->counter - 4) - startingSpot) <
              maximum_payload_size) // If counter = 208, starting spot = 200,
                                    // we're good to record.
          {
            incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] =
                incoming; // Store this byte into payload array
          } else {
            overrun = true;
          }
        }
      }
    }

    // incomingUBX->counter should never reach maximum_payload_size + class + id
    // + len[2] + checksum[2]
    if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) &&
                    (ignoreThisPayload == false))) {
      // Something has gone very wrong
      currentSentence =
          SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking
                                        // for a new start char
    }

    // Increment the counter
    incomingUBX->counter++;
  }

  // Once a packet has been received and validated, identify this packet's
  // class/id and update internal flags
  void DevUBLOXGNSS::processUBXpacket(ubxPacket * msg) {
    bool addedToFileBuffer = false;
    switch (msg->cls) {
    case UBX_CLASS_NAV:
      if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN) {
        // Parse various byte fields into storage - but only if we have memory
        // allocated for it
        if (packetUBXNAVPVT != nullptr) {
          packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
          packetUBXNAVPVT->data.year = extractInt(msg, 4);
          packetUBXNAVPVT->data.month = extractByte(msg, 6);
          packetUBXNAVPVT->data.day = extractByte(msg, 7);
          packetUBXNAVPVT->data.hour = extractByte(msg, 8);
          packetUBXNAVPVT->data.min = extractByte(msg, 9);
          packetUBXNAVPVT->data.sec = extractByte(msg, 10);
          packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
          packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
          packetUBXNAVPVT->data.nano =
              extractSignedLong(msg, 16); // Includes milliseconds
          packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
          packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
          packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
          packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
          packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
          packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
          packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
          packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
          packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
          packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
          packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
          packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
          packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
          packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
          packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
          packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
          packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
          packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
          packetUBXNAVPVT->data.flags3.all = extractInt(msg, 78);
          packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
          packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
          packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

          // Mark all datums as fresh (not read before)
          packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
          packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

          // Check if we need to copy the data for the callback
          if ((packetUBXNAVPVT->callbackData !=
               nullptr) // If RAM has been allocated for the copy of the data
              &&
              (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid ==
               false)) // AND the data is stale
          {
            memcpy(&packetUBXNAVPVT->callbackData->iTOW,
                   &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
            packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
          }

          // Check if we need to copy the data into the file buffer
          if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer) {
            addedToFileBuffer = storePacket(msg);
          }
        }
      }

      break;
    }

    // Check if this UBX message should be added to the file buffer - if it has
    // not been added already
    if ((!addedToFileBuffer) && (logThisUBX(msg->cls, msg->id)))
      storePacket(msg);
  }

  // Given a message, calc and store the two byte "8-Bit Fletcher" checksum over
  // the entirety of the message This is called before we send a command message
  void DevUBLOXGNSS::calcChecksum(ubxPacket * msg) {
    msg->checksumA = 0;
    msg->checksumB = 0;

    msg->checksumA += msg->cls;
    msg->checksumB += msg->checksumA;

    msg->checksumA += msg->id;
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len & 0xFF);
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len >> 8);
    msg->checksumB += msg->checksumA;

    for (uint16_t i = 0; i < msg->len; i++) {
      msg->checksumA += msg->payload[i];
      msg->checksumB += msg->checksumA;
    }
  }

  // Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
  // This is used when receiving messages from module
  void DevUBLOXGNSS::addToChecksum(uint8_t incoming) {
    rollingChecksumA += incoming;
    rollingChecksumB += rollingChecksumA;
  }

  // Given a packet and payload, send everything including CRC bytes via I2C
  // port
  sfe_ublox_status_e DevUBLOXGNSS::sendCommand(
      ubxPacket * outgoingUBX, uint16_t maxWait, bool expectACKonly) {
    if (!lock())
      return SFE_UBLOX_STATUS_FAIL;

    sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

    calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

    retVal = sendI2cCommand(outgoingUBX);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS) {
      unlock();
      return retVal;
    }

    unlock();

    if (maxWait > 0) {
      // Depending on what we just sent, either we need to look for an ACK or
      // not
      if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true)) {
        retVal =
            waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id,
                               maxWait); // Wait for Ack response
      } else {
        retVal =
            waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id,
                                 maxWait); // Wait for Ack response
      }
    } else {
      processSpiBuffer(
          &packetCfg, 0,
          0); // Process any SPI data received during the sendSpiCommand - but
              // only if not checking for a response
    }

    return retVal;
  }

  // Returns false if sensor fails to respond to I2C traffic
  sfe_ublox_status_e DevUBLOXGNSS::sendI2cCommand(ubxPacket * outgoingUBX) {
    // From the integration guide:
    // "The receiver does not provide any write access except for writing UBX
    // and NMEA messages to the
    //  receiver, such as configuration or aiding data. Therefore, the register
    //  set mentioned in section Read Access is not writeable. Following the
    //  start condition from the master, the 7-bit device address and the RW bit
    //  (which is a logic low for write access) are clocked onto the bus by the
    //  master transmitter. The receiver answers with an acknowledge (logic low)
    //  to indicate that it is responsible for the given address. Now, the
    //  master can write 2 to N bytes to the receiver, generating a stop
    //  condition after the last byte being written. The number of data bytes
    //  must be at least 2 to properly distinguish from the write access to set
    //  the address counter in random read accesses."
    // I take two things from this:
    // 1) We do not need to write 0xFF to point at register 0xFF. We're already
    // pointing at it. 2) We must always write at least 2 bytes, otherwise it
    // looks like we are starting to do a read. Point 2 is important. It means:
    // * In this function:
    //     if we do multiple writes (because we're trying to write more than
    //     i2cTransactionSize), we may need to write one byte less in the
    //     penultimate write to ensure we always have two bytes left for the
    //     final write.
    // * In pushRawData:
    //     if there is one byte to write, or one byte left to write, we need to
    //     do the same thing and may need to store a single byte until
    //     pushRawData is called again.

    // The total number of bytes to be written is: payload len + 8
    // UBX_SYNCH_1
    // UBX_SYNCH_2
    // cls
    // id
    // len (MSB)
    // len (LSB)
    // < payload >
    // checksumA
    // checksumB

    // i2cTransactionSize will be at least 8. We don't need to check for smaller
    // values than that.

    uint16_t bytesLeftToSend =
        outgoingUBX->len;   // How many bytes remain to be sent
    uint16_t startSpot = 0; // Payload pointer

    // Check if we can send all the data in one transfer?
    if (bytesLeftToSend + 8 <= i2cTransactionSize) {
      uint8_t buf[i2cTransactionSize];
      buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you
                            // micro-blox from now on.
      buf[1] = UBX_SYNCH_2; // b
      buf[2] = outgoingUBX->cls;
      buf[3] = outgoingUBX->id;
      buf[4] = outgoingUBX->len & 0xFF; // LSB
      buf[5] = outgoingUBX->len >> 8;   // MSB
      uint16_t i = 0;
      for (; i < outgoingUBX->len; i++)
        buf[i + 6] = outgoingUBX->payload[startSpot + i];
      buf[i + 6] = outgoingUBX->checksumA;
      buf[i + 7] = outgoingUBX->checksumB;

      if (writeBytes(buf, bytesLeftToSend + 8) != bytesLeftToSend + 8)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
    }

    else {
      uint8_t buf[6];
      buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you
                            // micro-blox from now on.
      buf[1] = UBX_SYNCH_2; // b
      buf[2] = outgoingUBX->cls;
      buf[3] = outgoingUBX->id;
      buf[4] = outgoingUBX->len & 0xFF; // LSB
      buf[5] = outgoingUBX->len >> 8;   // MSB

      if (writeBytes(buf, 6) != 6)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

      // If bytesLeftToSend is zero, that's OK.
      // If bytesLeftToSend is >= 2, that's OK.
      // But if bytesLeftToSend is 1, we need to carry that byte over and send
      // it with the checksum bytes
      while (bytesLeftToSend > 1) {
        uint16_t len =
            bytesLeftToSend; // How many bytes should we actually write?
        if (len > i2cTransactionSize) // Limit len to i2cTransactionSize
          len = i2cTransactionSize;

        bytesLeftToSend -=
            len; // Calculate how many bytes will be left after we do this write

        // Write a portion of the payload to the bus.
        // Keep going until we've sent as many bytes as we can in this
        // transmission (x == len) or until we reach the end of the payload
        // ((startSpot + x) == (outgoingUBX->len))
        uint16_t x = len;
        if ((startSpot + x) >= (outgoingUBX->len))
          x = outgoingUBX->len - startSpot;

        if (writeBytes(&outgoingUBX->payload[startSpot], x) != x)
          return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

        startSpot += x;
      }

      // Finally, write any left-over bytes plus the checksum
      if (bytesLeftToSend == 1) {
        buf[0] = outgoingUBX->payload[startSpot];
        buf[1] = outgoingUBX->checksumA;
        buf[2] = outgoingUBX->checksumB;

        if (writeBytes(buf, 3) != 3)
          return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
      } else {
        buf[0] = outgoingUBX->checksumA;
        buf[1] = outgoingUBX->checksumB;

        if (writeBytes(buf, 2) != 2)
          return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
      }
    }

    return (SFE_UBLOX_STATUS_SUCCESS);
  }

  // Pretty prints the current ubxPacket
  void DevUBLOXGNSS::printPacket(ubxPacket * packet, bool alwaysPrintPayload) {
    // Only print the payload is ignoreThisPayload is false otherwise
    // we could be printing gibberish from beyond the end of packetBuf
    // (These two lines get rid of a pesky compiler warning)
    bool printPayload = (ignoreThisPayload == false);
    printPayload |= (alwaysPrintPayload == true);
  }

  // When messages from the class CFG are sent to the receiver, the receiver
  // will send an "acknowledge"(UBX - ACK - ACK) or a
  //"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on
  // whether or not the message was processed correctly.
  // Some messages from other classes also use the same acknowledgement
  // mechanism.

  // When we poll or get a setting, we will receive _both_ a config packet and
  // an ACK If the poll or get request is not valid, we will receive _only_ a
  // NACK

  // If we are trying to get or poll a setting, then packetCfg.len will be 0 or
  // 1 when the packetCfg is _sent_. If we poll the setting for a particular
  // port using UBX-CFG-PRT then .len will be 1 initially For all other gets or
  // polls, .len will be 0 initially
  //(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to
  // poll
  // the settings for a particular message - but we don't use that (currently))

  // If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still
  // be 0 or 1 after the NACK is received But if the get or poll is ACK'd, then
  // packetCfg.len will have been updated by the incoming data and will always
  // be at least 2

  // If we are going to set the value for a setting, then packetCfg.len will be
  // at least 3 when the packetCfg is _sent_.
  //(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

  // We need to think carefully about how interleaved PVT packets affect things.
  // It is entirely possible that our packetCfg and packetAck were received
  // successfully but while we are still in the "if (checkUblox() == true)" loop
  // a PVT packet is processed or _starts_ to arrive (remember that Serial data
  // can arrive very slowly).

  // Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid
  // packetCfg (module is responding with register content) Returns
  // SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid
  // packetCfg needed, module absorbs new register data) Returns
  // SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum
  // failure) Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was
  // not-acknowledged (NACK) Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a
  // checksum failure Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out Returns
  // SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg
  // but that the packetCfg has been
  //  or is currently being overwritten (remember that Serial data can arrive
  //  very slowly)
  sfe_ublox_status_e DevUBLOXGNSS::waitForACKResponse(
      ubxPacket * outgoingUBX, uint8_t requestedClass, uint8_t requestedID,
      uint16_t maxTime) {
    outgoingUBX->valid =
        SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or
                                               // NOT_VALID) when we receive a
                                               // response to the packet we sent
    packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX->classAndIDmatch =
        SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or
                                               // NOT_VALID) when we receive a
                                               // packet that matches the
                                               // requested class and ID
    packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    unsigned long startTime = millis();
    while (millis() < (startTime + (unsigned long)maxTime)) {
      if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) ==
          true) // See if new data is available. Process bytes as they come in.
      {
        // If both the outgoingUBX->classAndIDmatch and
        // packetAck.classAndIDmatch are VALID and outgoingUBX->valid is _still_
        // VALID and the class and ID _still_ match then we can be confident
        // that the data in outgoingUBX is valid
        if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
            (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
            (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
            (outgoingUBX->cls == requestedClass) &&
            (outgoingUBX->id == requestedID)) {
          return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and
                                                   // a correct ACK!
        }

        // We can be confident that the data packet (if we are going to get one)
        // will always arrive before the matching ACK. So if we sent a config
        // packet which only produces an ACK then outgoingUBX->classAndIDmatch
        // will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID. We
        // should not check outgoingUBX->valid, outgoingUBX->cls or
        // outgoingUBX->id as these may have been changed by an automatic
        // packet.
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) &&
                 (packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForACKResponse: no data and valid ACK after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
        }

        // If both the outgoingUBX->classAndIDmatch and
        // packetAck.classAndIDmatch are VALID but the outgoingUBX->cls or ID no
        // longer match then we can be confident that we had valid data but it
        // has been or is currently being overwritten by an automatic packet
        // (e.g. PVT). If (e.g.) a PVT packet is _being_ received:
        // outgoingUBX->valid will be NOT_DEFINED If (e.g.) a PVT packet _has
        // been_ received: outgoingUBX->valid will be VALID (or just possibly
        // NOT_VALID) So we cannot use outgoingUBX->valid as part of this check.
        // Note: the addition of packetBuf should make this check redundant!
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 (packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 ((outgoingUBX->cls != requestedClass) ||
                  (outgoingUBX->id != requestedID))) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForACKResponse: data being OVERWRITTEN after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (
              SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been
                                                  // or is being overwritten
        }

        // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and
        // outgoingUBX->classAndIDmatch are NOT_VALID then we can be confident
        // we have had a checksum failure on the data packet
        else if ((packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 (outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) &&
                 (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(F("waitForACKResponse: CRC failed after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
        }

        // If our packet was not-acknowledged (NACK) we do not receive a data
        // packet - we only get the NACK. So you would expect outgoingUBX->valid
        // and outgoingUBX->classAndIDmatch to still be NOT_DEFINED But if a
        // full PVT packet arrives afterwards outgoingUBX->valid could be VALID
        // (or just possibly NOT_VALID) but outgoingUBX->cls and outgoingUBX->id
        // would not match... So I think this is telling us we need a special
        // state for packetAck.classAndIDmatch to tell us the packet was
        // definitely NACK'd otherwise we are possibly just guessing... Note:
        // the addition of packetBuf changes the logic of this, but we'll leave
        // the code as is for now.
        else if (packetAck.classAndIDmatch ==
                 SFE_UBLOX_PACKET_NOTACKNOWLEDGED) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(F(
                "waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
        }

        // If the outgoingUBX->classAndIDmatch is VALID but the
        // packetAck.classAndIDmatch is NOT_VALID then the ack probably had a
        // checksum error. We will take a gamble and return DATA_RECEIVED. If we
        // were playing safe, we should return FAIL instead
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 (packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) &&
                 (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 (outgoingUBX->cls == requestedClass) &&
                 (outgoingUBX->id == requestedID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForACKResponse: VALID data and INVALID ACK "
                  "received after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and
                                                   // an invalid ACK!
        }

        // If the outgoingUBX->classAndIDmatch is NOT_VALID and the
        // packetAck.classAndIDmatch is NOT_VALID then we return a FAIL. This
        // must be a double checksum failure?
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) &&
                 (packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(F("waitForACKResponse: INVALID data and INVALID "
                                 "ACK received after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an
                                          // invalid ACK!
        }

        // If the outgoingUBX->classAndIDmatch is VALID and the
        // packetAck.classAndIDmatch is NOT_DEFINED then the ACK has not yet
        // been received and we should keep waiting for it
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 (packetAck.classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED)) {
          // if (_printDebug == true)
          // {
          //   _debugSerial.print(F("waitForACKResponse: valid data after "));
          //   _debugSerial.print(millis() - startTime);
          //   _debugSerial.println(F(" msec. Waiting for ACK."));
          // }
        }

      } // checkUbloxInternal == true

      delay(1); // Allow an RTOS to get an elbow in (#11)
    } // while (millis() < (startTime + (unsigned long)maxTime))

    // We have timed out...
    // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble
    // and return DATA_RECEIVED even though we did not get an ACK
    if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
        (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) &&
        (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
        (outgoingUBX->cls == requestedClass) &&
        (outgoingUBX->id == requestedID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true) {
        _debugSerial.print(
            F("waitForACKResponse: TIMEOUT with valid data after "));
        _debugSerial.print(millis() - startTime);
        _debugSerial.println(F(" msec. "));
      }
#endif
      return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But
                                               // no ACK!
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.print(F("waitForACKResponse: TIMEOUT after "));
      _debugSerial.print(millis() - startTime);
      _debugSerial.println(F(" msec."));
    }
#endif

    return (SFE_UBLOX_STATUS_TIMEOUT);
  }

  // For non-CFG queries no ACK is sent so we use this function
  // Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of
  // response data that has CLS/ID match to our query packet Returns
  // SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID
  // match to our query packet Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
  // Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg
  // but that the packetCfg has been
  //  or is currently being overwritten (remember that Serial data can arrive
  //  very slowly)
  sfe_ublox_status_e DevUBLOXGNSS::waitForNoACKResponse(
      ubxPacket * outgoingUBX, uint8_t requestedClass, uint8_t requestedID,
      uint16_t maxTime) {
    outgoingUBX->valid =
        SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or
                                               // NOT_VALID) when we receive a
                                               // response to the packet we sent
    packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    outgoingUBX->classAndIDmatch =
        SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or
                                               // NOT_VALID) when we receive a
                                               // packet that matches the
                                               // requested class and ID
    packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
    packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

    unsigned long startTime = millis();
    while (millis() - startTime < maxTime) {
      if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) ==
          true) // See if new data is available. Process bytes as they come in.
      {

        // If outgoingUBX->classAndIDmatch is VALID
        // and outgoingUBX->valid is _still_ VALID and the class and ID _still_
        // match then we can be confident that the data in outgoingUBX is valid
        if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
            (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) &&
            (outgoingUBX->cls == requestedClass) &&
            (outgoingUBX->id == requestedID)) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForNoACKResponse: valid data with CLS/ID match after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
        }

        // If the outgoingUBX->classAndIDmatch is VALID
        // but the outgoingUBX->cls or ID no longer match then we can be
        // confident that we had valid data but it has been or is currently
        // being overwritten by another packet (e.g. PVT). If (e.g.) a PVT
        // packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED If
        // (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be
        // VALID (or just possibly NOT_VALID) So we cannot use
        // outgoingUBX->valid as part of this check. Note: the addition of
        // packetBuf should make this check redundant!
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_VALID) &&
                 ((outgoingUBX->cls != requestedClass) ||
                  (outgoingUBX->id != requestedID))) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForNoACKResponse: data being OVERWRITTEN after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (
              SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been
                                                  // or is being overwritten
        }

        // If outgoingUBX->classAndIDmatch is NOT_DEFINED
        // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
        else if ((outgoingUBX->classAndIDmatch ==
                  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) &&
                 (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)) {
          // if (_printDebug == true)
          // {
          //   _debugSerial.print(F("waitForNoACKResponse: valid but UNWANTED
          //   data after ")); _debugSerial.print(millis() - startTime);
          //   _debugSerial.print(F(" msec. Class: 0x"));
          //   _debugSerial.print(outgoingUBX->cls, HEX);
          //   _debugSerial.print(F(" ID: 0x"));
          //   _debugSerial.print(outgoingUBX->id, HEX);
          // }
        }

        // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC
        // failure
        else if (outgoingUBX->classAndIDmatch ==
                 SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true) {
            _debugSerial.print(
                F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
            _debugSerial.print(millis() - startTime);
            _debugSerial.println(F(" msec"));
          }
#endif
          return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
        }
      }

      delay(1); // Allow an RTOS to get an elbow in (#11)
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.print(F("waitForNoACKResponse: TIMEOUT after "));
      _debugSerial.print(millis() - startTime);
      _debugSerial.println(F(" msec. No packet received."));
    }
#endif

    return (SFE_UBLOX_STATUS_TIMEOUT);
  }

  // Check if any callbacks are waiting to be processed
  void DevUBLOXGNSS::checkCallbacks(void) {
    if (checkCallbacksReentrant ==
        true) // Check for reentry (i.e. checkCallbacks has been called from
              // inside a callback)
      return;

    checkCallbacksReentrant = true;

    if (packetUBXNAVPVT !=
        nullptr) // If RAM has been allocated for message storage
      if (packetUBXNAVPVT->callbackData !=
          nullptr) // If RAM has been allocated for the copy of the data
        if (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid ==
            true) // If the copy of the data is valid
        {
          if (packetUBXNAVPVT->callbackPointerPtr !=
              nullptr) // If the pointer to the callback has been defined
          {
            packetUBXNAVPVT->callbackPointerPtr(
                packetUBXNAVPVT->callbackData); // Call the callback
          }
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid =
              false; // Mark the data as stale
        }

    checkCallbacksReentrant = false;
  }

  //=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  // Changes the I2C address that the u-blox module responds to
  // 0x42 is the default but can be changed with this command
  // Note: the module stores the address in shifted format - not unshifted.
  // We need to shift left by one bit to compensate.
  bool DevUBLOXGNSS::setI2CAddress(uint8_t deviceAddress, uint8_t layer,
                                   uint16_t maxWait) {
    return setVal8(UBLOX_CFG_I2C_ADDRESS, deviceAddress << 1, layer,
                   maxWait); // Change the I2C address. Shift left by one bit.
  }

  // Changes the serial baud rate of the u-blox module, can't return
  // success/fail 'cause ACK from modem is lost due to baud rate change
  bool DevUBLOXGNSS::setSerialRate(uint32_t baudrate, uint8_t uartPort,
                                   uint8_t layer, uint16_t maxWait) {
    if (uartPort == COM_PORT_UART1)
      return setVal32(UBLOX_CFG_UART1_BAUDRATE, baudrate, layer, maxWait);
    else if (uartPort == COM_PORT_UART2)
      return setVal32(UBLOX_CFG_UART2_BAUDRATE, baudrate, layer, maxWait);
    else
      return false;
  }

  // Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
  bool DevUBLOXGNSS::setI2COutput(uint8_t comSettings, uint8_t layer,
                                  uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_I2COUTPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    return result;
  }
  bool DevUBLOXGNSS::setUART1Output(uint8_t comSettings, uint8_t layer,
                                    uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_UART1OUTPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_UART1OUTPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_UART1OUTPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    return result;
  }
  bool DevUBLOXGNSS::setUART2Output(uint8_t comSettings, uint8_t layer,
                                    uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_UART2OUTPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_UART2OUTPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    return result;
  }
  bool DevUBLOXGNSS::setUSBOutput(uint8_t comSettings, uint8_t layer,
                                  uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_USBOUTPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_USBOUTPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_USBOUTPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    return result;
  }
  bool DevUBLOXGNSS::setSPIOutput(uint8_t comSettings, uint8_t layer,
                                  uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_SPIOUTPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_SPIOUTPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_SPIOUTPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    return result;
  }

  // Configure a port to input UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool DevUBLOXGNSS::setI2CInput(uint8_t comSettings, uint8_t layer,
                                 uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_I2CINPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_I2CINPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_I2CINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1,
        layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    result |= setVal8(
        UBLOX_CFG_I2CINPROT_SPARTN,
        (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support SPARTN
    return result;
  }
  bool DevUBLOXGNSS::setUART1Input(uint8_t comSettings, uint8_t layer,
                                   uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_UART1INPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_UART1INPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_UART1INPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    result |= setVal8(
        UBLOX_CFG_UART1INPROT_SPARTN,
        (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support SPARTN
    return result;
  }
  bool DevUBLOXGNSS::setUART2Input(uint8_t comSettings, uint8_t layer,
                                   uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_UART2INPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_UART2INPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_UART2INPROT_RTCM3X,
        (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    result |= setVal8(
        UBLOX_CFG_UART2INPROT_SPARTN,
        (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support SPARTN
    return result;
  }
  bool DevUBLOXGNSS::setUSBInput(uint8_t comSettings, uint8_t layer,
                                 uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_USBINPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_USBINPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_USBINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1,
        layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    result |= setVal8(
        UBLOX_CFG_USBINPROT_SPARTN,
        (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support SPARTN
    return result;
  }
  bool DevUBLOXGNSS::setSPIInput(uint8_t comSettings, uint8_t layer,
                                 uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_SPIINPROT_UBX,
                            (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
    result &= addCfgValset8(UBLOX_CFG_SPIINPROT_NMEA,
                            (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
    result &= sendCfgValset(maxWait);
    result |= setVal8(
        UBLOX_CFG_SPIINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1,
        layer,
        maxWait); // This will be NACK'd if the module does not support RTCM3
    result |= setVal8(
        UBLOX_CFG_SPIINPROT_SPARTN,
        (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer,
        maxWait); // This will be NACK'd if the module does not support SPARTN
    return result;
  }

  // Reset to defaults

  void DevUBLOXGNSS::factoryReset() {
    // Copy default settings to permanent
    // Note: this does not load the permanent configuration into the current
    // configuration. Calling factoryDefault() will do that.
    uint8_t clearMemory[13] = {0xff, 0xff, 0xff, 0xff, 0, 0,   0,
                               0,    0,    0,    0,    0, 0xff};
    cfgCfg(clearMemory, 13, 0);
    hardReset(); // cause factory default config to actually be loaded and used
                 // cleanly
  }

  void DevUBLOXGNSS::hardReset() {
    // Issue hard reset
    uint8_t softwareResetGNSS[4] = {0xff, 0xff, 0, 0};
    cfgRst(softwareResetGNSS, 4);
  }

  void DevUBLOXGNSS::softwareResetGNSSOnly() {
    // Issue controlled software reset (GNSS only)
    uint8_t softwareResetGNSS[4] = {0, 0, 0x02, 0};
    cfgRst(softwareResetGNSS, 4);
  }

  void DevUBLOXGNSS::softwareEnableGNSS(bool enable) {
    // Issue controlled software reset (GNSS only)
    uint8_t softwareEnable[4] = {0, 0, 0, 0};
    softwareEnable[2] =
        enable ? 0x09 : 0x08; // 0x09 = start GNSS, 0x08 = stop GNSS
    cfgRst(softwareEnable, 4);
  }

  void DevUBLOXGNSS::cfgRst(uint8_t *data, uint8_t len) {
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_RST;
    packetCfg.len = len;
    packetCfg.startingSpot = 0;
    for (uint8_t i = 0; i < len; i++)
      payloadCfg[i] = *data++;
    sendCommand(&packetCfg, 0); // don't expect ACK
  }

  // Reset module to factory defaults
  // This still works but it is the old way of configuring ublox modules. See
  // getVal and setVal for the new methods
  bool DevUBLOXGNSS::factoryDefault(uint16_t maxWait) {
    uint8_t configSelective[12];

    // Clear packet payload
    memset(configSelective, 0, 12);

    configSelective[0] =
        0xFF; // Set any bit in the clearMask field to clear saved config
    configSelective[1] = 0xFF;
    configSelective[8] =
        0xFF; // Set any bit in the loadMask field to discard current config and
              // rebuild from lower non-volatile memory layers
    configSelective[9] = 0xFF;

    return (cfgCfg(configSelective, 12, maxWait));
  }

  bool DevUBLOXGNSS::cfgCfg(uint8_t *data, uint8_t len, uint16_t maxWait) {
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_CFG;
    packetCfg.len = len;
    packetCfg.startingSpot = 0;
    for (uint8_t i = 0; i < len; i++)
      payloadCfg[i] = *data++;
    return (sendCommand(&packetCfg, maxWait) ==
            SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  }

  // Set the DGNSS differential mode
  bool DevUBLOXGNSS::setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode,
                                           uint8_t layer, uint16_t maxWait) {
    return setVal8(UBLOX_CFG_NAVHPG_DGNSSMODE, (uint8_t)dgnssMode, layer,
                   maxWait);
  }

  // Module Protocol Version

  // Get the current protocol version of the u-blox module we're communicating
  // with This is helpful when deciding if we should call the high-precision
  // Lat/Long (HPPOSLLH) or the regular (POSLLH)
  uint8_t DevUBLOXGNSS::getProtocolVersionHigh(uint16_t maxWait) {
    if (!prepareModuleInfo(maxWait))
      return 0;
    return (moduleSWVersion->protocolVersionHigh);
  }
  uint8_t DevUBLOXGNSS::getProtocolVersionLow(uint16_t maxWait) {
    if (!prepareModuleInfo(maxWait))
      return 0;
    return (moduleSWVersion->protocolVersionLow);
  }

  // Get the firmware version of the u-blox module we're communicating with
  uint8_t DevUBLOXGNSS::getFirmwareVersionHigh(uint16_t maxWait) {
    if (!prepareModuleInfo(maxWait))
      return 0;
    return (moduleSWVersion->firmwareVersionHigh);
  }
  uint8_t DevUBLOXGNSS::getFirmwareVersionLow(uint16_t maxWait) {
    if (!prepareModuleInfo(maxWait))
      return 0;
    return (moduleSWVersion->firmwareVersionLow);
  }

  // Get the firmware type
  const char *DevUBLOXGNSS::getFirmwareType(uint16_t maxWait) {
    static const char unknownFirmware[4] = {'T', 'B', 'D', '\0'};
    if (!prepareModuleInfo(maxWait))
      return unknownFirmware;
    return ((const char *)moduleSWVersion->firmwareType);
  }

  // Get the module name
  const char *DevUBLOXGNSS::getModuleName(uint16_t maxWait) {
    static const char unknownModule[4] = {'T', 'B', 'D', '\0'};
    if (!prepareModuleInfo(maxWait))
      return unknownModule;
    return ((const char *)moduleSWVersion->moduleName);
  }

  // PRIVATE: Common code to initialize moduleSWVersion
  bool DevUBLOXGNSS::prepareModuleInfo(uint16_t maxWait) {
    if (moduleSWVersion == nullptr)
      initModuleSWVersion(); // Check that RAM has been allocated for the SW
                             // version
    if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (moduleSWVersion->moduleQueried == false)
      getModuleInfo(maxWait);

    return moduleSWVersion->moduleQueried;
  }

  // Get the current protocol version of the u-blox module we're communicating
  // with This is helpful when deciding if we should call the high-precision
  // Lat/Long (HPPOSLLH) or the regular (POSLLH)
  bool DevUBLOXGNSS::getProtocolVersion(
      uint16_t maxWait) // Old name - deprecated
  {
    return getModuleInfo(maxWait);
  }

  bool DevUBLOXGNSS::getModuleInfo(uint16_t maxWait) {
    if (moduleSWVersion == nullptr)
      initModuleSWVersion(); // Check that RAM has been allocated for the SW
                             // version
    if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
      return (false);

    // Send packet with only CLS and ID, length of zero. This will cause the
    // module to respond with the contents of that CLS/ID.
    packetCfg.cls = UBX_CLASS_MON;
    packetCfg.id = UBX_MON_VER;

    packetCfg.len = 0;
    packetCfg.startingSpot =
        40; // Start at first "extended software information" string

    if (sendCommand(&packetCfg, maxWait) !=
        SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
      return (false);                   // If command send fails then bail

    // Payload should now contain ~220 characters (depends on module type)

    // We will step through the payload looking at each extension field of 30
    // bytes
    char *ptr;
    uint8_t fwProtMod =
        0; // Flags to show if we extracted the FWVER, PROTVER and MOD data
    for (uint8_t extensionNumber = 0;
         extensionNumber < ((packetCfg.len - 40) / 30); extensionNumber++) {
      // Check for FWVER (should be in extension 1)
      ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "FWVER=");
      if (ptr != nullptr) {
        ptr += strlen("FWVER="); // Point to the firmware type (HPG etc.)
        int i = 0;
        while ((i < firmwareTypeLen) && (*ptr != '\0') &&
               (*ptr != ' ')) // Extract the firmware type (3-7 chars)
          moduleSWVersion->firmwareType[i++] = *ptr++;
        moduleSWVersion->firmwareType[i] = '\0'; // NULL-terminate

        if (*ptr == ' ')
          ptr++; // Skip the space

        int firmwareHi = 0;
        int firmwareLo = 0;
        int scanned = sscanf(ptr, "%d.%d", &firmwareHi, &firmwareLo);
        if (scanned ==
            2) // Check we extracted the firmware version successfully
        {
          moduleSWVersion->firmwareVersionHigh = firmwareHi;
          moduleSWVersion->firmwareVersionLow = firmwareLo;
          fwProtMod |= 0x01; // Record that we got the FWVER
        }
      }
      // Check for PROTVER (should be in extension 2)
      ptr =
          strstr((const char *)&payloadCfg[(30 * extensionNumber)], "PROTVER=");
      if (ptr != nullptr) {
        ptr += strlen("PROTVER="); // Point to the protocol version
        int protHi = 0;
        int protLo = 0;
        int scanned = sscanf(ptr, "%d.%d", &protHi, &protLo);
        if (scanned ==
            2) // Check we extracted the firmware version successfully
        {
          moduleSWVersion->protocolVersionHigh = protHi;
          moduleSWVersion->protocolVersionLow = protLo;
          fwProtMod |= 0x02; // Record that we got the PROTVER
        }
      }
      // Check for MOD (should be in extension 3)
      // Note: see issue #55. It appears that the UBX-M10050-KB chip does not
      // report MOD
      ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "MOD=");
      if (ptr != nullptr) {
        ptr += strlen("MOD="); // Point to the module name
        int i = 0;
        while ((i < moduleNameMaxLen) && (*ptr != '\0') &&
               (*ptr != ' ')) // Copy the module name
          moduleSWVersion->moduleName[i++] = *ptr++;
        moduleSWVersion->moduleName[i] = '\0'; // NULL-terminate
        fwProtMod |= 0x04;                     // Record that we got the MOD
      }
    }

    if ((fwProtMod & 0x04) == 0) // Is MOD missing?
    {
      strncpy(moduleSWVersion->moduleName, "NONE", moduleNameMaxLen);
      fwProtMod |= 0x04; // Record that we updated the MOD
    }

    if (fwProtMod == 0x07) // Did we extract all three?
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true) {
        _debugSerial.print(F("getModuleInfo: FWVER: "));
        _debugSerial.print(moduleSWVersion->firmwareVersionHigh);
        _debugSerial.print(F("."));
        _debugSerial.println(moduleSWVersion->firmwareVersionLow);
        _debugSerial.print(F("getModuleInfo: PROTVER: "));
        _debugSerial.print(moduleSWVersion->protocolVersionHigh);
        _debugSerial.print(F("."));
        _debugSerial.println(moduleSWVersion->protocolVersionLow);
        _debugSerial.print(F("getModuleInfo: MOD: "));
        _debugSerial.println(moduleSWVersion->moduleName);
      }
#endif

      moduleSWVersion->moduleQueried = true; // Mark this data as new

      return (true);
    }

    return (false); // We failed
  }

  // PRIVATE: Allocate RAM for moduleSWVersion and initialize it
  bool DevUBLOXGNSS::initModuleSWVersion() {
    moduleSWVersion = new moduleSWVersion_t; // Allocate RAM for the main struct
    if (moduleSWVersion == nullptr) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("initModuleSWVersion: RAM alloc failed!"));
#endif
      return (false);
    }
    moduleSWVersion->protocolVersionHigh = 0; // Clear the contents
    moduleSWVersion->protocolVersionLow = 0;
    moduleSWVersion->firmwareVersionHigh = 0;
    moduleSWVersion->firmwareVersionLow = 0;
    moduleSWVersion->firmwareType[0] = 0;
    moduleSWVersion->moduleName[0] = 0;
    moduleSWVersion->moduleQueried = false;
    return (true);
  }

  // Powers off the GPS device for a given duration to reduce power consumption.
  // NOTE: Querying the device before the duration is complete, for example by
  // "getLatitude()" will wake it up! Returns true if command has not been not
  // acknowledged. Returns false if command has not been acknowledged or maxWait
  // = 0.
  bool DevUBLOXGNSS::powerOff(uint32_t durationInMs, uint16_t maxWait) {
    // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.print(F("Powering off for "));
      _debugSerial.print(durationInMs);
      _debugSerial.println(" ms");
    }
#endif

    // Power off device using UBX-RXM-PMREQ
    packetCfg.cls = UBX_CLASS_RXM; // 0x02
    packetCfg.id = UBX_RXM_PMREQ;  // 0x41
    packetCfg.len = 8;
    packetCfg.startingSpot = 0;

    // duration
    // big endian to little endian, switch byte order
    for (uint8_t i = 0; i < 4; i++)
      payloadCfg[i] = durationInMs >> (8 * i); // Value

    payloadCfg[4] = 0x02; // Flags : set the backup bit
    payloadCfg[5] = 0x00; // Flags
    payloadCfg[6] = 0x00; // Flags
    payloadCfg[7] = 0x00; // Flags

    if (maxWait != 0) {
      // check for "not acknowledged" command
      return (sendCommand(&packetCfg, maxWait) !=
              SFE_UBLOX_STATUS_COMMAND_NACK);
    } else {
      sendCommand(&packetCfg, maxWait);
      return false; // can't tell if command not acknowledged if maxWait = 0
    }
  }

  // Powers off the GPS device for a given duration to reduce power consumption.
  // While powered off it can be woken up by creating a falling or rising
  // voltage edge on the specified pin. NOTE: The GPS seems to be sensitve to
  // signals on the pins while powered off. Works best when Microcontroller is
  // in deepsleep. NOTE: Querying the device before the duration is complete,
  // for example by "getLatitude()" will wake it up! Returns true if command has
  // not been not acknowledged. Returns false if command has not been
  // acknowledged or maxWait = 0.
  bool DevUBLOXGNSS::powerOffWithInterrupt(
      uint32_t durationInMs, uint32_t wakeupSources, bool forceWhileUsb,
      uint16_t maxWait) {
    // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.print(F("Powering off for "));
      _debugSerial.print(durationInMs);
      _debugSerial.println(" ms");
    }
#endif

    // Power off device using UBX-RXM-PMREQ
    packetCfg.cls = UBX_CLASS_RXM; // 0x02
    packetCfg.id = UBX_RXM_PMREQ;  // 0x41
    packetCfg.len = 16;
    packetCfg.startingSpot = 0;

    payloadCfg[0] = 0x00; // message version

    // bytes 1-3 are reserved - and must be set to zero
    payloadCfg[1] = 0x00;
    payloadCfg[2] = 0x00;
    payloadCfg[3] = 0x00;

    // duration
    // big endian to little endian, switch byte order
    for (uint8_t i = 0; i < 4; i++)
      payloadCfg[4 + i] = durationInMs >> (8 * i); // Value

    // flags

    // disables USB interface when powering off, defaults to true
    if (forceWhileUsb) {
      payloadCfg[8] = 0x06; // force | backup
    } else {
      payloadCfg[8] = 0x02; // backup only (leave the force bit clear - module
                            // will stay on if USB is connected)
    }

    payloadCfg[9] = 0x00;
    payloadCfg[10] = 0x00;
    payloadCfg[11] = 0x00;

    // wakeUpSources

    // wakeupPin mapping, defaults to VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0

    // Possible values are:
    // VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX
    // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0
    // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1
    // VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS

    for (uint8_t i = 0; i < 4; i++)
      payloadCfg[12 + i] = wakeupSources >> (8 * i); // Value

    if (maxWait != 0) {
      // check for "not acknowledged" command
      return (sendCommand(&packetCfg, maxWait) !=
              SFE_UBLOX_STATUS_COMMAND_NACK);
    } else {
      sendCommand(&packetCfg, maxWait);
      return false; // can't tell if command not acknowledged if maxWait = 0
    }
  }

  // Dynamic Platform Model

  // Change the dynamic platform model using UBX-CFG-NAV5
  // Possible values are:
  // PORTABLE,STATIONARY,PEDESTRIAN,AUTOMOTIVE,SEA,
  // AIRBORNE1g,AIRBORNE2g,AIRBORNE4g,WRIST,BIKE
  // WRIST is not supported in protocol versions less than 18
  // BIKE is supported in protocol versions 19.2
  bool DevUBLOXGNSS::setDynamicModel(dynModel newDynamicModel, uint8_t layer,
                                     uint16_t maxWait) {
    return setVal8(UBLOX_CFG_NAVSPG_DYNMODEL, (uint8_t)newDynamicModel, layer,
                   maxWait);
  }

  // Get the dynamic platform model using UBX-CFG-NAV5
  // Returns DYN_MODEL_UNKNOWN (255) if the sendCommand fails
  uint8_t DevUBLOXGNSS::getDynamicModel(uint8_t layer, uint16_t maxWait) {
    uint8_t model;

    if (!getVal8(UBLOX_CFG_NAVSPG_DYNMODEL, &model, layer, maxWait))
      return (DYN_MODEL_UNKNOWN);

    return (model); // Return the dynamic model
  }

  uint32_t DevUBLOXGNSS::getEnableGNSSConfigKey(sfe_ublox_gnss_ids_e id) {
    const uint32_t gnssConfigKeys[(uint8_t)SFE_UBLOX_GNSS_ID_UNKNOWN] = {
        UBLOX_CFG_SIGNAL_GPS_ENA,
        UBLOX_CFG_SIGNAL_SBAS_ENA,
        UBLOX_CFG_SIGNAL_GAL_ENA,
        UBLOX_CFG_SIGNAL_BDS_ENA,
        0, // IMES has no ENA key
        UBLOX_CFG_SIGNAL_QZSS_ENA,
        UBLOX_CFG_SIGNAL_GLO_ENA};

    if (id >= SFE_UBLOX_GNSS_ID_UNKNOWN)
      return 0;
    else
      return (gnssConfigKeys[(uint8_t)id]);
  }

  // Enable/Disable individual GNSS systems using UBX-CFG-GNSS
  bool DevUBLOXGNSS::enableGNSS(bool enable, sfe_ublox_gnss_ids_e id,
                                uint8_t layer, uint16_t maxWait) {
    uint32_t key = getEnableGNSSConfigKey(id);
    return (setVal8(key, enable ? 1 : 0, layer, maxWait));
  }

  // Check if an individual GNSS system is enabled
  bool DevUBLOXGNSS::isGNSSenabled(sfe_ublox_gnss_ids_e id, bool *enabled,
                                   uint8_t layer, uint16_t maxWait) {
    uint32_t key = getEnableGNSSConfigKey(id);
    return (getVal8(key, (uint8_t *)enabled, layer, maxWait));
  }
  bool DevUBLOXGNSS::isGNSSenabled(sfe_ublox_gnss_ids_e id, uint8_t layer,
                                   uint16_t maxWait) // Unsafe
  {
    uint32_t key = getEnableGNSSConfigKey(id);
    uint8_t enabled;
    getVal8(key, &enabled, layer, maxWait);
    return ((bool)enabled);
  }

  // Reset ESF automatic IMU-mount alignment

  // Enable/disable esfAutoAlignment

  // Get the RF information using UBX_MON_RF

  // Get the extended hardware status using UBX_MON_HW2

  // UBX-CFG-NAVX5 - get/set the ackAiding byte. If ackAiding is 1, UBX-MGA-ACK
  // messages will be sent by the module to acknowledge the MGA data
  uint8_t DevUBLOXGNSS::getAckAiding(
      uint8_t layer,
      uint16_t maxWait) // Get the ackAiding byte - returns
                        // 255 if the sendCommand fails
  {
    uint8_t enabled;
    bool success =
        getVal8(UBLOX_CFG_NAVSPG_ACKAIDING, &enabled, layer, maxWait);
    if (success)
      return enabled;
    return 255;
  }
  bool DevUBLOXGNSS::setAckAiding(uint8_t ackAiding, uint8_t layer,
                                  uint16_t maxWait) // Set the ackAiding byte
  {
    return setVal8(UBLOX_CFG_NAVSPG_ACKAIDING, ackAiding, layer, maxWait);
  }

  // AssistNow Autonomous support
  // UBX-CFG-NAVX5 - get the AssistNow Autonomous configuration (aopCfg) -
  // returns 255 if the sendCommand fails
  uint8_t DevUBLOXGNSS::getAopCfg(uint8_t layer, uint16_t maxWait) {
    uint8_t enabled;
    bool success = getVal8(UBLOX_CFG_ANA_USE_ANA, &enabled, layer, maxWait);
    if (success)
      return enabled;
    return 255;
  }
  // Set the aopCfg byte and the aopOrdMaxErr word
  bool DevUBLOXGNSS::setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr,
                               uint8_t layer, uint16_t maxWait) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(UBLOX_CFG_ANA_USE_ANA, aopCfg);
    if ((aopOrbMaxErr >= 5) &&
        (aopOrbMaxErr <= 1000)) // Maximum acceptable (modeled) orbit error in
                                // m. Range is from 5 to 1000.
      result &= addCfgValset16(UBLOX_CFG_ANA_ORBMAXERR, aopOrbMaxErr);
    result &= sendCfgValset(maxWait);
    return result;
  }

  // SPARTN dynamic keys
  //"When the receiver boots, the host should send 'current' and 'next' keys in
  // one message." - Use setDynamicSPARTNKeys for this. "Every time the
  // 'current' key is expired, 'next' takes its place." "Therefore the host
  // should then retrieve the new 'next' key and send only that." - Use
  // setDynamicSPARTNKey for this. The key can be provided in binary (uint8_t)
  // format or in ASCII Hex (char) format, but in both cases keyLengthBytes
  // _must_ represent the binary key length in bytes.

  // Support for SPARTN parsing
  // Mostly stolen from
  // https://github.com/u-blox/ubxlib/blob/master/common/spartn/src/u_spartn_crc.c

  // Parse SPARTN data

  // Get the unique chip ID using UBX-SEC-UNIQID
  // The ID is five bytes on the F9 and M9 (version 1) but six bytes on the M10
  // (version 2)
  bool DevUBLOXGNSS::getUniqueChipId(UBX_SEC_UNIQID_data_t * data,
                                     uint16_t maxWait) {
    if (data == nullptr) // Check if the user forgot to include the data pointer
      return (false);    // Bail

    packetCfg.cls = UBX_CLASS_SEC;
    packetCfg.id = UBX_SEC_UNIQID;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    if (sendCommand(&packetCfg, maxWait) !=
        SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
      return (false);

    // Extract the data
    data->version = extractByte(&packetCfg, 0);
    for (uint8_t i = 0; i < 5; i++)
      data->uniqueId[i] = extractByte(&packetCfg, i + 4);

    // The ID is five bytes on the F9 and M9 (version 1) but six bytes on the
    // M10 (version 2)
    if ((data->version == 2) && (packetCfg.len == UBX_SEC_UNIQID_LEN_VERSION2))
      data->uniqueId[5] = extractByte(&packetCfg, 9);
    else
      data->uniqueId[5] = 0;

    return (true);
  }
  // Get the unique chip ID as text

  // CONFIGURATION INTERFACE (protocol v27 and above)

  // Given a key, load the payload with data that can then be extracted to 8,
  // 16, or 32 bits This function takes a full 32-bit key Default layer is RAM
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  sfe_ublox_status_e DevUBLOXGNSS::getVal(uint32_t key, uint8_t layer,
                                          uint16_t maxWait) {
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALGET;
    packetCfg.len = 4 + 4 * 1; // While multiple keys are allowed, we will send
                               // only one key at a time
    packetCfg.startingSpot = 0;

    // Clear packet payload
    memset(payloadCfg, 0, packetCfg.len);

    // VALGET uses different memory layer definitions to VALSET
    // because it can only return the value for one layer.
    // So we need to fiddle the layer here.
    // And just to complicate things further, the ZED-F9P only responds
    // correctly to layer 0 (RAM) and layer 7 (Default)!
    uint8_t getLayer = VAL_LAYER_DEFAULT; // 7 is the "Default Layer"
    if (layer == VAL_LAYER_RAM)           // Did the user request the RAM layer?
    {
      getLayer = 0;                    // Layer 0 is RAM
    } else if (layer == VAL_LAYER_BBR) // Did the user request the BBR layer?
    {
      getLayer = 1; // Layer 1 is BBR
    } else if (layer ==
               VAL_LAYER_FLASH) // Did the user request the Flash layer?
    {
      getLayer = 2; // Layer 2 is Flash
    }

    payloadCfg[0] = 0;        // Message Version - set to 0
    payloadCfg[1] = getLayer; // Layer

    // Load key into outgoing payload
    key &= ~UBX_CFG_SIZE_MASK;    // Mask off the size identifer bits
    payloadCfg[4] = key >> 8 * 0; // Key LSB
    payloadCfg[5] = key >> 8 * 1;
    payloadCfg[6] = key >> 8 * 2;
    payloadCfg[7] = key >> 8 * 3;

    // Send VALGET command with this key

    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true) {
      _debugSerial.print(F("getVal: sendCommand returned: "));
      _debugSerial.println(statusString(retVal));
    }
#endif

    // Verify the response is the correct length as compared to what the user
    // called (did the module respond with 8-bits but the user called getVal32?)
    // Response is 8 bytes plus cfg data
    // if(packet->len > 8+1)

    // The response is now sitting in payload, ready for extraction
    return (retVal);
  }

  // Given a key, return its value
  // This function takes a full 32-bit key
  // Default layer is RAM
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::getVal8(uint32_t key, uint8_t *val, uint8_t layer,
                             uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractByte(&packetCfg, 8);
    return result;
  }
  uint8_t DevUBLOXGNSS::getVal8(uint32_t key, uint8_t layer,
                                uint16_t maxWait) // Unsafe overload
  {
    uint8_t result = 0;
    getVal8(key, &result, layer, maxWait);
    return result;
  }
  bool DevUBLOXGNSS::getValSigned8(uint32_t key, int8_t *val, uint8_t layer,
                                   uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractSignedChar(&packetCfg, 8);
    return result;
  }

  bool DevUBLOXGNSS::getVal16(uint32_t key, uint16_t *val, uint8_t layer,
                              uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractInt(&packetCfg, 8);
    return result;
  }
  uint16_t DevUBLOXGNSS::getVal16(uint32_t key, uint8_t layer,
                                  uint16_t maxWait) // Unsafe overload
  {
    uint16_t result = 0;
    getVal16(key, &result, layer, maxWait);
    return result;
  }
  bool DevUBLOXGNSS::getValSigned16(uint32_t key, int16_t *val, uint8_t layer,
                                    uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractSignedInt(&packetCfg, 8);
    return result;
  }

  bool DevUBLOXGNSS::getVal32(uint32_t key, uint32_t *val, uint8_t layer,
                              uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractLong(&packetCfg, 8);
    return result;
  }
  uint32_t DevUBLOXGNSS::getVal32(uint32_t key, uint8_t layer,
                                  uint16_t maxWait) // Unsafe overload
  {
    uint32_t result = 0;
    getVal32(key, &result, layer, maxWait);
    return result;
  }
  bool DevUBLOXGNSS::getValSigned32(uint32_t key, int32_t *val, uint8_t layer,
                                    uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractSignedLong(&packetCfg, 8);
    return result;
  }

  bool DevUBLOXGNSS::getVal64(uint32_t key, uint64_t *val, uint8_t layer,
                              uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractLongLong(&packetCfg, 8);
    return result;
  }
  uint64_t DevUBLOXGNSS::getVal64(uint32_t key, uint8_t layer,
                                  uint16_t maxWait) // Unsafe overload
  {
    uint64_t result = 0;
    getVal64(key, &result, layer, maxWait);
    return result;
  }
  bool DevUBLOXGNSS::getValSigned64(uint32_t key, int64_t *val, uint8_t layer,
                                    uint16_t maxWait) {
    bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractSignedLongLong(&packetCfg, 8);
    return result;
  }

  bool DevUBLOXGNSS::getValFloat(uint32_t key, float *val, uint8_t layer,
                                 uint16_t maxWait) {
    if (sizeof(float) != 4)
      return false;

    bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractFloat(&packetCfg, 8);
    return result;
  }

  bool DevUBLOXGNSS::getValDouble(uint32_t key, double *val, uint8_t layer,
                                  uint16_t maxWait) {
    if (sizeof(double) != 8)
      return false;

    bool result = getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED;
    if (result)
      *val = extractDouble(&packetCfg, 8);
    return result;
  }

  // Given a key, set a N-byte value
  // This function takes a full 32-bit key
  // Default layer is RAM+BBR
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::setValN(uint32_t key, uint8_t *value, uint8_t N,
                             uint8_t layer, uint16_t maxWait) {
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4 + 4 + N; // 4 byte header, 4 byte key ID, N bytes of value
    packetCfg.startingSpot = 0;

    // Clear packet payload
    memset(payloadCfg, 0, packetCfg.len);

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // Load key into outgoing payload
    key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
    for (uint8_t i = 0; i < 4; i++)
      payloadCfg[i + 4] = key >> (8 * i); // Key

    // Load user's value
    for (uint8_t i = 0; i < N; i++)
      payloadCfg[i + 8] = *value++;

    // Send VALSET command with this key and value
    return (sendCommand(&packetCfg, maxWait) ==
            SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  }

  // Given a key, set an 8-bit value
  // This function takes a full 32-bit key
  // Default layer is RAM+BBR
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::setVal8(uint32_t key, uint8_t value, uint8_t layer,
                             uint16_t maxWait) {
    uint8_t val[1] = {value};
    return (setValN(key, val, 1, layer, maxWait));
  }
  bool DevUBLOXGNSS::setValSigned8(uint32_t key, int8_t value, uint8_t layer,
                                   uint16_t maxWait) {
    unsignedSigned8 converter;
    converter.signed8 = value;
    return (setVal8(key, converter.unsigned8, layer, maxWait));
  }

  // Given a key, set a 16-bit value
  // This function takes a full 32-bit key
  // Default layer is RAM+BBR
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer,
                              uint16_t maxWait) {
    uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
    return (setValN(key, val, 2, layer, maxWait));
  }
  bool DevUBLOXGNSS::setValSigned16(uint32_t key, int16_t value, uint8_t layer,
                                    uint16_t maxWait) {
    unsignedSigned16 converter;
    converter.signed16 = value;
    return (setVal16(key, converter.unsigned16, layer, maxWait));
  }

  // Given a key, set a 32-bit value
  // This function takes a full 32-bit key
  // Default layer is RAM+BBR
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::setVal32(uint32_t key, uint32_t value, uint8_t layer,
                              uint16_t maxWait) {
    uint8_t val[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8),
                      (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    return (setValN(key, val, 4, layer, maxWait));
  }
  bool DevUBLOXGNSS::setValSigned32(uint32_t key, int32_t value, uint8_t layer,
                                    uint16_t maxWait) {
    unsignedSigned32 converter;
    converter.signed32 = value;
    return (setVal32(key, converter.unsigned32, layer, maxWait));
  }

  // Given a key, set a 64-bit value
  // This function takes a full 32-bit key
  // Default layer is RAM+BBR
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::setVal64(uint32_t key, uint64_t value, uint8_t layer,
                              uint16_t maxWait) {
    uint8_t val[8];

    // Load user's value
    for (uint8_t i = 0; i < 8; i++)
      val[i] = (uint8_t)(value >> (8 * i)); // Value

    return (setValN(key, val, 8, layer, maxWait));
  }
  bool DevUBLOXGNSS::setValSigned64(uint32_t key, int64_t value, uint8_t layer,
                                    uint16_t maxWait) {
    unsignedSigned64 converter;
    converter.signed64 = value;
    return (setVal64(key, converter.unsigned64, layer, maxWait));
  }

  bool DevUBLOXGNSS::setValFloat(uint32_t key, float value, uint8_t layer,
                                 uint16_t maxWait) {
    if (sizeof(float) != 4) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("setValFloat not supported!"));
      }
#endif
      return false;
    }
    unsigned32float converter;
    converter.flt = value;
    return (setVal32(key, converter.unsigned32, layer, maxWait));
  }

  bool DevUBLOXGNSS::setValDouble(uint32_t key, double value, uint8_t layer,
                                  uint16_t maxWait) {
    if (sizeof(double) != 8) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("setValDouble not supported!"));
      }
#endif
      return false;
    }
    unsigned64double converter;
    converter.dbl = value;
    return (setVal64(key, converter.unsigned64, layer, maxWait));
  }

  // Start defining a new (empty) UBX-CFG-VALSET ubxPacket
  // Configuration of modern u-blox modules is now done via
  // getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
  bool DevUBLOXGNSS::newCfgValset(uint8_t layer) {
    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_VALSET;
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;

    _numCfgKeys = 0;

    // Clear all of packet payload
    memset(payloadCfg, 0, packetCfgPayloadSize);

    payloadCfg[0] = 0;     // Message Version - set to 0
    payloadCfg[1] = layer; // By default we ask for the BBR layer

    // All done
    return (true);
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and N-byte value
  bool DevUBLOXGNSS::addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N) {
    if ((_autoSendAtSpaceRemaining > 0) &&
        (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining))) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("addCfgValsetN: autosend"));
#endif
      if (sendCommand(&packetCfg) !=
          SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
        return false;
      packetCfg.len = 4; // 4 byte header
      packetCfg.startingSpot = 0;
      _numCfgKeys = 0;
      memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
    }

    if (packetCfg.len >= (packetCfgPayloadSize - (4 + N))) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("addCfgValsetN: packetCfgPayloadSize reached!"));
#endif
      return false;
    }

    if (_numCfgKeys == CFG_VALSET_MAX_KEYS) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
        _debugSerial.println(F("addCfgValsetN: key limit reached!"));
#endif
      return false;
    }

    // Load key into outgoing payload
    key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
    for (uint8_t i = 0; i < 4; i++)
      payloadCfg[packetCfg.len + i] = key >> (8 * i); // Key

    // Load user's value
    for (uint8_t i = 0; i < N; i++)
      payloadCfg[packetCfg.len + i + 4] = *value++; // Value

    // Update packet length: 4 byte key ID, 8 bytes of value
    packetCfg.len = packetCfg.len + 4 + N;

    _numCfgKeys++;

    // All done
    return (true);
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 64-bit value
  bool DevUBLOXGNSS::addCfgValset64(uint32_t key, uint64_t value) {
    uint8_t val[8];

    // Load user's value
    for (uint8_t i = 0; i < 8; i++)
      val[i] = (uint8_t)(value >> (8 * i)); // Value

    return (addCfgValsetN(key, val, 8));
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 32-bit value
  bool DevUBLOXGNSS::addCfgValset32(uint32_t key, uint32_t value) {
    uint8_t val[4] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8),
                      (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    return (addCfgValsetN(key, val, 4));
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 16-bit value
  bool DevUBLOXGNSS::addCfgValset16(uint32_t key, uint16_t value) {
    uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
    return (addCfgValsetN(key, val, 2));
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 8-bit value
  bool DevUBLOXGNSS::addCfgValset8(uint32_t key, uint8_t value) {
    uint8_t val[1] = {value};
    return (addCfgValsetN(key, val, 1));
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 32-bit float (R4) value
  bool DevUBLOXGNSS::addCfgValsetFloat(uint32_t key, float value) {
    if (sizeof(float) != 4) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("addCfgValsetFloat not supported!"));
      }
#endif
      return false;
    }

    // Define a union to convert from float to uint32_t
    unsigned32float convert32;

    convert32.flt = value;

    return (addCfgValset32(key, convert32.unsigned32));
  }

  // Add another key and value to an existing UBX-CFG-VALSET ubxPacket
  // This function takes a full 32-bit key and 64-bit double (R8) value
  // This won't work on older AVR platforms where double is 32-bit
  bool DevUBLOXGNSS::addCfgValsetDouble(uint32_t key, double value) {
    if (sizeof(double) != 8) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("addCfgValsetDouble not supported!"));
      }
#endif
      return false;
    }

    // Define a union to convert from double to uint64_t
    unsigned64double convert64;

    convert64.dbl = value;

    return (addCfgValset64(key, convert64.unsigned64));
  }

  // Send the UBX-CFG-VALSET ubxPacket
  bool DevUBLOXGNSS::sendCfgValset(uint16_t maxWait) {
    if (_numCfgKeys == 0)
      return true; // Nothing to send...

    // Send VALSET command with this key and value
    bool success = sendCommand(&packetCfg, maxWait) ==
                   SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK

    if (success)
      _numCfgKeys = 0;

    return success;
  }

  // Return the number of keys in the CfgValset
  uint8_t DevUBLOXGNSS::getCfgValsetLen() { return _numCfgKeys; }

  // Return the number of free bytes remaining in packetCfgPayload
  size_t DevUBLOXGNSS::getCfgValsetSpaceRemaining() {
    return getPacketCfgSpaceRemaining();
  }

  // Deprecated - only included for backward-compatibility. Use newCfgValset and
  // sendCfgValset
  bool DevUBLOXGNSS::newCfgValset8(uint32_t key, uint8_t value, uint8_t layer) {
    bool result = newCfgValset(layer);
    result &= addCfgValset8(key, value);
    return result;
  }
  bool DevUBLOXGNSS::newCfgValset16(uint32_t key, uint16_t value,
                                    uint8_t layer) {
    bool result = newCfgValset(layer);
    result &= addCfgValset16(key, value);
    return result;
  }
  bool DevUBLOXGNSS::newCfgValset32(uint32_t key, uint32_t value,
                                    uint8_t layer) {
    bool result = newCfgValset(layer);
    result &= addCfgValset32(key, value);
    return result;
  }
  bool DevUBLOXGNSS::newCfgValset64(uint32_t key, uint64_t value,
                                    uint8_t layer) {
    bool result = newCfgValset(layer);
    result &= addCfgValset64(key, value);
    return result;
  }
  bool DevUBLOXGNSS::sendCfgValset8(uint32_t key, uint8_t value,
                                    uint16_t maxWait) {
    bool result = addCfgValset8(key, value);
    result &= sendCfgValset(maxWait);
    return result;
  }
  bool DevUBLOXGNSS::sendCfgValset16(uint32_t key, uint16_t value,
                                     uint16_t maxWait) {
    bool result = addCfgValset16(key, value);
    result &= sendCfgValset(maxWait);
    return result;
  }
  bool DevUBLOXGNSS::sendCfgValset32(uint32_t key, uint32_t value,
                                     uint16_t maxWait) {
    bool result = addCfgValset32(key, value);
    result &= sendCfgValset(maxWait);
    return result;
  }
  bool DevUBLOXGNSS::sendCfgValset64(uint32_t key, uint64_t value,
                                     uint16_t maxWait) {
    bool result = addCfgValset64(key, value);
    result &= sendCfgValset(maxWait);
    return result;
  }

  bool DevUBLOXGNSS::newCfgValget(
      uint8_t layer) // Create a new, empty UBX-CFG-VALGET. Add entries with
                     // addCfgValget8/16/32/64
  {
    return (newCfgValget(&packetCfg, packetCfgPayloadSize, layer));
  }

  bool DevUBLOXGNSS::newCfgValget(
      ubxPacket * pkt, uint16_t maxPayload,
      uint8_t layer) // Create a new, empty UBX-CFG-VALGET. Add entries with
                     // addCfgValget8/16/32/64
  {
    if (cfgValgetValueSizes ==
        nullptr) // Check if RAM has been allocated for cfgValgetValueSizes
    {
      cfgValgetValueSizes = new uint8_t[CFG_VALSET_MAX_KEYS];
    }

    _cfgValgetMaxPayload = maxPayload;

    pkt->cls = UBX_CLASS_CFG;
    pkt->id = UBX_CFG_VALGET;
    pkt->len = 4; // 4 byte header
    pkt->startingSpot = 0;

    _numGetCfgKeys = 0;
    _lenCfgValGetResponse = 0;

    // Clear all of packet payload
    if (pkt == &packetCfg) {
      memset(payloadCfg, 0, packetCfgPayloadSize);
    } else {
      // Custom packet: we don't know how large payload is, so only clear the
      // two skip keys bytes
      pkt->payload[2] = 0; // Set the skip keys bytes to zero
      pkt->payload[3] = 0;
    }

    // VALGET uses different memory layer definitions to VALSET
    // because it can only return the value for one layer.
    // So we need to fiddle the layer here.
    // And just to complicate things further, the ZED-F9P only responds
    // correctly to layer 0 (RAM) and layer 7 (Default)!
    uint8_t getLayer = VAL_LAYER_DEFAULT; // 7 is the "Default Layer"
    if (layer == VAL_LAYER_RAM)           // Did the user request the RAM layer?
    {
      getLayer = 0;                    // Layer 0 is RAM
    } else if (layer == VAL_LAYER_BBR) // Did the user request the BBR layer?
    {
      getLayer = 1; // Layer 1 is BBR
    } else if (layer ==
               VAL_LAYER_FLASH) // Did the user request the Flash layer?
    {
      getLayer = 2; // Layer 2 is Flash
    }

    pkt->payload[0] = 0;        // Message Version - set to 0
    pkt->payload[1] = getLayer; // Layer

    if (maxPayload <
        9) // Sanity check - make sure there's room for a single L/U1 response
      return false;

    // All done
    return (true);
  }

  bool DevUBLOXGNSS::addCfgValget(
      uint32_t key) // Add a new key to an existing UBX-CFG-VALGET ubxPacket
  {
    return (addCfgValget(&packetCfg, key));
  }

  bool DevUBLOXGNSS::addCfgValget(
      ubxPacket * pkt,
      uint32_t key) // Add a new key to an existing UBX-CFG-VALGET ubxPacket
  {
    // Extract the value size
    uint8_t valueSizeBytes = getCfgValueSizeBytes(key);

    if (_lenCfgValGetResponse >=
        (_cfgValgetMaxPayload - (4 + (valueSizeBytes)))) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("addCfgValget: packetCfgPayloadSize reached!"));
      }
#endif
      return false;
    }

    if (_numGetCfgKeys == CFG_VALSET_MAX_KEYS) {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) ||
          (_printLimitedDebug ==
           true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial.println(F("addCfgValget: key limit reached!"));
      }
#endif
      return false;
    }

    // Store the value size in cfgValgetValueSizes
    if (cfgValgetValueSizes != nullptr) {
      cfgValgetValueSizes[_numGetCfgKeys] = valueSizeBytes;
    }

    // Load key into outgoing payload
    uint8_t *ptr;
    ptr = pkt->payload;
    ptr += pkt->len;
    key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
    for (uint8_t i = 0; i < 4; i++) {
      *ptr = key >> (8 * i); // Key
      ptr++;
    }

    // Update packet length: 4 byte key ID
    pkt->len += 4;

    _numGetCfgKeys++;
    _lenCfgValGetResponse +=
        4 + (valueSizeBytes); // 4 byte key ID, N byte value

    // All done
    return (true);
  }

  bool DevUBLOXGNSS::sendCfgValget(
      uint16_t maxWait) // Send the CfgValget (UBX-CFG-VALGET) construct
  {
    return (sendCfgValget(&packetCfg, maxWait));
  }

  bool DevUBLOXGNSS::sendCfgValget(
      ubxPacket * pkt,
      uint16_t maxWait) // Send the CfgValget (UBX-CFG-VALGET) construct
  {
    if (_numGetCfgKeys == 0)
      return true; // Nothing to send...

    // Send VALSET command with this key and value
    bool success =
        sendCommand(pkt, maxWait) ==
        SFE_UBLOX_STATUS_DATA_RECEIVED; // We are expecting data and an ACK

    if (success)
      _numGetCfgKeys = 0;

    return success;
  }

  uint8_t DevUBLOXGNSS::getCfgValueSizeBytes(const uint32_t key) {
    switch (key & UBX_CFG_SIZE_MASK) {
    case UBX_CFG_L:
    case UBX_CFG_U1:
    case UBX_CFG_I1:
    case UBX_CFG_E1:
    case UBX_CFG_X1:
      return 1;
      break;
    case UBX_CFG_U2:
    case UBX_CFG_I2:
    case UBX_CFG_E2:
    case UBX_CFG_X2:
      return 2;
      break;
    case UBX_CFG_U4:
    case UBX_CFG_I4:
    case UBX_CFG_E4:
    case UBX_CFG_X4:
    case UBX_CFG_R4:
      return 4;
      break;
    case UBX_CFG_U8:
    case UBX_CFG_I8:
    case UBX_CFG_X8:
    case UBX_CFG_R8:
      return 8;
      break;
    default:
      return 0; // Error
      break;
    }
    return 0;
  }

  // ***** no PVT automatic support

  // Get the latest Position/Velocity/Time solution and fill all global
  // variables
  bool DevUBLOXGNSS::getPVT(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    // The GPS is not automatically reporting navigation position so we have
    // to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;
    // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can
    // record up to 20+packetCfgPayloadSize = 84 bytes Note:now hard-coded in
    // processUBX

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
      return (true);

    return (false);
  }

  // PRIVATE: Allocate RAM for packetUBXNAVPVT and initialize it
  bool DevUBLOXGNSS::initPacketUBXNAVPVT() {
    packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
    if (packetUBXNAVPVT == nullptr) {
      return (false);
    }
    packetUBXNAVPVT->automaticFlags.flags.all = 0;
    packetUBXNAVPVT->callbackPointerPtr = nullptr;
    packetUBXNAVPVT->callbackData = nullptr;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
    packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
    return (true);
  }

  // Mark all the PVT data as read/stale. This is handy to get data alignment
  // after CRC failure
  void DevUBLOXGNSS::flushPVT() {
    if (packetUBXNAVPVT == nullptr)
      return; // Bail if RAM has not been allocated (otherwise we could be
              // writing anywhere!)
    packetUBXNAVPVT->moduleQueried.moduleQueried1.all =
        0; // Mark all datums as stale (read before)
    packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
  }

  // ***** CFG RATE Helper Functions

  // Set the rate at which the module will give us an updated navigation
  // solution Expects a number that is the updates per second. For example 1 =
  // 1Hz, 2 = 2Hz, etc. Max is 40Hz(?!)
  bool DevUBLOXGNSS::setNavigationFrequency(uint8_t navFreq, uint8_t layer,
                                            uint16_t maxWait) {
    if (navFreq == 0) // Return now if navFreq is zero
      return (false);

    if (navFreq > 40)
      navFreq = 40; // Limit navFreq to 40Hz so i2cPollingWait is set correctly

    // Adjust the I2C polling timeout based on update rate
    // Do this even if the sendCommand fails
    i2cPollingWaitNAV =
        1000 /
        (((int)navFreq) * 4); // This is the number of ms to wait between checks
                              // for new I2C data. Max is 250. Min is 6.
    i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR
                         ? i2cPollingWaitNAV
                         : i2cPollingWaitHNR; // Set i2cPollingWait to the lower
                                              // of NAV and HNR

    uint16_t measurementRate = 1000 / navFreq;

    return setVal16(UBLOX_CFG_RATE_MEAS, measurementRate, layer, maxWait);
  }

  // Get the rate at which the module is outputting nav solutions
  // Note: if the measurementRate (which is actually a period) is less than
  // 1000ms, this will return zero
  bool DevUBLOXGNSS::getNavigationFrequency(uint8_t *navFreq, uint8_t layer,
                                            uint16_t maxWait) {
    uint16_t measurementRate = 0;

    bool result =
        getVal16(UBLOX_CFG_RATE_MEAS, &measurementRate, layer, maxWait);

    if ((result) && (measurementRate > 0))
      *navFreq =
          1000 / measurementRate; // This may return an int when it's a float,
                                  // but I'd rather not return 4 bytes

    return result;
  }
  uint8_t DevUBLOXGNSS::getNavigationFrequency(
      uint8_t layer,
      uint16_t maxWait) // Unsafe overload...
  {
    uint8_t navFreq = 0;

    getNavigationFrequency(&navFreq, layer, maxWait);

    return navFreq;
  }

  // Set the elapsed time between GNSS measurements in milliseconds, which
  // defines the rate
  bool DevUBLOXGNSS::setMeasurementRate(uint16_t rate, uint8_t layer,
                                        uint16_t maxWait) {
    if (rate <
        25) // "Measurement rate should be greater than or equal to 25 ms."
      rate = 25;

    // Adjust the I2C polling timeout based on update rate
    if (rate >= 1000)
      i2cPollingWaitNAV = 250;
    else
      i2cPollingWaitNAV =
          rate /
          4; // This is the number of ms to wait between checks for new I2C data
    i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR
                         ? i2cPollingWaitNAV
                         : i2cPollingWaitHNR; // Set i2cPollingWait to the lower
                                              // of NAV and HNR

    return setVal16(UBLOX_CFG_RATE_MEAS, rate, layer, maxWait);
  }

  // Return the elapsed time between GNSS measurements in milliseconds, which
  // defines the rate
  bool DevUBLOXGNSS::getMeasurementRate(uint16_t *measRate, uint8_t layer,
                                        uint16_t maxWait) {
    uint16_t measurementRate;

    bool result =
        getVal16(UBLOX_CFG_RATE_MEAS, &measurementRate, layer, maxWait);

    if (result)
      *measRate = measurementRate;

    return result;
  }
  uint16_t DevUBLOXGNSS::getMeasurementRate(
      uint8_t layer,
      uint16_t maxWait) // Unsafe overload...
  {
    uint16_t measurementRate;

    getMeasurementRate(&measurementRate, layer, maxWait);

    return measurementRate;
  }

  // Set the ratio between the number of measurements and the number of
  // navigation solutions. Unit is cycles. Max is 127.
  bool DevUBLOXGNSS::setNavigationRate(uint16_t rate, uint8_t layer,
                                       uint16_t maxWait) {
    return setVal16(UBLOX_CFG_RATE_NAV, rate, layer, maxWait);
  }

  // Return the ratio between the number of measurements and the number of
  // navigation solutions. Unit is cycles
  bool DevUBLOXGNSS::getNavigationRate(uint16_t *navRate, uint8_t layer,
                                       uint16_t maxWait) {
    uint16_t navigationRate;

    bool result = getVal16(UBLOX_CFG_RATE_NAV, &navigationRate, layer, maxWait);

    if (result)
      *navRate = navigationRate;

    return result;
  }
  uint16_t DevUBLOXGNSS::getNavigationRate(
      uint8_t layer,
      uint16_t maxWait) // Unsafe overload...
  {
    uint16_t navigationRate;

    getNavigationRate(&navigationRate, layer, maxWait);

    return navigationRate;
  }

  // ***** DOP Helper Functions

  uint32_t DevUBLOXGNSS::getTimeOfWeek(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.iTOW);
  }

  // Get the current year
  uint16_t DevUBLOXGNSS::getYear(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.year);
  }

  // Get the current month
  uint8_t DevUBLOXGNSS::getMonth(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.month);
  }

  // Get the current day
  uint8_t DevUBLOXGNSS::getDay(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.day);
  }

  // Get the current hour
  uint8_t DevUBLOXGNSS::getHour(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.hour);
  }

  // Get the current minute
  uint8_t DevUBLOXGNSS::getMinute(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.min);
  }

  // Get the current second
  uint8_t DevUBLOXGNSS::getSecond(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.sec);
  }

  // Get the current millisecond
  uint16_t DevUBLOXGNSS::getMillisecond(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.iTOW % 1000);
  }

  // Get the current nanoseconds - includes milliseconds
  int32_t DevUBLOXGNSS::getNanosecond(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.nano);
  }

  // Get the current Unix epoch time rounded to the nearest second
  uint32_t DevUBLOXGNSS::getUnixEpoch(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020; // Jan 1st 2020 as days from
                                                   // Jan 1st 1970
    t += (uint32_t)
        SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year -
                                  2020]; // Add on the number of days since 2020
    t += (uint32_t)
        SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1]
                                  [packetUBXNAVPVT->data.month -
                                   1]; // Add on the number of days since Jan
                                       // 1st
    t += (uint32_t)packetUBXNAVPVT->data.day -
         1;  // Add on the number of days since the 1st of the month
    t *= 24; // Convert to hours
    t += (uint32_t)packetUBXNAVPVT->data.hour; // Add on the hour
    t *= 60;                                   // Convert to minutes
    t += (uint32_t)packetUBXNAVPVT->data.min;  // Add on the minute
    t *= 60;                                   // Convert to seconds
    t += (uint32_t)packetUBXNAVPVT->data.sec;  // Add on the second
    return t;
  }

  // Get the current Unix epoch including microseconds
  uint32_t DevUBLOXGNSS::getUnixEpoch(uint32_t &microsecond, uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano = false;
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020; // Jan 1st 2020 as days from
                                                   // Jan 1st 1970
    t += (uint32_t)
        SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year -
                                  2020]; // Add on the number of days since 2020
    t += (uint32_t)
        SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1]
                                  [packetUBXNAVPVT->data.month -
                                   1]; // Add on the number of days since Jan
                                       // 1st
    t += (uint32_t)packetUBXNAVPVT->data.day -
         1;  // Add on the number of days since the 1st of the month
    t *= 24; // Convert to hours
    t += (uint32_t)packetUBXNAVPVT->data.hour;      // Add on the hour
    t *= 60;                                        // Convert to minutes
    t += (uint32_t)packetUBXNAVPVT->data.min;       // Add on the minute
    t *= 60;                                        // Convert to seconds
    t += (uint32_t)packetUBXNAVPVT->data.sec;       // Add on the second
    int32_t us = packetUBXNAVPVT->data.nano / 1000; // Convert nanos to micros
    microsecond = (uint32_t)us;                     // Could be -ve!
    // Adjust t if nano is negative
    if (us < 0) {
      microsecond = (uint32_t)(us + 1000000); // Make nano +ve
      t--;                                    // Decrement t by 1 second
    }
    return t;
  }

  // Get the current date validity
  bool DevUBLOXGNSS::getDateValid(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.valid.bits.validDate);
  }

  // Get the current time validity
  bool DevUBLOXGNSS::getTimeValid(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.valid.bits.validTime);
  }

  // Check to see if the UTC time has been fully resolved
  bool DevUBLOXGNSS::getTimeFullyResolved(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved ==
        false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.valid.bits.fullyResolved);
  }

  // Get the confirmed date validity
  bool DevUBLOXGNSS::getConfirmedDate(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate ==
        false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedDate);
  }

  // Get the confirmed time validity
  bool DevUBLOXGNSS::getConfirmedTime(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime ==
        false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedTime);
  }

  // Get the current fix type
  // 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
  uint8_t DevUBLOXGNSS::getFixType(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.fixType);
  }

  // Get whether we have a valid fix (i.e within DOP & accuracy masks)
  bool DevUBLOXGNSS::getGnssFixOk(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.flags.bits.gnssFixOK);
  }

  // Get whether differential corrections were applied
  bool DevUBLOXGNSS::getDiffSoln(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.flags.bits.diffSoln);
  }

  // Get whether head vehicle valid or not
  bool DevUBLOXGNSS::getHeadVehValid(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid ==
        false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.flags.bits.headVehValid);
  }

  // Get the carrier phase range solution status
  // Useful when querying module to see if it has high-precision RTK fix
  // 0=No solution, 1=Float solution, 2=Fixed solution
  uint8_t DevUBLOXGNSS::getCarrierSolutionType(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.flags.bits.carrSoln);
  }

  // Get the number of satellites used in fix
  uint8_t DevUBLOXGNSS::getSIV(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.numSV);
  }

  // Get the current longitude in degrees
  // Returns a long representing the number of degrees *10^-7
  int32_t DevUBLOXGNSS::getLongitude(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.lon);
  }

  // Get the current latitude in degrees
  // Returns a long representing the number of degrees *10^-7
  int32_t DevUBLOXGNSS::getLatitude(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.lat);
  }

  // Get the current altitude in mm according to ellipsoid model
  int32_t DevUBLOXGNSS::getAltitude(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.height);
  }

  // Get the current altitude in mm according to mean sea level
  // Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
  // Difference between Ellipsoid Model and Mean Sea Level:
  // https://eos-gnss.com/elevation-for-beginners/ Also see:
  // https://portal.u-blox.com/s/question/0D52p00008HKDSkCAP/what-geoid-model-is-used-and-where-is-this-calculated
  // and: https://cddis.nasa.gov/926/egm96/egm96.html on 10x10 degree grid
  int32_t DevUBLOXGNSS::getAltitudeMSL(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.hMSL);
  }

  int32_t DevUBLOXGNSS::getHorizontalAccEst(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.hAcc);
  }

  int32_t DevUBLOXGNSS::getVerticalAccEst(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.vAcc);
  }

  int32_t DevUBLOXGNSS::getNedNorthVel(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.velN);
  }

  int32_t DevUBLOXGNSS::getNedEastVel(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.velE);
  }

  int32_t DevUBLOXGNSS::getNedDownVel(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.velD);
  }

  // Get the ground speed in mm/s
  int32_t DevUBLOXGNSS::getGroundSpeed(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.gSpeed);
  }

  // Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
  int32_t DevUBLOXGNSS::getHeading(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.headMot);
  }

  uint32_t DevUBLOXGNSS::getSpeedAccEst(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.sAcc);
  }

  uint32_t DevUBLOXGNSS::getHeadingAccEst(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.headAcc);
  }

  // Get the positional dillution of precision * 10^-2 (dimensionless)
  uint16_t DevUBLOXGNSS::getPDOP(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.pDOP);
  }

  bool DevUBLOXGNSS::getInvalidLlh(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return (false);

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return ((bool)packetUBXNAVPVT->data.flags3.bits.invalidLlh);
  }

  int32_t DevUBLOXGNSS::getHeadVeh(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.headVeh);
  }

  int16_t DevUBLOXGNSS::getMagDec(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.magDec);
  }

  uint16_t DevUBLOXGNSS::getMagAcc(uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc == false)
      getPVT(maxWait);
    packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVT->data.magAcc);
  }

  // getGeoidSeparation is currently redundant. The geoid separation seems to
  // only be provided in NMEA GGA and GNS messages.
  int32_t DevUBLOXGNSS::getGeoidSeparation(uint16_t maxWait) {
    (void)maxWait; // Do something with maxWait just to get rid of the pesky
                   // compiler warning

    return (0);
  }

  // ***** HPPOSECEF Helper Functions

  // Get the current 3D high precision positional accuracy - a fun thing to
  // watch Returns a long representing the 3D accuracy in millimeters
  uint32_t DevUBLOXGNSS::getPositionAccuracy(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc == false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    uint32_t tempAccuracy = packetUBXNAVHPPOSECEF->data.pAcc;

    if ((tempAccuracy % 10) >= 5)
      tempAccuracy += 5; // Round fraction of mm up to next mm if .5 or above
    tempAccuracy /= 10;  // Convert 0.1mm units to mm

    return (tempAccuracy);
  }

  // Get the current 3D high precision X coordinate
  // Returns a long representing the coordinate in cm
  int32_t DevUBLOXGNSS::getHighResECEFX(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX == false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefX);
  }

  // Get the current 3D high precision Y coordinate
  // Returns a long representing the coordinate in cm
  int32_t DevUBLOXGNSS::getHighResECEFY(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY == false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefY);
  }

  // Get the current 3D high precision Z coordinate
  // Returns a long representing the coordinate in cm
  int32_t DevUBLOXGNSS::getHighResECEFZ(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ == false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefZ);
  }

  // Get the high precision component of the ECEF X coordinate
  // Returns a signed byte representing the component as 0.1*mm
  int8_t DevUBLOXGNSS::getHighResECEFXHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp ==
        false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefXHp);
  }

  // Get the high precision component of the ECEF Y coordinate
  // Returns a signed byte representing the component as 0.1*mm
  int8_t DevUBLOXGNSS::getHighResECEFYHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp ==
        false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefYHp);
  }

  // Get the high precision component of the ECEF Z coordinate
  // Returns a signed byte representing the component as 0.1*mm
  int8_t DevUBLOXGNSS::getHighResECEFZHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSECEF == nullptr)
      initPacketUBXNAVHPPOSECEF(); // Check that RAM has been allocated for the
                                   // HPPOSECEF data
    if (packetUBXNAVHPPOSECEF == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp ==
        false)
      getNAVHPPOSECEF(maxWait);
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVHPPOSECEF->data.ecefZHp);
  }

  // ***** HPPOSLLH Helper Functions

  uint32_t DevUBLOXGNSS::getTimeOfWeekFromHPPOSLLH(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.iTOW);
  }

  int32_t DevUBLOXGNSS::getHighResLongitude(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.lon);
  }

  int32_t DevUBLOXGNSS::getHighResLatitude(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.lat);
  }

  int32_t DevUBLOXGNSS::getElipsoid(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.height);
  }

  int32_t DevUBLOXGNSS::getMeanSeaLevel(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.hMSL);
  }

  int8_t DevUBLOXGNSS::getHighResLongitudeHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.lonHp);
  }

  int8_t DevUBLOXGNSS::getHighResLatitudeHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.latHp);
  }

  int8_t DevUBLOXGNSS::getElipsoidHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp ==
        false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.heightHp);
  }

  int8_t DevUBLOXGNSS::getMeanSeaLevelHp(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.hMSLHp);
  }

  uint32_t DevUBLOXGNSS::getHorizontalAccuracy(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.hAcc);
  }

  uint32_t DevUBLOXGNSS::getVerticalAccuracy(uint16_t maxWait) {
    if (packetUBXNAVHPPOSLLH == nullptr)
      initPacketUBXNAVHPPOSLLH(); // Check that RAM has been allocated for the
                                  // HPPOSLLH data
    if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc == false)
      getHPPOSLLH(maxWait);
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVHPPOSLLH->data.vAcc);
  }

  // ***** PVAT Helper Functions

  int32_t DevUBLOXGNSS::getVehicleRoll(uint16_t maxWait) {
    if (packetUBXNAVPVAT == nullptr)
      initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT
                              // data
    if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll == false)
      getNAVPVAT(maxWait);
    packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVAT->data.vehRoll);
  }

  int32_t DevUBLOXGNSS::getVehiclePitch(uint16_t maxWait) {
    if (packetUBXNAVPVAT == nullptr)
      initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT
                              // data
    if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch == false)
      getNAVPVAT(maxWait);
    packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVAT->data.vehPitch);
  }

  int32_t DevUBLOXGNSS::getVehicleHeading(uint16_t maxWait) {
    if (packetUBXNAVPVAT == nullptr)
      initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT
                              // data
    if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading == false)
      getNAVPVAT(maxWait);
    packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVAT->data.vehHeading);
  }

  int32_t DevUBLOXGNSS::getMotionHeading(uint16_t maxWait) {
    if (packetUBXNAVPVAT == nullptr)
      initPacketUBXNAVPVAT(); // Check that RAM has been allocated for the PVAT
                              // data
    if (packetUBXNAVPVAT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading == false)
      getNAVPVAT(maxWait);
    packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
    return (packetUBXNAVPVAT->data.motHeading);
  }

  // ***** SVIN Helper Functions

  bool DevUBLOXGNSS::getSurveyInActive(uint16_t maxWait) {
    if (packetUBXNAVSVIN == nullptr)
      initPacketUBXNAVSVIN(); // Check that RAM has been allocated for the SVIN
                              // data
    if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
      return false;

    if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active == false)
      getSurveyStatus(maxWait);
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
    return ((bool)packetUBXNAVSVIN->data.active);
  }

  bool DevUBLOXGNSS::getSurveyInValid(uint16_t maxWait) {
    if (packetUBXNAVSVIN == nullptr)
      initPacketUBXNAVSVIN(); // Check that RAM has been allocated for the SVIN
                              // data
    if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
      return false;

    if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid == false)
      getSurveyStatus(maxWait);
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
    return ((bool)packetUBXNAVSVIN->data.valid);
  }

  uint32_t DevUBLOXGNSS::getSurveyInObservationTimeFull(
      uint16_t maxWait) // Return the full uint32_t
  {
    if (packetUBXNAVSVIN == nullptr)
      initPacketUBXNAVSVIN(); // Check that RAM has been allocated for the SVIN
                              // data
    if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur == false)
      getSurveyStatus(maxWait);
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

    return (packetUBXNAVSVIN->data.dur);
  }

  uint16_t DevUBLOXGNSS::getSurveyInObservationTime(
      uint16_t maxWait) // Truncated to 65535 seconds
  {
    // dur (Passed survey-in observation time) is U4 (uint32_t) seconds. Here we
    // truncate to 16 bits
    uint32_t tmpObsTime = getSurveyInObservationTimeFull(maxWait);
    if (tmpObsTime <= 0xFFFF) {
      return ((uint16_t)tmpObsTime);
    } else {
      return (0xFFFF);
    }
  }

  float DevUBLOXGNSS::getSurveyInMeanAccuracy(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVSVIN == nullptr)
      initPacketUBXNAVSVIN(); // Check that RAM has been allocated for the SVIN
                              // data
    if (packetUBXNAVSVIN == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc == false)
      getSurveyStatus(maxWait);
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

    // meanAcc is U4 (uint32_t) in 0.1mm. We convert this to float.
    uint32_t tempFloat = packetUBXNAVSVIN->data.meanAcc;
    return (((float)tempFloat) / 10000.0); // Convert 0.1mm to m
  }

  // ***** TIMELS Helper Functions

  uint8_t DevUBLOXGNSS::getLeapIndicator(int32_t &timeToLsEvent,
                                         uint16_t maxWait) {
    if (packetUBXNAVTIMELS == nullptr)
      initPacketUBXNAVTIMELS(); // Check that RAM has been allocated for the
                                // TIMELS data
    if (packetUBXNAVTIMELS == nullptr) // Bail if the RAM allocation failed
      return 3;

    if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits
            .validTimeToLsEvent == false)
      getLeapSecondEvent(maxWait);
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validTimeToLsEvent =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.lsChange = false;
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.timeToLsEvent = false;
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
    timeToLsEvent = packetUBXNAVTIMELS->data.timeToLsEvent;
    // returns NTP Leap Indicator
    // 0 -no warning
    // 1 -last minute of the day has 61 seconds
    // 2 -last minute of the day has 59 seconds
    // 3 -unknown (clock unsynchronized)
    return ((bool)packetUBXNAVTIMELS->data.valid.bits.validTimeToLsEvent
                ? (uint8_t)(packetUBXNAVTIMELS->data.lsChange == -1
                                ? 2
                                : packetUBXNAVTIMELS->data.lsChange)
                : 3);
  }

  int8_t DevUBLOXGNSS::getCurrentLeapSeconds(sfe_ublox_ls_src_e & source,
                                             uint16_t maxWait) {
    if (packetUBXNAVTIMELS == nullptr)
      initPacketUBXNAVTIMELS(); // Check that RAM has been allocated for the
                                // TIMELS data
    if (packetUBXNAVTIMELS == nullptr) // Bail if the RAM allocation failed
      return false;

    if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs ==
        false)
      getLeapSecondEvent(maxWait);
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.srcOfCurrLs = false;
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.currLs = false;
    packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
    source = ((sfe_ublox_ls_src_e)packetUBXNAVTIMELS->data.srcOfCurrLs);
    return ((int8_t)packetUBXNAVTIMELS->data.currLs);
  }

  // ***** RELPOSNED Helper Functions and automatic support

  float DevUBLOXGNSS::getRelPosN(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN ==
        false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.relPosN) /
            100.0); // Convert to m
  }

  float DevUBLOXGNSS::getRelPosE(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE ==
        false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.relPosE) /
            100.0); // Convert to m
  }

  float DevUBLOXGNSS::getRelPosD(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD ==
        false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.relPosD) /
            100.0); // Convert to m
  }

  float DevUBLOXGNSS::getRelPosAccN(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN == false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.accN) /
            10000.0); // Convert to m
  }

  float DevUBLOXGNSS::getRelPosAccE(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE == false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.accE) /
            10000.0); // Convert to m
  }

  float DevUBLOXGNSS::getRelPosAccD(uint16_t maxWait) // Returned as m
  {
    if (packetUBXNAVRELPOSNED == nullptr)
      initPacketUBXNAVRELPOSNED(); // Check that RAM has been allocated for the
                                   // RELPOSNED data
    if (packetUBXNAVRELPOSNED == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD == false)
      getRELPOSNED(maxWait);
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXNAVRELPOSNED->data.accD) /
            10000.0); // Convert to m
  }

  // ***** AOPSTATUS Helper Functions

  uint8_t DevUBLOXGNSS::getAOPSTATUSuseAOP(uint16_t maxWait) {
    if (packetUBXNAVAOPSTATUS == nullptr)
      initPacketUBXNAVAOPSTATUS(); // Check that RAM has been allocated for the
                                   // AOPSTATUS data
    if (packetUBXNAVAOPSTATUS == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP == false)
      getAOPSTATUS(maxWait);
    packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVAOPSTATUS->data.aopCfg.bits.useAOP);
  }

  uint8_t DevUBLOXGNSS::getAOPSTATUSstatus(uint16_t maxWait) {
    if (packetUBXNAVAOPSTATUS == nullptr)
      initPacketUBXNAVAOPSTATUS(); // Check that RAM has been allocated for the
                                   // AOPSTATUS data
    if (packetUBXNAVAOPSTATUS == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status == false)
      getAOPSTATUS(maxWait);
    packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXNAVAOPSTATUS->data.status);
  }

  // ***** TIM TP Helper Functions

  uint32_t DevUBLOXGNSS::getTIMTPtowMS(uint16_t maxWait) {
    if (packetUBXTIMTP == nullptr)
      initPacketUBXTIMTP(); // Check that RAM has been allocated for the TP data
    if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS == false)
      getTIMTP(maxWait);
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXTIMTP->data.towMS);
  }

  uint32_t DevUBLOXGNSS::getTIMTPtowSubMS(uint16_t maxWait) {
    if (packetUBXTIMTP == nullptr)
      initPacketUBXTIMTP(); // Check that RAM has been allocated for the TP data
    if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS == false)
      getTIMTP(maxWait);
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXTIMTP->data.towSubMS);
  }

  uint16_t DevUBLOXGNSS::getTIMTPweek(uint16_t maxWait) {
    if (packetUBXTIMTP == nullptr)
      initPacketUBXTIMTP(); // Check that RAM has been allocated for the TP data
    if (packetUBXTIMTP == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.week == false)
      getTIMTP(maxWait);
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.week =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;
    return (packetUBXTIMTP->data.week);
  }

  // Convert TIM TP to Unix epoch including microseconds
  // CAUTION! Assumes the time base is UTC and the week number is GPS
  uint32_t DevUBLOXGNSS::getTIMTPAsEpoch(uint32_t &microsecond,
                                         uint16_t maxWait) {
    if (packetUBXNAVPVT == nullptr)
      initPacketUBXNAVPVT(); // Check that RAM has been allocated for the PVT
                             // data
    if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
      return 0;

    if (packetUBXTIMTP->moduleQueried.moduleQueried.bits.week == false)
      getTIMTP(maxWait);
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.week =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.towMS = false;
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.towSubMS = false;
    packetUBXTIMTP->moduleQueried.moduleQueried.bits.all = false;

    uint32_t tow = packetUBXTIMTP->data.week -
                   SFE_UBLOX_JAN_1ST_2020_WEEK; // Calculate the number of weeks
                                                // since Jan 1st 2020
    tow *= SFE_UBLOX_SECS_PER_WEEK;             // Convert weeks to seconds
    tow += SFE_UBLOX_EPOCH_WEEK_2086;           // Add the TOW for Jan 1st 2020
    tow += packetUBXTIMTP->data.towMS / 1000;   // Add the TOW for the next TP

    uint32_t us = packetUBXTIMTP->data.towMS % 1000; // Extract the milliseconds
    us *= 1000;                                      // Convert to microseconds

    double subMS = packetUBXTIMTP->data.towSubMS; // Get towSubMS (ms * 2^-32)
    subMS *=
        2.3283064365386963e-10; // pow(2.0, -32.0);  // Convert to milliseconds
    subMS *= 1000;              // Convert to microseconds

    us += (uint32_t)subMS; // Add subMS

    microsecond = us;
    return tow;
  }

  // ***** MON COMMS Helper Functions

  // Get the communication port information using UBX_MON_COMMS
  bool DevUBLOXGNSS::getCommsPortInfo(UBX_MON_COMMS_data_t * data,
                                      uint16_t maxWait) {
    if (data == nullptr) // Check if the user forgot to include the data pointer
      return (false);    // Bail

    if (!getMONCOMMS(maxWait))
      return (false);

    memcpy(data, &packetUBXMONCOMMS->data, sizeof(UBX_MON_COMMS_data_t));

    packetUBXMONCOMMS->moduleQueried = false; // Mark the data as stale

    return (true);
  }

  // ***** MON HW Helper Functions

  // Get the hardware status (including jamming) using UBX_MON_HW
  bool DevUBLOXGNSS::getHWstatus(UBX_MON_HW_data_t * data, uint16_t maxWait) {
    if (data == nullptr) // Check if the user forgot to include the data pointer
      return (false);    // Bail

    if (!getMONHW(maxWait))
      return (false);

    memcpy(data, &packetUBXMONHW->data, sizeof(UBX_MON_HW_data_t));

    packetUBXMONHW->moduleQueried.moduleQueried.all =
        0; // Mark all datums as stale

    return (true);
  }

  // Return the aStatus: 0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN
  sfe_ublox_antenna_status_e DevUBLOXGNSS::getAntennaStatus(uint16_t maxWait) {
    if (packetUBXMONHW == nullptr)
      initPacketUBXMONHW(); // Check that RAM has been allocated for the TP data
    if (packetUBXMONHW == nullptr) // Bail if the RAM allocation failed
      return SFE_UBLOX_ANTENNA_STATUS_INIT;

    if (packetUBXMONHW->moduleQueried.moduleQueried.bits.aStatus == false)
      getMONHW(maxWait);
    packetUBXMONHW->moduleQueried.moduleQueried.bits.aStatus =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXMONHW->moduleQueried.moduleQueried.bits.all = false;
    return ((sfe_ublox_antenna_status_e)packetUBXMONHW->data.aStatus);
  }

  // ***** Helper functions for the NEO-F10N
  bool DevUBLOXGNSS::getLNAMode(sfe_ublox_lna_mode_e * mode, uint8_t layer,
                                uint16_t maxWait) {
    return getVal8(UBLOX_CFG_HW_RF_LNA_MODE, (uint8_t *)mode, layer,
                   maxWait); // Get the LNA mode
  }
  bool DevUBLOXGNSS::setLNAMode(sfe_ublox_lna_mode_e mode, uint8_t layer,
                                uint16_t maxWait) {
    return setVal8(UBLOX_CFG_HW_RF_LNA_MODE, (uint8_t)mode, layer,
                   maxWait); // Set the LNA mode
  }
  bool DevUBLOXGNSS::getGPSL5HealthOverride(bool *override, uint8_t layer,
                                            uint16_t maxWait) {
    return getVal8(UBLOX_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE,
                   (uint8_t *) override, layer,
                   maxWait); // Get the GPS L5 health override status
  }
  bool DevUBLOXGNSS::setGPSL5HealthOverride(bool override, uint8_t layer,
                                            uint16_t maxWait) {
    return setVal8(UBLOX_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, (uint8_t) override,
                   layer, maxWait); // Set the GPS L5 health override status
  }

#ifndef SFE_UBLOX_DISABLE_ESF
  // ***** ESF Helper Functions

  float DevUBLOXGNSS::getESFroll(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXESFALG == nullptr)
      initPacketUBXESFALG(); // Check that RAM has been allocated for the ESF
                             // ALG data
    if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXESFALG->moduleQueried.moduleQueried.bits.roll == false)
      getESFALG(maxWait);
    packetUBXESFALG->moduleQueried.moduleQueried.bits.roll =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXESFALG->data.roll) / 100.0); // Convert to degrees
  }

  float DevUBLOXGNSS::getESFpitch(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXESFALG == nullptr)
      initPacketUBXESFALG(); // Check that RAM has been allocated for the ESF
                             // ALG data
    if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch == false)
      getESFALG(maxWait);
    packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXESFALG->data.pitch) / 100.0); // Convert to degrees
  }

  float DevUBLOXGNSS::getESFyaw(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXESFALG == nullptr)
      initPacketUBXESFALG(); // Check that RAM has been allocated for the ESF
                             // ALG data
    if (packetUBXESFALG == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw == false)
      getESFALG(maxWait);
    packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXESFALG->data.yaw) / 100.0); // Convert to degrees
  }

  bool DevUBLOXGNSS::getSensorFusionMeasurement(
      UBX_ESF_MEAS_sensorData_t * sensorData, UBX_ESF_MEAS_data_t ubxDataStruct,
      uint8_t sensor) {
    sensorData->data.all = ubxDataStruct.data[sensor].data.all;
    return (true);
  }

  bool DevUBLOXGNSS::getRawSensorMeasurement(
      UBX_ESF_RAW_sensorData_t * sensorData, UBX_ESF_RAW_data_t ubxDataStruct,
      uint8_t sensor) {
    sensorData->data.all = ubxDataStruct.data[sensor].data.all;
    sensorData->sTag = ubxDataStruct.data[sensor].sTag;
    return (true);
  }

  bool DevUBLOXGNSS::getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *
                                               sensorStatus,
                                           uint8_t sensor, uint16_t maxWait) {
    if (packetUBXESFSTATUS == nullptr)
      initPacketUBXESFSTATUS(); // Check that RAM has been allocated for the ESF
                                // STATUS data
    if (packetUBXESFSTATUS == nullptr) // Bail if the RAM allocation failed
      return (false);

    if ((packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status &
         (1 << sensor)) == 0)
      getESFSTATUS(maxWait);
    packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status &=
        ~(1 << sensor); // Since we are about to give this to user, mark this
                        // data as stale
    packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all = false;
    sensorStatus->sensStatus1.all =
        packetUBXESFSTATUS->data.status[sensor].sensStatus1.all;
    sensorStatus->sensStatus2.all =
        packetUBXESFSTATUS->data.status[sensor].sensStatus2.all;
    sensorStatus->freq = packetUBXESFSTATUS->data.status[sensor].freq;
    sensorStatus->faults.all =
        packetUBXESFSTATUS->data.status[sensor].faults.all;
    return (true);
  }

  bool DevUBLOXGNSS::getSensorFusionStatus(
      UBX_ESF_STATUS_sensorStatus_t * sensorStatus,
      UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor) {
    sensorStatus->sensStatus1.all =
        ubxDataStruct.status[sensor].sensStatus1.all;
    sensorStatus->sensStatus2.all =
        ubxDataStruct.status[sensor].sensStatus2.all;
    sensorStatus->freq = ubxDataStruct.status[sensor].freq;
    sensorStatus->faults.all = ubxDataStruct.status[sensor].faults.all;
    return (true);
  }
#endif

#ifndef SFE_UBLOX_DISABLE_HNR
  // ***** HNR Helper Functions

  // Set the High Navigation Rate
  // Returns true if the setHNRNavigationRate is successful
  bool DevUBLOXGNSS::setHNRNavigationRate(uint8_t rate, uint8_t layer,
                                          uint16_t maxWait) {
    if (rate == 0) // Return now if rate is zero
      return (false);

    if (rate > 40)
      rate = 40; // Limit rate to 40Hz so i2cPollingWait is set correctly

    // Placeholder for when HNR switches to the configuration interface
    (void)layer;

    // Adjust the I2C polling timeout based on update rate
    // Do this even if the sendCommand is not ACK'd
    i2cPollingWaitHNR =
        1000 / (((int)rate) * 4); // This is the number of ms to wait between
                                  // checks for new I2C data. Max 250. Min 6.
    i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR
                         ? i2cPollingWaitNAV
                         : i2cPollingWaitHNR; // Set i2cPollingWait to the lower
                                              // of NAV and HNR

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_HNR;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current HNR settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (false);

    // Load the new navigation rate into payloadCfg
    payloadCfg[0] = rate;

    // Update the navigation rate
    sfe_ublox_status_e result =
        sendCommand(&packetCfg, maxWait); // We are only expecting an ACK

    return (result == SFE_UBLOX_STATUS_DATA_SENT);
  }

  // Get the High Navigation Rate
  // Returns 0 if the getHNRNavigationRate fails
  uint8_t DevUBLOXGNSS::getHNRNavigationRate(uint8_t layer, uint16_t maxWait) {
    // Placeholder for when HNR switches to the configuration interface
    (void)layer;

    packetCfg.cls = UBX_CLASS_CFG;
    packetCfg.id = UBX_CFG_HNR;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // Ask module for the current HNR settings. Loads into payloadCfg.
    if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (0);

    // Return the navigation rate
    return (payloadCfg[0]);
  }

  float DevUBLOXGNSS::getHNRroll(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXHNRATT == nullptr)
      initPacketUBXHNRATT(); // Check that RAM has been allocated for the HNR
                             // ATT data
    if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll == false)
      getHNRATT(maxWait);
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXHNRATT->data.roll) /
            100000.0); // Convert to degrees
  }

  float DevUBLOXGNSS::getHNRpitch(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXHNRATT == nullptr)
      initPacketUBXHNRATT(); // Check that RAM has been allocated for the HNR
                             // ATT data
    if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch == false)
      getHNRATT(maxWait);
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXHNRATT->data.pitch) /
            100000.0); // Convert to degrees
  }

  float DevUBLOXGNSS::getHNRheading(uint16_t maxWait) // Returned as degrees
  {
    if (packetUBXHNRATT == nullptr)
      initPacketUBXHNRATT(); // Check that RAM has been allocated for the HNR
                             // ATT data
    if (packetUBXHNRATT == nullptr) // Bail if the RAM allocation failed
      return (0);

    if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading == false)
      getHNRATT(maxWait);
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading =
        false; // Since we are about to give this to user, mark this data as
               // stale
    packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
    return (((float)packetUBXHNRATT->data.heading) /
            100000.0); // Convert to degrees
  }
#endif

  // Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
  // From v2.0: These are public. The user can call these to extract data from
  // custom packets

  // Given a spot in the payload array, extract eight bytes and build a uint64_t
  uint64_t DevUBLOXGNSS::extractLongLong(ubxPacket * msg,
                                         uint16_t spotToStart) {
    uint64_t val = 0;
    for (uint8_t i = 0; i < 8; i++)
      val |= (uint64_t)msg->payload[spotToStart + i] << (8 * i);
    return (val);
  }

  // Given a spot in the payload array, extract eight bytes and build a int64_t
  int64_t DevUBLOXGNSS::extractSignedLongLong(ubxPacket * msg,
                                              uint16_t spotToStart) {
    unsignedSigned64 converter64;

    converter64.unsigned64 = extractLongLong(msg, spotToStart);
    return (converter64.signed64);
  }

  // Given a spot in the payload array, extract four bytes and build a long
  uint32_t DevUBLOXGNSS::extractLong(ubxPacket * msg, uint16_t spotToStart) {
    uint32_t val = 0;
    for (uint8_t i = 0; i < 4; i++)
      val |= (uint32_t)msg->payload[spotToStart + i] << (8 * i);
    return (val);
  }

  // Just so there is no ambiguity about whether a uint32_t will cast to a
  // int32_t correctly...
  int32_t DevUBLOXGNSS::extractSignedLong(ubxPacket * msg,
                                          uint16_t spotToStart) {
    unsignedSigned32 converter;
    converter.unsigned32 = extractLong(msg, spotToStart);
    return (converter.signed32);
  }

  // Given a spot in the payload array, extract two bytes and build an int
  uint16_t DevUBLOXGNSS::extractInt(ubxPacket * msg, uint16_t spotToStart) {
    uint16_t val = (uint16_t)msg->payload[spotToStart + 0] << 8 * 0;
    val |= (uint16_t)msg->payload[spotToStart + 1] << 8 * 1;
    return (val);
  }

  // Just so there is no ambiguity about whether a uint16_t will cast to a
  // int16_t correctly...
  int16_t DevUBLOXGNSS::extractSignedInt(ubxPacket * msg,
                                         uint16_t spotToStart) {
    unsignedSigned16 converter;
    converter.unsigned16 = extractInt(msg, spotToStart);
    return (converter.signed16);
  }

  // Given a spot, extract a byte from the payload
  uint8_t DevUBLOXGNSS::extractByte(ubxPacket * msg, uint16_t spotToStart) {
    return (msg->payload[spotToStart]);
  }

  // Given a spot, extract a signed 8-bit value from the payload
  int8_t DevUBLOXGNSS::extractSignedChar(ubxPacket * msg,
                                         uint16_t spotToStart) {
    unsignedSigned8 converter;
    converter.unsigned8 = extractByte(msg, spotToStart);
    return (converter.signed8);
  }

  // Given a spot, extract a signed 32-bit float from the payload
  float DevUBLOXGNSS::extractFloat(ubxPacket * msg, uint16_t spotToStart) {
    unsigned32float converter;
    converter.unsigned32 = extractLong(msg, spotToStart);
    return (converter.flt);
  }

  // Given a spot, extract a signed 64-bit double from the payload
  double DevUBLOXGNSS::extractDouble(ubxPacket * msg, uint16_t spotToStart) {
    unsigned64double converter;
    converter.unsigned64 = extractLongLong(msg, spotToStart);
    return (converter.dbl);
  }

  // Given a pointer, extract an unsigned integer with width bits, starting at
  // bit start
  uint64_t DevUBLOXGNSS::extractUnsignedBits(uint8_t *ptr, uint16_t start,
                                             uint16_t width) {
    uint64_t result = 0;
    uint16_t count = 0;
    uint8_t bitMask = 0x80;

    // Skip whole bytes (8 bits)
    ptr += start / 8;
    count += (start / 8) * 8;

    // Loop until we reach the start bit
    while (count < start) {
      bitMask >>= 1; // Shift the bit mask
      count++;       // Increment the count

      if (bitMask == 0) // Have we counted 8 bits?
      {
        ptr++;          // Point to the next byte
        bitMask = 0x80; // Reset the bit mask
      }
    }

    // We have reached the start bit and ptr is pointing at the correct byte
    // Now extract width bits, incrementing ptr and shifting bitMask as we go
    while (count < (start + width)) {
      if (*ptr & bitMask) // Is the bit set?
        result |= 1;      // Set the corresponding bit in result

      bitMask >>= 1; // Shift the bit mask
      count++;       // Increment the count

      if (bitMask == 0) // Have we counted 8 bits?
      {
        ptr++;          // Point to the next byte
        bitMask = 0x80; // Reset the bit mask
      }

      if (count < (start + width)) // Do we need to shift result?
        result <<= 1;              // Shift the result
    }

    return result;
  }

  // Given a pointer, extract an signed integer with width bits, starting at bit
  // start
  int64_t DevUBLOXGNSS::extractSignedBits(uint8_t *ptr, uint16_t start,
                                          uint16_t width) {

    unsignedSigned64 result;
    result.unsigned64 = 0;

    uint64_t twosComplement = 0xFFFFFFFFFFFFFFFF;

    bool isNegative;

    uint16_t count = 0;
    uint8_t bitMask = 0x80;

    // Skip whole bytes (8 bits)
    ptr += start / 8;
    count += (start / 8) * 8;

    // Loop until we reach the start bit
    while (count < start) {
      bitMask >>= 1; // Shift the bit mask
      count++;       // Increment the count

      if (bitMask == 0) // Have we counted 8 bits?
      {
        ptr++;          // Point to the next byte
        bitMask = 0x80; // Reset the bit mask
      }
    }

    isNegative =
        *ptr &
        bitMask; // Record the first bit - indicates in the number is negative

    // We have reached the start bit and ptr is pointing at the correct byte
    // Now extract width bits, incrementing ptr and shifting bitMask as we go
    while (count < (start + width)) {
      if (*ptr & bitMask)       // Is the bit set?
        result.unsigned64 |= 1; // Set the corresponding bit in result

      bitMask >>= 1;        // Shift the bit mask
      count++;              // Increment the count
      twosComplement <<= 1; // Shift the two's complement mask (clear LSB)

      if (bitMask == 0) // Have we counted 8 bits?
      {
        ptr++;          // Point to the next byte
        bitMask = 0x80; // Reset the bit mask
      }

      if (count < (start + width)) // Do we need to shift result?
        result.unsigned64 <<= 1;   // Shift the result
    }

    // Handle negative number
    if (isNegative)
      result.unsigned64 |= twosComplement; // OR in the two's complement mask

    return result.signed64;
  }
