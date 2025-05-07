#ifndef _LORA_MESSAGE_H_
#define _LORA_MESSAGE_H_

#include <stdint.h>
#include <cstdio>

#include "LoraEncoder.h"

class LoraMessage {
    public:
        LoraMessage();
        ~LoraMessage();
        LoraMessage& addUnixtime(uint32_t unixtime);
        LoraMessage& addLatLng(double latitude, double longitude);
        LoraMessage& addUint16(uint16_t i);
        LoraMessage& addUint32(uint32_t i);
        LoraMessage& addTemperature(float temperature);
        LoraMessage& addUint8(uint8_t i);
        LoraMessage& addHumidity(float humidity);
        LoraMessage& addBitmap(bool a, bool b, bool c, bool d, bool e, bool f, bool g, bool h);
        LoraMessage& addRawFloat(float value);
        uint8_t* getBytes();
        int getLength();
    private:
        LoraEncoder _reallocBuffer(int delta);
        uint8_t* _buffer;
        int _currentSize;
};

#endif
