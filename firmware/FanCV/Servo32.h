// ESP32 Servo by AlexGyver

#pragma once
#include <Arduino.h>

#define SRV_MIN 500   // us
#define SRV_MAX 2500  // us
#define SRV_RES 14    // bit
#define SRV_FREQ 50   // Hz

class Servo32 {
   public:
    Servo32(int8_t channel = -1) {
        setChannel(channel);
    }

    void setChannel(int8_t channel) {
        detach();
        _ch = channel;
        if (_ch >= 0) ledcSetup(_ch, SRV_FREQ, SRV_RES);
    }

    void reverse(bool rev) {
        _rev = rev;
    }

    void attach() {
        if (_ch >= 0 && _pin >= 0) ledcAttachPin(_pin, _ch);
    }

    void attach(int8_t pin, uint16_t min = SRV_MIN, uint16_t max = SRV_MAX, uint16_t value = 0, uint16_t smooth = 0) {
        if (_ch < 0 || pin < 0) return;
        _pin = pin;
        _min = min;
        _max = max;
        _atch = true;
        ledcAttachPin(_pin, _ch);
        if (value) {
            if (smooth) {
                for (uint16_t i = 0; i < 10; i++) {
                    ledcDetachPin(_ch);
                    delay(smooth / 10 - 10);
                    ledcAttachPin(_pin, _ch);
                    write(value);
                    delay(10);
                }
            } else {
                write(value);
            }
        }
    }

    void detach() {
        if (_ch >= 0) ledcDetachPin(_ch);
        _atch = false;
    }

    void write(uint16_t value) {
        if (value < 200) writeMicroseconds(map(value, 0, 180, _min, _max));
        else writeMicroseconds(value);
    }

    void writeMicroseconds(uint16_t value) {
        if (!_atch) return;
        _val = value;
        if (_rev) value = _max - value + _min;
        //value = constrain(value, _min, _max);
        value = value * (1ul << SRV_RES) / (1000000ul / SRV_FREQ);
        ledcWrite(_ch, value);
    }

    uint16_t read() {
        return map(_val, _min, _max, 0, 180);
    }

    uint16_t readMicroseconds() {
        return _val;
    }

    bool attached() {
        return _atch;
    }

    uint16_t getMin() {
        return _min;
    }
    uint16_t getMax() {
        return _max;
    }

   private:
    bool _rev = 0;
    bool _atch = 0;
    int8_t _ch = -1;
    int8_t _pin = -1;
    uint16_t _min, _max;
    uint16_t _val = 0;
};