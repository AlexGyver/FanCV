#pragma once
#include <Arduino.h>

class PWM {
   public:
    PWM(uint8_t pin, uint8_t channel, uint16_t freq = 16384, uint8_t res = 8) : _channel(channel) {
        ledcSetup(_channel, freq, res);
        ledcAttachPin(pin, _channel);
    }

    void write(uint16_t value) {
        ledcWrite(_channel, value);
    }

   private:
    uint8_t _channel = 0;
};