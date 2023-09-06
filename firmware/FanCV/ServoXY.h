#pragma once
#include <Arduino.h>

#include "BrezPlanner.h"
#include "Servo32.h"

class ServoXY : public BrezPlanner<2, uint16_t> {
   public:
    ServoXY(Servo32* sx, Servo32* sy) : _sx(sx), _sy(sy) {}
    void updateCurrent() {
        setCurrent(_sx->readMicroseconds(), _sy->readMicroseconds());
    }
    void tick() {
        if (BrezPlanner<2, uint16_t>::tick()) {
            _sx->writeMicroseconds(getPos(0));
            _sy->writeMicroseconds(getPos(1));
        }
    }

   private:
    Servo32* _sx;
    Servo32* _sy;
};