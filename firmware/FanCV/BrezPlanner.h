// Multi-axis Bresenham planner by AlexGyver [beta]

#pragma once
#include <Arduino.h>

// указать количество осей и тип данных координат
template <uint8_t AXES, typename T = int16_t>
class BrezPlanner {
   public:
    // подключить функцию-обработчик шагов вида f(uint8_t idx)
    void attach(void (*handler)(uint8_t idx)) {
        cb = *handler;
    }

    // установить текущую позицию массивом
    void setCurrent(T* cur) {
        for (uint8_t i = 0; i < AXES; i++) {
            _pos[i] = cur[i];
        }
    }

    // установить текущую позицию списком
    void setCurrent(T cur, ...) {
        _pos[0] = cur;
        va_list valist;
        va_start(valist, cur);
        for (uint8_t i = 1; i < AXES; i++) {
            _pos[i] = (T)va_arg(valist, int);
        }
        va_end(valist);
    }

    // установить целевую позицию массивом и начать движение
    void setTarget(T* tar) {
        _dist = 0;
        for (uint8_t i = 0; i < AXES; i++) {
            _ds[i] = abs((int32_t)tar[i] - (int32_t)_pos[i]);
            _dir[i] = (_pos[i] < tar[i]) ? 1 : -1;
            if (_ds[i] > _dist) _dist = _ds[i];
        }
        for (uint8_t i = 0; i < AXES; i++) _nd[i] = _dist / 2;
        _step = 0;
        _state = 1;
        _ready = 0;
    }

    // установить целевую позицию списком и начать движение
    void setTarget(T tar, ...) {
        T arr[AXES] = {tar};
        va_list valist;
        va_start(valist, tar);
        for (uint8_t i = 1; i < AXES; i++) {
            arr[i] = (T)va_arg(valist, int);
        }
        va_end(valist);
        setTarget(arr);
    }

    // установить скорость (шагов в секунду)
    void setSpeed(uint16_t stepSec) {
        if (stepSec > 1000) setPeriodUs(1000000ul / stepSec);
        else setPeriodMs(1000 / stepSec);
    }

    // установить скорость (период в мс)
    void setPeriodMs(uint16_t prd) {
        _prd = prd;
        _usMode = false;
    }

    // установить скорость (период в мкс)
    void setPeriodUs(uint16_t prd) {
        _prd = prd;
        _usMode = true;
    }

    // установить масштаб - шагов за период (умолч. 1)
    void setScale(uint8_t scale) {
        _scale = scale;
    }

    // продолжить движение
    void resume() {
        _state = 1;
    }

    // остановить движение
    void stop() {
        _state = 0;
    }

    // статус (true - движется)
    bool state() {
        return _state;
    }

    // однократно вернёт true при достижении цели
    bool ready() {
        return _ready ? (_ready = 0, true) : false;
    }

    // тикер, вызывать в loop. Вернёт true при следующем шаге
    bool tick() {
        if (!_state) return false;
        uint16_t time = _usMode ? micros() : millis();
        if ((uint16_t)(time - _tmr) >= _prd) {
            _tmr = time;
            if (_scale > 1) {
                for (uint8_t i = 0; i < _scale; i++) {
                    if (tickManual()) break;
                }
            } else {
                tickManual();
            }
            return true;
        }
        return false;
    }

    // сделать один шаг вручную, вернёт true если это был последний шаг
    bool tickManual() {
        _step++;
        if (_step == _dist) {
            _state = 0;
            _ready = 1;
        }
        for (uint8_t i = 0; i < AXES; i++) {
            if (_nd[i] < _ds[i]) {
                _nd[i] += _dist - _ds[i];
                _pos[i] += _dir[i];
                if (cb) cb(i);
            } else {
                _nd[i] -= _ds[i];
            }
        }
        return _step == _dist;
    }

    // получить текущую координату по оси
    T getPos(uint8_t idx) {
        return _pos[idx];
    }

    // получить текущее направление по оси
    int8_t getDir(uint8_t idx) {
        return _dir[idx];
    }

    // получить оставшееся количество шагов
    T getLeft() {
        return _dist - _step;
    }

    // получить длину траектории в количестве шагов
    T getTotal() {
        return _dist;
    }

   private:
    void (*cb)(uint8_t idx) = nullptr;
    uint16_t _prd = 10, _tmr = 0;
    bool _state = 0, _ready = 0;
    uint8_t _scale = 1;
    bool _usMode = false;

    int8_t _dir[AXES];
    T _ds[AXES], _nd[AXES], _pos[AXES];
    T _dist = 0, _step = 0;
};