#pragma once
#include <Arduino.h>

#include "FaceFinder.h"
#include "Servo32.h"
#include "ServoXY.h"
#include "config.h"
#include "data.h"
#include "pwm32.h"

#define GH_NO_MQTT
#include <GyverHub.h>
GyverHub hub("MyDevices", "FanCV", "");

Servo32 servoX(0);
Servo32 servoY(1);
ServoXY xy(&servoX, &servoY);
FaceFinder face;
PWM fan(FAN_PWM_PIN, 2);

uint16_t bufpos_x, bufpos_y;
bool manual = 0;
bool locked = 0;
uint8_t sys_state = 0;  // 0 idle, 1 search, 2 locked

void build() {
    bool upd = 0;

    hub.Title(F("Fan"));
    hub.BeginWidgets();
    hub.WidgetSize(100);
    if (hub.Slider(&data.fan_speed, GH_UINT8, F("speed"), 0, 255)) {
        fan.write(data.fan_speed);
        upd |= 1;
    }

    hub.Title(F("Servo"));

    hub.WidgetSize(50);
    if (hub.Slider(&data.s_speed, GH_UINT16, F("speed"), 40, 200)) {
        xy.setSpeed(data.s_speed);
        upd |= 1;
    }
    upd |= hub.Slider(&data.search_speed, GH_UINT16, F("search speed"), 100, 300);

    if (hub.Button(0, F("home"))) {
        xy.setTarget(data.px.home, data.py.home);
    }
    if (hub.Switch(&manual, F("manual"))) {
        if (manual) xy.setTarget(xy.getPos(0), xy.getPos(1));
        locked = 0;
    }

    uint16_t sx = xy.getPos(0);
    if (hub.Slider(&sx, GH_UINT16, F("pos x"), 400, 2500)) {
        if (manual) xy.setTarget(hub.action().valueInt(), xy.getPos(1));
    }
    uint16_t sy = xy.getPos(1);
    if (hub.Slider(&sy, GH_UINT16, F("pos y"), 400, 2500)) {
        if (manual) xy.setTarget(xy.getPos(0), hub.action().valueInt());
    }

    upd |= hub.Input(&data.sx.min, GH_UINT16, F("min x"));
    upd |= hub.Input(&data.sy.min, GH_UINT16, F("min y"));
    upd |= hub.Input(&data.sx.max, GH_UINT16, F("max x"));
    upd |= hub.Input(&data.sy.max, GH_UINT16, F("max y"));
    upd |= hub.Input(&data.sx.anlge, GH_UINT8, F("angle x"));
    upd |= hub.Input(&data.sy.anlge, GH_UINT8, F("angle y"));

    if (hub.Switch(&data.sx.reverse, F("reverse x"))) {
        // servoX.reverse(data.sx.reverse);
        upd |= 1;
    }
    if (hub.Switch(&data.sy.reverse, F("reverse y"))) {
        // servoY.reverse(data.sy.reverse);
        upd |= 1;
    }

    hub.Title(F("Planner"));
    upd |= hub.Slider(&data.px.offset, GH_INT16, F("offset x"), -100, 100);
    upd |= hub.Slider(&data.py.offset, GH_INT16, F("offset y"), -100, 100);

    upd |= hub.Input(&data.px.min, GH_UINT16, F("min x"));
    upd |= hub.Input(&data.py.min, GH_UINT16, F("min y"));
    upd |= hub.Input(&data.px.max, GH_UINT16, F("max x"));
    upd |= hub.Input(&data.py.max, GH_UINT16, F("max y"));
    upd |= hub.Input(&data.px.home, GH_UINT16, F("home x"));
    upd |= hub.Input(&data.py.home, GH_UINT16, F("home y"));

    hub.Title(F("System"));
    upd |= hub.Input(&data.angle, GH_UINT8, F("camera angle"));
    upd |= hub.Input(&data.min_error, GH_UINT8, F("min error, deg"));
    upd |= hub.Input(&data.search_time, GH_UINT8, F("search time, s"));
    upd |= hub.Input(&data.idle_time, GH_UINT8, F("idle time, s"));

    if (upd) memory.update();
}

void core0(void *p) {
    EEPROM.begin(memory.blockSize());
    memory.begin(0, 'f');

    hub.onBuild(build);
    hub.begin();

    servoX.attach(SRV_X_PIN, data.sx.min, data.sx.max, data.px.home);
    servoY.attach(SRV_Y_PIN, data.sy.min, data.sy.max, data.py.home);
    xy.setSpeed(data.s_speed);
    xy.updateCurrent();
    fan.write(data.fan_speed);
    xy.setTarget(data.px.home, data.py.home);

    GHtimer searchTmr(1);  // start 1 ms чтобы перейти в search
    GHtimer idleTmr;

    for (;;) {
        memory.tick();
        hub.tick();
        xy.tick();

        if (face.found) {
            face.found = 0;
            // Serial.printf("Detect: %d,%d,%d,%d\n", face.x, face.y, face.w, face.h);
            if (!manual) {
                fan.write(data.fan_speed);
                sys_state = 2;
                locked = 1;
                searchTmr.start(0, data.search_time);
                uint16_t cx = face.x + face.w / 2;
                uint16_t cy = face.y + face.h / 2;
                int16_t dx = -(face.frame_w / 2 - cx);
                int16_t dy = -(face.frame_h / 2 - cy);
                if (data.sx.reverse) dx = -dx;
                if (data.sy.reverse) dy = -dy;

                int16_t dxx = dx * (data.sx.max - data.sx.min) * data.angle / face.frame_w / data.sx.anlge;
                int16_t dyy = dy * (data.sy.max - data.sy.min) * (data.angle * 3 / 4) / face.frame_h / data.sy.anlge;
                dxx += data.px.offset;
                dyy += data.py.offset;
                int16_t tx = constrain(bufpos_x + dxx, data.px.min, data.px.max);
                int16_t ty = constrain(bufpos_y + dyy, data.py.min, data.py.max);
                if (abs(dx) >= data.min_error || abs(dy) >= data.min_error) xy.setTarget(tx, ty);
                else xy.stop();
                xy.setSpeed(map(xy.getTotal(), 500, 50, data.s_speed, 40));
                // xy.setSpeed(data.s_speed);

                Serial.printf("Locked!\n");
                Serial.printf("x:%d, w:%d, cx:%d, dx:%d, dxx:%d, tx:%d\n", face.x, face.w, cx, dx, dxx, tx);
                Serial.printf("y:%d, h:%d, cy:%d, dy:%d, dyy:%d, ty:%d\n", face.y, face.h, cy, dy, dyy, ty);
            }
        }
        if (!manual) {
            if (searchTmr) {
                Serial.println("State: search");
                searchTmr.stop();
                sys_state = 1;
                xy.setTarget(data.px.min, data.py.home);
                idleTmr.start(0, data.idle_time);
                xy.setSpeed(data.search_speed);
            }
            switch (sys_state) {
                case 0:  // idle
                    break;
                case 1:  // search
                    if (xy.ready()) {
                        xy.setTarget(xy.getPos(0) > data.px.home ? data.px.min : data.px.max, data.py.home);
                    }
                    if (idleTmr) {
                        Serial.println("State: idle");
                        idleTmr.stop();
                        sys_state = 0;
                        fan.write(0);
                        xy.setTarget(data.px.home, data.py.home);
                    }
                    break;
                case 2:  // locked
                    break;
            }
        }
        vTaskDelay(1);
    }
}