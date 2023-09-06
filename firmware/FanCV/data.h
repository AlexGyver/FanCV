#pragma once
#include <Arduino.h>
#include <EEManager.h>

struct servo_p {
    uint16_t min = 500;
    uint16_t max = 2200;
    uint16_t anlge = 180;
    bool reverse = 0;
};
struct planner_p {
    uint16_t min = 500;
    uint16_t max = 2200;
    uint16_t home = 1500;
    int16_t offset = 0;
};

struct Data {
    servo_p sx;
    servo_p sy;
    planner_p px;
    planner_p py;
    uint8_t angle = 120;
    uint8_t search_time = 10;
    uint8_t idle_time = 120;
    uint16_t s_speed = 90;
    uint8_t fan_speed = 100;
    uint8_t search_speed = 110;
    uint8_t min_error = 10;
};

Data data;
EEManager memory(data);