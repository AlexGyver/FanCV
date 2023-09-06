#pragma once
#include <Arduino.h>

#include <vector>

#include "fb_gfx.h"
#include "human_face_detect_mnp01.hpp"
#include "human_face_detect_msr01.hpp"
#include "img_converters.h"

// #include "cat_face_detect_mn03.hpp"

class FaceFinder {
   public:
    bool find(uint8_t *buf565, uint16_t width, uint16_t height, bool draw = true, bool fill = false) {
        found = 0;
        frame_w = width;
        frame_h = height;
        {
            HumanFaceDetectMSR01 s1(0.1F, 0.5F, 2, 0.3F);
            HumanFaceDetectMNP01 s2(0.4F, 0.3F, 1);
            std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)buf565, {height, width, 3});
            std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)buf565, {height, width, 3}, candidates);

            // CatFaceDetectMN03 cat(0.4F, 0.3F, 10, 0.3F);
            // std::list<dl::detect::result_t> &results = cat.infer((uint16_t *)buf565, {height, width, 3});

            if (!results.size()) return 0;

            std::list<dl::detect::result_t>::iterator prediction = results.begin();
            x = (int)prediction->box[0];
            y = (int)prediction->box[1];
            w = (int)prediction->box[2] - x + 1;
            h = (int)prediction->box[3] - y + 1;
            if ((x + w) > width) w = width - x;
            if ((y + h) > height) h = height - y;
            results.end();
        }

        if (draw) {
            fb_data_t fbd;
            fbd.width = width;
            fbd.height = height;
            fbd.data = buf565;
            fbd.bytes_per_pixel = 2;
            fbd.format = FB_RGB565;
            uint32_t color = 0b1111100000000000;
            if (fill) {
                fb_gfx_fillRect(&fbd, x, y, w, h, color);
            } else {
                fb_gfx_drawFastHLine(&fbd, x, y, w, color);
                fb_gfx_drawFastHLine(&fbd, x, y + h - 1, w, color);
                fb_gfx_drawFastVLine(&fbd, x, y, h, color);
                fb_gfx_drawFastVLine(&fbd, x + w - 1, y, h, color);
            }
        }
        found = 1;
        return 1;
    }

    int16_t x, y, w, h;
    uint16_t frame_w, frame_h;
    bool found = 0;
};