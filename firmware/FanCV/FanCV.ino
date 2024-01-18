#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#include "camera.h"
#include "config.h"
#include "core0.h"
#include "img_converters.h"

WebSocketsServer ws(82, "", "hub");
TaskHandle_t Task0;

void setup() {
    Serial.begin(115200);
    delay(200);
    cam_init(FRAMESIZE_VGA, PIXFORMAT_JPEG, 10);
    // cam_init(FRAMESIZE_VGA, PIXFORMAT_RGB565);

    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println(WiFi.localIP());

    ws.begin();

    xTaskCreatePinnedToCore(core0, "Task0", 10000, NULL, 1, &Task0, 0);
}

void loop() {
    ws.loop();
    // if (!ws.connectedClients()) return;

    // camera_fb_t *fbj = esp_camera_fb_get();
    // if (fbj) ws.broadcastBIN(fbj->buf, fbj->len);
    // esp_camera_fb_return(fbj);
    // delay(500);

    // camera_fb_t *fbj = esp_camera_fb_get();
    // if (fbj) {
    //     for (uint32_t i = 0; i < fbj->len; i += 2) {
    //         uint8_t b = fbj->buf[i];
    //         fbj->buf[i] = fbj->buf[i + 1];
    //         fbj->buf[i + 1] = b;
    //     }
    //     size_t jpg_buf_len = 0;
    //     uint8_t *jpg_buf = nullptr;
    //     fmt2jpg(fbj->buf, fbj->len, fbj->width, fbj->height, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_buf_len);
    //     if (jpg_buf) {
    //         ws.broadcastBIN(jpg_buf, jpg_buf_len);
    //         free(jpg_buf);
    //     }
    // }
    // esp_camera_fb_return(fbj);
    // return;

    // skip frame
    camera_fb_t *fbj = nullptr;
    fbj = esp_camera_fb_get();
    esp_camera_fb_return(fbj);

    fbj = nullptr;
    fbj = esp_camera_fb_get();
    bufpos_x = xy.getPos(0);
    bufpos_y = xy.getPos(1);

    if (fbj) {
        uint32_t len = fbj->width * fbj->height * 2;
        uint8_t *buf = (uint8_t *)ps_malloc(len);

        if (buf) {
            bool ok = jpg2rgb565(fbj->buf, fbj->len, buf, JPG_SCALE_NONE);
            if (ok) {
                // swap low->high byte
                for (uint32_t i = 0; i < len; i += 2) {
                    uint8_t b = buf[i];
                    buf[i] = buf[i + 1];
                    buf[i + 1] = b;
                }
                face.find(buf, fbj->width, fbj->height);
                // if (face.found) Serial.printf("%d,%d,%d,%d\n", face.x, face.y, face.w, face.h);

                if (ws.connectedClients()) {
                    size_t jpg_buf_len = 0;
                    uint8_t *jpg_buf = nullptr;
                    ok = fmt2jpg(buf, len, fbj->width, fbj->height, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_buf_len);
                    if (ok) ws.broadcastBIN(jpg_buf, jpg_buf_len);
                    if (jpg_buf) free(jpg_buf);
                }
            }
            free(buf);
        }
    }
    esp_camera_fb_return(fbj);
    delay(30);
}
