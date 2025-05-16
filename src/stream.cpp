#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include <HTTPClient.h>  // 用于HTTP客户端连接中转服务器
#include <ArduinoJson.h> // 用于处理JSON数据
#include <algorithm>     // 用于std::min函数

// 中转服务器配置
const char* streamServerUrl = "http://stream.link2you.top/api/stream"; // 云服务器接收API
const int streamSendInterval = 33; // 约30fps, 33ms一帧
unsigned long lastStreamSendTime = 0;

// MJPEG流处理所需常量
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
float currentFPS = 0;
unsigned long frameCount = 0;
unsigned long lastFPSCalculationTime = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastStreamSendTime >= streamSendInterval) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      HTTPClient http;
      http.begin(streamServerUrl);
      http.addHeader("Content-Type", "image/jpeg");
      http.addHeader("X-ESP32-ID", WiFi.macAddress());
      int httpResponseCode = http.POST(fb->buf, fb->len);
      http.end();
      esp_camera_fb_return(fb);
    }
    lastStreamSendTime = now;
  }
}
