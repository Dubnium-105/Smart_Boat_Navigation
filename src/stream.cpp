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
bool useStreamServer = false; // 是否将视频流发送到中转服务器

// 将原loop函数重命名为streamLoop，避免与Arduino主循环冲突
void streamLoop() {
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

// 配置是否使用中转服务器以及服务器URL
void configureStreamServer(bool enable, const char* serverUrl) {
  useStreamServer = enable;
  if (enable && serverUrl != NULL) {
    // 直接设置服务器URL
    // 注意：如果serverUrl是临时字符串，可能需要复制它
    streamServerUrl = serverUrl;
  }
}

// 启动简单的摄像头流服务器
void startSimpleCameraStream() {
  // 实现摄像头流服务器启动逻辑
  // 这个函数在main.cpp中被调用
  Serial.println("启动视频流服务器...");
  // 在这里可以添加启动HTTP服务器的具体实现
}
