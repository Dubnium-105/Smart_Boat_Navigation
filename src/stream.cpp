#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include <HTTPClient.h>  // 用于HTTP客户端连接中转服务器
#include <ArduinoJson.h> // 用于处理JSON数据
#include <algorithm>     // 用于std::min函数

// 中转服务器配置
const char* streamServerUrl = "http://stream.link2you.top/api/stream"; // 云服务器接收API
const int streamSendInterval = 50;
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
  static unsigned long lastLogTime = 0;
  unsigned long now = millis();
  if (now - lastStreamSendTime >= streamSendInterval) {
    unsigned long t0 = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    unsigned long t1 = millis();
    if (fb) {
      HTTPClient http;
      http.begin(streamServerUrl);
      http.addHeader("Content-Type", "image/jpeg");
      http.addHeader("X-ESP32-ID", WiFi.macAddress());
      int httpResponseCode = http.POST(fb->buf, fb->len);
      unsigned long t2 = millis();
      // 每5秒输出一次推流信息和耗时
      if (now - lastLogTime > 5000) {
        Serial.printf("[推流] 帧大小: %d bytes, 采集: %lums, 上传: %lums, 帧头: %02X %02X %02X %02X\n", fb->len, t1-t0, t2-t1, fb->buf[0], fb->buf[1], fb->buf[2], fb->buf[3]);
        if (httpResponseCode > 0) {
          Serial.printf("[推流] 上传图片成功，响应码: %d\n", httpResponseCode);
        } else {
          Serial.printf("[推流] 上传图片失败，错误: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        // 输出板子性能占用情况
        Serial.printf("[系统] Free Heap: %u bytes, Min Free Heap: %u bytes", ESP.getFreeHeap(), ESP.getMinFreeHeap());
        #ifdef BOARD_HAS_PSRAM
        Serial.printf(", PSRAM: %u bytes", ESP.getFreePsram());
        #endif
        Serial.println();
        lastLogTime = now;
      }
      http.end();
      esp_camera_fb_return(fb);
    } else {
      if (now - lastLogTime > 5000) {
        Serial.println("[推流] 获取摄像头帧失败");
        lastLogTime = now;
      }
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
