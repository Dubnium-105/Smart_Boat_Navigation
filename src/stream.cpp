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
unsigned long lastLogTime = 0; // 记录上次日志输出时间
bool useStreamServer = false; // 是否将视频流发送到中转服务器

// 将原loop函数重命名为streamLoop，避免与Arduino主循环冲突
void streamLoop() {
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
        Serial.printf("[推流] 帧大小: %d bytes, 采集: %lums, 上传: %lums\n", fb->len, t1-t0, t2-t1);
        if (httpResponseCode > 0) {
          Serial.printf("[推流] 上传图片成功，响应码: %d\n", httpResponseCode);
        } else {
          Serial.printf("[推流] 上传图片失败，错误: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        Serial.println();
        lastLogTime = now;
      }
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
  Serial.println("启动视频流服务器...");
  if (stream_httpd) {
    Serial.println("HTTP流服务器已在运行");
    return;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80; // 默认80端口
  config.max_uri_handlers = 8;
  config.stack_size = 8192;

  // MJPEG流处理handler
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = [](httpd_req_t *req) -> esp_err_t {
      // 设置响应头
      httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
      char part_buf[64];
      while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
          Serial.println("获取摄像头帧失败");
          continue;
        }
        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
        httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        httpd_resp_send_chunk(req, part_buf, hlen);
        httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        // 控制帧率
        vTaskDelay(33 / portTICK_PERIOD_MS); // 约30fps
      }
      httpd_resp_send_chunk(req, NULL, 0);
      return ESP_OK;
    },
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("本地HTTP流服务已启动: http://<板子IP>/stream");
  } else {
    Serial.println("启动本地HTTP流服务失败");
  }
}
