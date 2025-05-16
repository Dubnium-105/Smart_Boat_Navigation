#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include <HTTPClient.h>  // 用于HTTP客户端连接中转服务器
#include <ArduinoJson.h> // 用于处理JSON数据
#include <algorithm>     // 用于std::min函数

// 中转服务器配置
const char* streamServerUrl = "stream.link2you.top";  // 更改为您的中转服务器地址
const int streamServerPort = 80;                                      // 更改为您的中转服务器端口
bool useStreamServer = false;                                         // 控制是否启用中转服务器推送
const int streamSendInterval = 500;                                    // 向中转服务器发送帧的时间间隔(毫秒)
unsigned long lastStreamSendTime = 0;                                  // 最后一次发送到中转服务器的时间

// MJPEG流处理所需常量
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
float currentFPS = 0;
unsigned long frameCount = 0;
unsigned long lastFPSCalculationTime = 0;

// 向中转服务器发送视频数据
bool sendFrameToServer(uint8_t *jpg_buf, size_t jpg_buf_len) {
  if (!useStreamServer) return false;
  
  HTTPClient http;
  
  // 建立连接到中转服务器
  http.begin(streamServerUrl);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Content-Length", String(jpg_buf_len));
  http.addHeader("X-ESP32-ID", WiFi.macAddress()); // 使用MAC地址作为设备标识
  
  // 发送POST请求
  int httpResponseCode = http.POST(jpg_buf, jpg_buf_len);
  
  // 检查响应
  bool success = false;
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("服务器中转响应: %d - %s\n", httpResponseCode, response.c_str());
    success = true;
  } else {
    Serial.printf("向中转服务器发送帧失败，错误码: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  
  http.end();
  return success;
}

// 设置是否使用中转服务器及其URL
void configureStreamServer(bool enable, const char* serverUrl = NULL) {
  useStreamServer = enable;
  if (serverUrl != NULL) {
    streamServerUrl = serverUrl;
  }
  Serial.printf("视频流中转服务器状态: %s, URL: %s\n", 
                enable ? "已启用" : "已禁用", 
                streamServerUrl);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[128];
  static int64_t last_frame = 0;
  uint8_t *dynamic_gray_buf = NULL; // 动态分配灰度缓冲区
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else {
      if (fb->format == PIXFORMAT_GRAYSCALE) {
        // 直接JPEG编码
        bool jpeg_converted = frame2jpg(fb, 40, &_jpg_buf, &_jpg_buf_len); // 降低JPEG质量提升速度
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          res = ESP_FAIL;
        }
      } else {
        // 非灰度帧，先转换为灰度再编码
        size_t gray_len = fb->width * fb->height;
        dynamic_gray_buf = (uint8_t *)malloc(gray_len);
        if (dynamic_gray_buf) {
          // 简单平均法灰度化
          for (size_t i = 0; i < gray_len; ++i) {
            uint8_t r = fb->buf[i * 3 + 0];
            uint8_t g = fb->buf[i * 3 + 1];
            uint8_t b = fb->buf[i * 3 + 2];
            dynamic_gray_buf[i] = (r * 38 + g * 75 + b * 15) >> 7;
          }
          bool jpeg_converted = fmt2jpg(dynamic_gray_buf, gray_len, fb->width, fb->height, PIXFORMAT_GRAYSCALE, 40, &_jpg_buf, &_jpg_buf_len);
          free(dynamic_gray_buf);
          dynamic_gray_buf = NULL;
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            res = ESP_FAIL;
          }
        } else {
          esp_camera_fb_return(fb);
          fb = NULL;
          res = ESP_FAIL;
        }
      }
    }
    if (res == ESP_OK) {
      // 检查是否需要发送到中转服务器
      unsigned long currentTime = millis();
      if (useStreamServer && _jpg_buf != NULL && _jpg_buf_len > 0 && 
          (currentTime - lastStreamSendTime >= streamSendInterval)) {
        if (sendFrameToServer(_jpg_buf, _jpg_buf_len)) {
          lastStreamSendTime = currentTime;
        }
      }
      
      // 合并头和JPEG数据发送到HTTP客户端
      size_t hlen = snprintf(part_buf, sizeof(part_buf), "%s%s%u%s", _STREAM_BOUNDARY, "Content-Type: image/jpeg\r\nContent-Length: ", (unsigned)_jpg_buf_len, "\r\n\r\n");
      // 先发送边界+头
      res = httpd_resp_send_chunk(req, part_buf, hlen);
      if (res == ESP_OK) {
        // 再发送JPEG数据
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
    }
    if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    frameCount++;
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    if ((esp_timer_get_time() - lastFPSCalculationTime) / 1000 > 1000) {
      currentFPS = frameCount * 1000.0 / ((esp_timer_get_time() - lastFPSCalculationTime) / 1000);
      lastFPSCalculationTime = esp_timer_get_time();
      frameCount = 0;
    }
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  return stream_handler(req);
}

// REST API端点，用于配置视频流设置
static esp_err_t cmd_handler(httpd_req_t *req) {
  char content[100];
  size_t recv_size = std::min(req->content_len, sizeof(content) - 1); // 使用std::min替代MIN

  int ret = httpd_req_recv(req, content, recv_size);
  if (ret <= 0) {
    return ESP_FAIL;
  }
  content[ret] = '\0';

  // 解析JSON命令
  DynamicJsonDocument doc(256); // 使用DynamicJsonDocument替代JsonDocument
  DeserializationError error = deserializeJson(doc, content);

  if (!error) {
    if (doc["stream_server"].is<JsonObject>()) {
      bool enable = doc["stream_server"]["enable"].as<bool>();
      
      if (doc["stream_server"]["url"].is<const char*>()) {
        const char* url = doc["stream_server"]["url"];
        configureStreamServer(enable, url);
      } else {
        configureStreamServer(enable);
      }
      
      // 返回成功
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      char resp[100];
      snprintf(resp, sizeof(resp), "{\"success\":true,\"stream_server\":{\"enable\":%s,\"url\":\"%s\"}}", 
              useStreamServer ? "true" : "false", streamServerUrl);
      httpd_resp_send(req, resp, strlen(resp));
      return ESP_OK;
    }
  }
  
  // 返回错误
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, "{\"success\":false,\"message\":\"Invalid command\"}", -1);
  return ESP_OK;
}

void startSimpleCameraStream() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t cmd_uri = {
    .uri = "/api/config",
    .method = HTTP_POST,
    .handler = cmd_handler,
    .user_ctx = NULL
  };
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &cmd_uri);
    Serial.println("HTTP服务器启动成功");
  } else {
    Serial.println("HTTP服务器启动失败");
  }
}
