#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include <HTTPClient.h>  // 用于HTTP客户端连接中转服务器

// 中转服务器配置
const char* streamServerUrl = "http://stream.link2you.top/api/stream"; // 云服务器接收API
const int streamSendInterval =1000/60;
unsigned long lastStreamSendTime = 0;

float currentFPS = 0; // 供MQTT状态上报用
bool useStreamServer = false; // 供MQTT状态上报用

// MJPEG流处理所需常量
#define PART_BOUNDARY "123456789000000000000987654321"

httpd_handle_t stream_httpd = NULL;

// 获取当前时间戳字符串
String getTimestamp() {
  char buf[32];
  unsigned long ms = millis();
  unsigned long sec = ms / 1000;
  unsigned long min = sec / 60;
  unsigned long hour = min / 60;
  snprintf(buf, sizeof(buf), "[%02lu:%02lu:%02lu.%03lu] ", hour % 24, min % 60, sec % 60, ms % 1000);
  return String(buf);
}

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
      http.end();
      esp_camera_fb_return(fb);
    }
    lastStreamSendTime = now;
  }
}

// 配置是否使用中转服务器以及服务器URL
void configureStreamServer(bool enable, const char* serverUrl) {
  if (enable && serverUrl != NULL) {
    // 直接设置服务器URL
    // 注意：如果serverUrl是临时字符串，可能需要复制它
    streamServerUrl = serverUrl;
  }
}

// 静态函数：根路径返回 HTML 页面
esp_err_t index_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  // 获取当前配置
  int framesize = s->status.framesize;
  int quality = s->status.quality;
  int brightness = s->status.brightness;
  int contrast = s->status.contrast;
  // 分辨率value字符串
  const char* sizeStr = "VGA";
  switch(framesize) {
    case FRAMESIZE_96X96: sizeStr = "96X96"; break;
    case FRAMESIZE_QQVGA: sizeStr = "QQVGA"; break;
    case FRAMESIZE_QCIF: sizeStr = "QCIF"; break;
    case FRAMESIZE_HQVGA: sizeStr = "HQVGA"; break;
    case FRAMESIZE_240X240: sizeStr = "240X240"; break;
    case FRAMESIZE_QVGA: sizeStr = "QVGA"; break;
    case FRAMESIZE_CIF: sizeStr = "CIF"; break;
    case FRAMESIZE_HVGA: sizeStr = "HVGA"; break;
    case FRAMESIZE_VGA: sizeStr = "VGA"; break;
    case FRAMESIZE_SVGA: sizeStr = "SVGA"; break;
    case FRAMESIZE_XGA: sizeStr = "XGA"; break;
    case FRAMESIZE_HD: sizeStr = "HD"; break;
    case FRAMESIZE_SXGA: sizeStr = "SXGA"; break;
    case FRAMESIZE_UXGA: sizeStr = "UXGA"; break;
    default: sizeStr = "VGA"; break;
  }
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 Camera</title>"
    "<style>body{margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;}"
    "#panel{position:fixed;top:20px;right:20px;background:rgba(30,30,30,0.8);padding:16px 20px;border-radius:10px;z-index:10;color:#fff;}"
    "label{margin-right:8px;}select,input{padding:2px 8px;border-radius:4px;margin-right:8px;}"
    "img{max-width:100vw;max-height:100vh;box-shadow:0 0 16px #000;}"
    "#msg{margin-top:8px;color:#ffd700;font-size:1rem;}"
    "</style>"
    "<script>console.log('[DEBUG] 页面加载');</script>"
    "</head><body>"
    "<div id='panel'>"
    "<label for='resSel'>分辨率:</label>"
    "<select id='resSel'>"
    "<option value='96X96'" + String(strcmp(sizeStr,"96X96")==0?" selected":"") + ">96x96</option>"
    "<option value='QQVGA'" + String(strcmp(sizeStr,"QQVGA")==0?" selected":"") + ">160x120</option>"
    "<option value='QCIF'" + String(strcmp(sizeStr,"QCIF")==0?" selected":"") + ">176x144</option>"
    "<option value='HQVGA'" + String(strcmp(sizeStr,"HQVGA")==0?" selected":"") + ">240x176</option>"
    "<option value='240X240'" + String(strcmp(sizeStr,"240X240")==0?" selected":"") + ">240x240</option>"
    "<option value='QVGA'" + String(strcmp(sizeStr,"QVGA")==0?" selected":"") + ">320x240</option>"
    "<option value='CIF'" + String(strcmp(sizeStr,"CIF")==0?" selected":"") + ">352x288(60FPS)</option>"
    "<option value='HVGA'" + String(strcmp(sizeStr,"HVGA")==0?" selected":"") + ">480x320</option>"
    "<option value='VGA'" + String(strcmp(sizeStr,"VGA")==0?" selected":"") + ">640x480</option>"
    "<option value='SVGA'" + String(strcmp(sizeStr,"SVGA")==0?" selected":"") + ">800x600</option>"
    "<option value='XGA'" + String(strcmp(sizeStr,"XGA")==0?" selected":"") + ">1024x768</option>"
    "<option value='HD'" + String(strcmp(sizeStr,"HD")==0?" selected":"") + ">1280x720</option>"
    "<option value='SXGA'" + String(strcmp(sizeStr,"SXGA")==0?" selected":"") + ">1280x1024</option>"
    "<option value='UXGA'" + String(strcmp(sizeStr,"UXGA")==0?" selected":"") + ">1600x1200(15FPS)</option>"
    "</select>"
    "<label for='quality'>画质:</label>"
    "<input id='quality' type='number' min='10' max='63' value='" + String(quality) + "' style='width:50px;'>"
    "<label for='brightness'>亮度:</label>"
    "<input id='brightness' type='number' min='-2' max='2' value='" + String(brightness) + "' style='width:40px;'>"
    "<label for='contrast'>对比度:</label>"
    "<input id='contrast' type='number' min='-2' max='2' value='" + String(contrast) + "' style='width:40px;'>"
    "<button id='setBtn'>应用</button>"
    "<div id='msg'></div>"
    "</div>"
    "<img id='cam' src='/stream' alt='Camera Stream'>"
    "<script>\nconst img=document.getElementById('cam');\nconst sel=document.getElementById('resSel');\nconst quality=document.getElementById('quality');\nconst brightness=document.getElementById('brightness');\nconst contrast=document.getElementById('contrast');\nconst setBtn=document.getElementById('setBtn');\nconst msg=document.getElementById('msg');\nfunction showMsg(t){msg.textContent=t;setTimeout(()=>{msg.textContent='';},1500);}"
    "img.onerror=function(){console.log('[DEBUG] 视频流断开，尝试重连');setTimeout(()=>{img.src='/stream?'+Date.now()},1000);};"
    "img.onload=function(){console.log('[DEBUG] 视频流已连接');};"
    "setBtn.onclick=function(){\n  img.src=''; /* 先断开流 */\n  let url='/set_quality?size='+sel.value+'&quality='+quality.value+'&brightness='+brightness.value+'&contrast='+contrast.value;\n  fetch(url).then(r=>r.text()).then(t=>{showMsg('设置成功');setTimeout(()=>{img.src='/stream?'+Date.now()},500);});\n};"
    "</script>"
    "</body></html>";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html.c_str(), html.length());
  return ESP_OK;
}

// 静态函数：/stream 返回 MJPEG
esp_err_t stream_handler(httpd_req_t *req) {
  static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
  static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
  static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  char part_buf[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println(getTimestamp() + "获取摄像头帧失败");
      continue;
    }
    size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
    if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK ||
        httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      Serial.println(getTimestamp() + "客户端断开，退出流循环");
      break;
    }
    esp_camera_fb_return(fb);
    vTaskDelay(33 / portTICK_PERIOD_MS);
  }
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// 新增：动态设置摄像头分辨率的HTTP handler
esp_err_t set_quality_handler(httpd_req_t *req) {
  char buf[64];
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    char param[20];
    httpd_req_get_url_query_str(req, buf, buf_len);
    sensor_t *s = esp_camera_sensor_get();
    // 删除xclk相关后端处理逻辑
    // 分辨率
    if (httpd_query_key_value(buf, "size", param, sizeof(param)) == ESP_OK) {
      if (strcmp(param, "96X96") == 0) s->set_framesize(s, FRAMESIZE_96X96);
      else if (strcmp(param, "QQVGA") == 0) s->set_framesize(s, FRAMESIZE_QQVGA);
      else if (strcmp(param, "QCIF") == 0) s->set_framesize(s, FRAMESIZE_QCIF);
      else if (strcmp(param, "HQVGA") == 0) s->set_framesize(s, FRAMESIZE_HQVGA);
      else if (strcmp(param, "240X240") == 0) s->set_framesize(s, FRAMESIZE_240X240);
      else if (strcmp(param, "QVGA") == 0) s->set_framesize(s, FRAMESIZE_QVGA);
      else if (strcmp(param, "CIF") == 0) s->set_framesize(s, FRAMESIZE_CIF);
      else if (strcmp(param, "HVGA") == 0) s->set_framesize(s, FRAMESIZE_HVGA);
      else if (strcmp(param, "VGA") == 0) s->set_framesize(s, FRAMESIZE_VGA);
      else if (strcmp(param, "SVGA") == 0) s->set_framesize(s, FRAMESIZE_SVGA);
      else if (strcmp(param, "XGA") == 0) s->set_framesize(s, FRAMESIZE_XGA);
      else if (strcmp(param, "HD") == 0) s->set_framesize(s, FRAMESIZE_HD);
      else if (strcmp(param, "SXGA") == 0) s->set_framesize(s, FRAMESIZE_SXGA);
      else if (strcmp(param, "UXGA") == 0) s->set_framesize(s, FRAMESIZE_UXGA);
      Serial.println(getTimestamp() + String("切换分辨率为: ") + param);
    }
    // 画质
    if (httpd_query_key_value(buf, "quality", param, sizeof(param)) == ESP_OK) {
      int q = atoi(param);
      if (q >= 10 && q <= 63) s->set_quality(s, q);
      Serial.println(getTimestamp() + String("设置画质: ") + q);
    }
    // 亮度
    if (httpd_query_key_value(buf, "brightness", param, sizeof(param)) == ESP_OK) {
      int b = atoi(param);
      if (b >= -2 && b <= 2) s->set_brightness(s, b);
      Serial.println(getTimestamp() + String("设置亮度: ") + b);
    }
    // 对比度
    if (httpd_query_key_value(buf, "contrast", param, sizeof(param)) == ESP_OK) {
      int c = atoi(param);
      if (c >= -2 && c <= 2) s->set_contrast(s, c);
      Serial.println(getTimestamp() + String("设置对比度: ") + c);
    }
    // ...不再处理xclk...
  }
  httpd_resp_sendstr(req, "ok");
  return ESP_OK;
}

void startSimpleCameraStream() {
  Serial.println(getTimestamp() + "启动视频流服务器...");
  if (stream_httpd) {
    Serial.println(getTimestamp() + "HTTP流服务器已在运行");
    return;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80; // 默认80端口
  config.max_uri_handlers = 8;
  config.stack_size = 16384;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  // 新增分辨率切换接口
  httpd_uri_t set_quality_uri = {
    .uri       = "/set_quality",
    .method    = HTTP_GET,
    .handler   = set_quality_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &set_quality_uri);
    Serial.println(getTimestamp() + "本地HTTP流服务已启动: http://<板子IP>/");
  } else {
    Serial.println(getTimestamp() + "启动本地HTTP流服务失败");
  }
}
