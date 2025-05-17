/**
 * ESP32-S3摄像头手动船舶控制系统 - 主程序
 * 
 * 功能:
 * - 初始化摄像头、WiFi、MQTT、电机
 * - 启动视频流服务器并推送至中转服务器
 * - 创建后台任务 (串口监控、系统监控)
 * - 主循环处理摄像头帧和MQTT消息
 * 推送master
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_server.h" // 添加HTTP服务器头文件

// --- 包含需要的模块的头文件 --- 
#include "camera_setup.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "tasks.h" // 包含任务创建函数和外部声明

// --- 声明外部函数和变量 ---
extern void startSimpleCameraStream(); // 来自 stream.cpp
extern httpd_handle_t stream_httpd;    // 来自 stream.cpp
extern float currentFPS;               // 来自 stream.cpp
extern unsigned long frameCount;       // 来自 stream.cpp
extern unsigned long lastFPSCalculationTime; // 来自 stream.cpp

// 摄像头分辨率结构体声明
struct CustomCameraResolution {
    int width;
    int height;
};

// 摄像头分辨率映射表 - 改名以避免与ESP32 Camera API的定义冲突
static const CustomCameraResolution camera_resolution[] = {
    { 96, 96 },     // FRAMESIZE_96X96,    0
    { 160, 120 },   // FRAMESIZE_QQVGA,    1
    { 176, 144 },   // FRAMESIZE_QCIF,     2
    { 240, 176 },   // FRAMESIZE_HQVGA,    3
    { 240, 240 },   // FRAMESIZE_240X240,  4
    { 320, 240 },   // FRAMESIZE_QVGA,     5
    { 400, 296 },   // FRAMESIZE_CIF,      6
    { 480, 320 },   // FRAMESIZE_HVGA,     7
    { 640, 480 },   // FRAMESIZE_VGA,      8
    { 800, 600 },   // FRAMESIZE_SVGA,     9
    { 1024, 768 },  // FRAMESIZE_XGA,      10
    { 1280, 720 },  // FRAMESIZE_HD,       11
    { 1280, 1024 }, // FRAMESIZE_SXGA,     12
    { 1600, 1200 }, // FRAMESIZE_UXGA,     13
};

// 获取摄像头配置的函数
sensor_t* sensor_get_config() {
    return esp_camera_sensor_get();
}

// --- 全局共享变量和同步原语 ---
SemaphoreHandle_t frameAccessMutex = NULL;
volatile bool systemIsBusy = false;      // 系统繁忙标志 (由systemMonitorTask更新)
bool cameraAvailable = false; // 全局摄像头可用标志，用于指示摄像头是否成功初始化。

// --- Arduino Setup ---
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      船舶手动控制系统启动中...     ");
  Serial.println("====================================");

  // 1. 创建互斥锁 (必须在任务使用前创建)
  frameAccessMutex = xSemaphoreCreateMutex();
  if (frameAccessMutex == NULL) {
    Serial.println("错误: 无法创建帧访问互斥锁! 重启...");
    delay(1000);
    ESP.restart();
  }
  
  // 2. 初始化摄像头 (增加重试机制)
  const int maxCameraRetries = 3;
  int cameraRetryCount = 0;
  while (cameraRetryCount < maxCameraRetries) {
    cameraAvailable = setupCamera();
    if (cameraAvailable) {
      printCameraSettings(); // 打印摄像头配置
      break;
    }
    Serial.printf("摄像头初始化失败，重试中 (%d/%d)...\n", cameraRetryCount + 1, maxCameraRetries);
    cameraRetryCount++;
    delay(1000); // 等待1秒后重试
  }
  if (!cameraAvailable) {
    Serial.println("摄像头初始化失败，启用降级模式：仅WiFi和MQTT功能");
  }

  // 3. 初始化电机
  setup_motors();

  // 4. 连接WiFi
  if (!connectWiFi()) {
    Serial.println("WiFi连接失败，重启...");
    delay(1000);
    ESP.restart();
  }

  // 5. 初始化MQTT客户端
  setupMQTT();
  // 首次尝试连接MQTT (如果WiFi连接成功)
  if (WiFi.status() == WL_CONNECTED) {
      mqtt_reconnect(); 
  }

  // 6. 初始化性能监控变量 
  lastFPSCalculationTime = esp_timer_get_time();
  frameCount = 0;

  // 7. 启动视频流服务器
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");

  // 8. 创建后台任务 (固定到核心0以减少对摄像头/流的影响)
  createSerialMonitorTask(0); 
  createSystemMonitorTask(0);

  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  if (WiFi.status() == WL_CONNECTED) {
      Serial.print("访问: http://");
      Serial.print(WiFi.localIP());
      Serial.println(" 查看视频流");
  }
  Serial.println("====================================");
}

// --- Arduino Loop ---
void loop() {
  if (!wifiAutoReconnect()) {
    return;
  }
  static unsigned long lastMqttReconnectAttempt = 0;
  unsigned long currentTime = millis();

  // 1. 处理MQTT连接和消息
  if (!mqttClient.connected()) {
    if (currentTime - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = currentTime;
      Serial.println("MQTT断开连接，尝试重连...");
      mqtt_reconnect();
    }
  } else {
    mqttClient.loop();
  }

  // 2. 视频流处理
  if (cameraAvailable && !systemIsBusy) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // 视频流通过HTTP服务器自动处理
      esp_camera_fb_return(fb);
    }
  }

  delay(2); // 保持高帧率
}
