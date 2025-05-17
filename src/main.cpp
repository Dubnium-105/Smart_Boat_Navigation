/**
 * ESP32-S3摄像头手动船舶控制系统 - 主程序
 * 
 * 功能:
 * - 初始化摄像头、WiFi、MQTT、电机
 * - 启动视频流服务器并推送至中转服务器
 * - 创建后台任务 (串口监控、系统监控)
 * - 主循环处理摄像头帧和MQTT消息
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_server.h" // 添加HTTP服务器头文件
#include "mqtt_video.h" // 新增，声明mqttVideoSendFrame等

// --- 包含需要的模块的头文件 --- 
#include "camera_setup.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "tasks.h"
#include "navigation.h"
#include "find_brightest.h"

// --- 声明外部函数和变量 ---
extern void startSimpleCameraStream(); // 来自 stream.cpp
extern httpd_handle_t stream_httpd;    // 来自 stream.cpp
extern void streamLoop(); // 来自 stream.cpp，主动推流函数

// 前置声明（解决未定义标识符编译错误）
void setup_ir_sensors();
void setup_mpu();

// --- 全局共享变量和同步原语 ---
SemaphoreHandle_t frameAccessMutex = NULL;
volatile int sharedBrightX = -1;
volatile int sharedBrightY = -1;
volatile bool newBrightPointAvailable = false;
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

  // 3. 初始化红外传感器、MPU、电机
  //setup_ir_sensors();
  //setup_mpu();
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

  // 6. 初始化导航系统
  initNavigation();

  // 7. 启动视频流服务器
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");

  // 8. 创建后台任务 (固定到核心0以减少对摄像头/流的影响)
  createSerialMonitorTask(0); 
  createSystemMonitorTask(0);

  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  if (WiFi.status() == WL_CONNECTED) {}
  Serial.println("====================================");
}

// --- Arduino Loop ---
void loop() {
  static unsigned long lastMqttReconnect = 0;
  static unsigned long lastSensorDataSend = 0;
  static unsigned long lastInterruptCheck = 0;
  unsigned long now = millis();

  // 检查WiFi连接状态
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi连接断开，尝试重连");
    connectWiFi();
    return;
  }

  // 检查MQTT连接状态
  if (!mqttClient.connected()) {
    if (now - lastMqttReconnect > 5000) {
      lastMqttReconnect = now;
      mqtt_reconnect();
    }
  } else {
    loopMQTT();
  }

  // 检查是否有新最亮点
  bool shouldProcess = false;
  if (newBrightPointAvailable) {
    shouldProcess = true;
    newBrightPointAvailable = false;
  }

  // 处理摄像头帧和导航
  if (cameraAvailable) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      int bx = -1, by = -1;
      find_brightest(fb->buf, fb->width, fb->height, bx, by);
      sharedBrightX = bx;
      sharedBrightY = by;
      newBrightPointAvailable = true;
      navigationStateMachine(fb, bx, by);
      esp_camera_fb_return(fb);
    }
    streamLoop(); // 保持视频流
  }

  // 定期发送传感器数据
  if (now - lastSensorDataSend > 100) {
    lastSensorDataSend = now;
    // 可选: 发送红外/MPU等传感器数据到MQTT
    // send_sensor_data();
  }

  // 周期性清除并重新读取中断状态，防止中断标志丢失
  if (now - lastInterruptCheck > 1000) {
    lastInterruptCheck = now;
    // ...如有中断相关处理可补充...
  }

  delay(2);
}
