/**
 * ESP32-S3摄像头智能船导航系统 - 主程序
 * 
 * 功能:
 * - 初始化摄像头、WiFi、MQTT、电机、导航系统
 * - 启动视频流服务器
 * - 创建后台任务 (串口监控、系统监控)
 * - 主循环处理摄像头帧、MQTT消息和导航逻辑
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_server.h" // 添加HTTP服务器头文件

// --- 包含新模块的头文件 ---
#include "camera_setup.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "navigation.h"
#include "tasks.h" // 包含任务创建函数和外部声明

// --- 声明外部函数和变量 (来自其他未模块化的文件) ---
extern "C" { // 如果find_brightest.cpp是C代码编译的，需要这个
    #include "find_brightest.h" // 包含find_brightest函数声明
}
// extern void find_brightest(const uint8_t* gray, int width, int height, int& out_x, int& out_y); // 或者直接声明
extern void startSimpleCameraStream(); // 来自 stream.cpp
extern httpd_handle_t stream_httpd;    // 来自 stream.cpp
extern float currentFPS;               // 来自 stream.cpp (或由tasks.h提供)
extern unsigned long frameCount;       // 来自 stream.cpp
extern unsigned long lastFPSCalculationTime; // 来自 stream.cpp

// --- 全局共享变量和同步原语 ---
SemaphoreHandle_t frameAccessMutex = NULL;
volatile int sharedBrightX = -1; // 初始化为无效值
volatile int sharedBrightY = -1;
volatile bool newBrightPointAvailable = false;
volatile bool systemIsBusy = false;      // 系统繁忙标志 (由systemMonitorTask更新)

// --- 帧处理函数 ---
// 在每个帧处理时手动调用的函数 (可以考虑移入camera_processing模块)
void processFrame(camera_fb_t *fb) {
  static uint32_t last_frame_time = 0;
  uint32_t now = millis();
  
  // 降低处理频率，例如每200ms处理一次最亮点查找
  if (now - last_frame_time > 200) {
    last_frame_time = now;
    
    // 尝试获取互斥锁，超时时间短以避免阻塞视频流
    if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (fb && fb->format == PIXFORMAT_GRAYSCALE) {
        int bx, by;
        // 调用最亮点查找函数 (假设已正确包含或声明)
        find_brightest(fb->buf, fb->width, fb->height, bx, by);
        
        // 更新共享变量
        sharedBrightX = bx;
        sharedBrightY = by;
        newBrightPointAvailable = true; // 标记有新数据
      }
      xSemaphoreGive(frameAccessMutex); // 释放锁
    } else {
        // Serial.println("processFrame: 未能获取锁"); // 调试信息
    }
  }
}


// --- Arduino Setup ---
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      智能船导航系统启动中...     ");
  Serial.println("====================================");

  // 1. 创建互斥锁 (必须在任务使用前创建)
  frameAccessMutex = xSemaphoreCreateMutex();
  if (frameAccessMutex == NULL) {
    Serial.println("错误: 无法创建帧访问互斥锁! 重启...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("帧访问互斥锁已创建");

  // 2. 初始化摄像头
  if (!setupCamera()) {
    Serial.println("摄像头初始化失败，重启...");
    delay(1000);
    ESP.restart();
  }
  printCameraSettings(); // 打印摄像头配置

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

  // 6. 初始化导航系统
  initNavigation();

  // 7. 初始化性能监控变量 (移到stream.cpp或全局?)
  lastFPSCalculationTime = esp_timer_get_time();
  frameCount = 0;

  // 8. 启动视频流服务器 (假设在核心1运行)
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");

  // 9. 创建后台任务 (固定到核心0以减少对摄像头/流的影响)
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
  static unsigned long lastMqttReconnectAttempt = 0;
  static unsigned long lastFrameProcessTime = 0;
  static unsigned long lastNavigationUpdateTime = 0;
  unsigned long currentTime = millis();

  // 1. 处理MQTT连接和消息
  if (!mqttClient.connected()) {
    // 每隔5秒尝试重连MQTT
    if (currentTime - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = currentTime;
      Serial.println("MQTT断开连接，尝试重连...");
      mqtt_reconnect();
    }
  } else {
    mqttClient.loop(); // 处理传入消息和保持连接
  }

  // 2. 获取并处理摄像头帧 (如果系统不繁忙)
  // 增加帧处理间隔，例如100ms获取一次图像
  if (!systemIsBusy && (currentTime - lastFrameProcessTime >= 100)) { 
    lastFrameProcessTime = currentTime;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // a. 处理帧数据 (例如，查找最亮点)
      processFrame(fb); 
      
      // b. 安全地获取最新的最亮点数据 (用于导航)
      int currentBrightX = -1, currentBrightY = -1;
      if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        // 读取共享变量，即使newBrightPointAvailable是false也读取当前值
        currentBrightX = sharedBrightX;
        currentBrightY = sharedBrightY;
        // 注意：导航逻辑可能需要知道数据是否是"新"的，
        // 但为了简化，这里直接读取最新值。
        // 如果需要区分新旧，可以在获取锁后检查 newBrightPointAvailable
        xSemaphoreGive(frameAccessMutex);
      } else {
         // Serial.println("Loop: 未能获取锁读取最亮点"); // 调试信息
      }

      // c. 更新导航状态机 (增加导航更新间隔，例如300ms)
      if (currentTime - lastNavigationUpdateTime >= 300) {
          lastNavigationUpdateTime = currentTime;
          // 将当前帧和获取到的最亮点信息传递给导航状态机
          navigationStateMachine(fb, currentBrightX, currentBrightY); 
      }
      
      // d. 处理完毕，返回帧缓冲区
      esp_camera_fb_return(fb);
    } else {
      // Serial.println("未能获取摄像头帧"); // 调试信息
    }
  } else if (systemIsBusy) {
      // Serial.println("系统繁忙，跳过帧处理"); // 调试信息
      // 当系统繁忙时，可以考虑降低电机速度或执行安全操作
      // motor_control(0, 0);
      // motor_control(1, 0);
  }

  // 3. 短暂延时，让其他任务有机会运行
  // vTaskDelay(pdMS_TO_TICKS(10)); // 使用FreeRTOS延时更佳
  delay(10); // 简单延时
}
