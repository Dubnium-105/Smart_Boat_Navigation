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
#include "vision_processor.h" // 添加新的视觉处理器头文件
#include "ir_vision_processor.h" // 添加IR视觉处理器头文件

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
volatile int sharedBrightX = -1; // 初始化为无效值
volatile int sharedBrightY = -1;
volatile bool newBrightPointAvailable = false;
volatile bool systemIsBusy = false;      // 系统繁忙标志 (由systemMonitorTask更新)

// --- 帧处理函数 ---
// 在每个帧处理时手动调用的函数 (可以考虑移入camera_processing模块)
void processFrame(camera_fb_t *fb) {
  // 使用新的视觉处理器处理帧
  if (g_visionProcessor != nullptr) {
    // 调用视觉处理器的处理函数
    g_visionProcessor->processFrame(fb);
    return;
  }
  
  // 如果视觉处理器未初始化，则使用旧的处理逻辑
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
  Serial.println("      晶能船导航系统启动中...     ");
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
  
  // 7. 初始化视觉处理器 (新增)
  sensor_t* sensor = sensor_get_config();
  if (sensor == nullptr || sensor->status.framesize < 0) {
    Serial.println("警告: 摄像头配置不可用，无法初始化视觉处理器");
  } else {
    // 获取摄像头分辨率
    int framesize = sensor->status.framesize;
    if (framesize >= 0 && framesize < sizeof(camera_resolution)/sizeof(camera_resolution[0])) {
      int width = camera_resolution[framesize].width;
      int height = camera_resolution[framesize].height;
      
      // 初始化视觉处理器
      if (initVisionProcessor(width, height)) {
        Serial.println("视觉处理器初始化成功");
      } else {
        Serial.println("视觉处理器初始化失败，将使用基本处理");
      }
      
      // 初始化IR视觉处理器
      if (initIRVisionProcessor(width, height)) {
        Serial.println("IR视觉处理器初始化成功");
      } else {
        Serial.println("IR视觉处理器初始化失败");
      }
    } else {
      Serial.printf("警告: 无效的framesize: %d\n", framesize);
    }
  }

  // 8. 初始化性能监控变量 (移到stream.cpp或全局?)
  lastFPSCalculationTime = esp_timer_get_time();
  frameCount = 0;

  // 9. 启动视频流服务器 (假设在核心1运行)
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");

  // 10. 创建后台任务 (固定到核心0以减少对摄像头/流的影响)
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
