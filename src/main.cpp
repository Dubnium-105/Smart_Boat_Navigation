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
#include "network_service.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "IR-control.h"

#ifndef NETWORK_4G_MODULE_TEST
#define NETWORK_4G_MODULE_TEST 0
#endif

extern void startSimpleCameraStream(); // 来自 stream.cpp
extern void streamLoop(); // 来自 stream.cpp

// --- 补充IR与电机自动控制相关声明 ---
extern int getMainIRDirection(const int irVals[8]);
extern void motor_control_ir_auto(int mainDirIdx);

// IR传感器引脚定义（请根据实际硬件修改）
extern const int IR_PINS[8]; // 删除本地定义，使用头文件声明

// 获取摄像头配置的函数
sensor_t* sensor_get_config() {
    return esp_camera_sensor_get();
}

// --- 全局共享变量和同步原语 ---
SemaphoreHandle_t frameAccessMutex = nullptr;
bool cameraAvailable = false; // 全局摄像头可用标志，用于指示摄像头是否成功初始化。
TaskHandle_t cameraTaskHandle = NULL;
void cameraTask(void *pvParameters);

// === 手动模式下电机速度全局变量 ===
int speedA = 0;
int speedB = 0;

// --- Arduino Setup ---
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      船舶控制系统启动中...     ");
  Serial.println("====================================");

  // 1. 创建互斥锁 (必须在任务使用前创建)
  frameAccessMutex = xSemaphoreCreateMutex();
  if (frameAccessMutex == NULL) {
    Serial.println("错误: 无法创建帧访问互斥锁! 重启...");
    delay(1000);
    ESP.restart();
  }
  
  // 2. 初始化摄像头 (增加重试机制)
  #if NETWORK_4G_MODULE_TEST
  Serial.println("[4G-TEST] skip camera init to avoid modem UART pin conflicts");
  cameraAvailable = false;
  #else
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
  // 3. 初始化红外接收器
  Serial.println("初始化红外接收器...");
  #endif
  setupIR();

  // 4. 初始化电机
  setup_motors();

  // 5. 初始化并连接网络服务: WiFi优先，4G兜底
  setupNetworkService();
  if (!connectNetworkService()) {
#if NETWORK_4G_MODULE_TEST
    Serial.println("[4G-TEST] 首次连接失败，将在循环中持续重试，不重启。");
#else
    Serial.println("网络服务连接失败（WiFi+4G），重启...");
    delay(1000);
    ESP.restart();
#endif
  }

  NetworkServiceStatus netStatus = getNetworkServiceStatus();
  Serial.printf("网络服务就绪，当前链路=%s\n", activeNetworkName(netStatus.activeNetwork));

  if (netStatus.activeNetwork == ActiveNetworkType::Cellular4G) {
    Serial.println("当前运行在4G兜底链路");
  }

  // 6. 初始化MQTT客户端
#if NETWORK_4G_MODULE_TEST
  Serial.println("[4G-TEST] 已跳过MQTT初始化，进入4G模块单测模式");
#else
  setupMQTT();
  // 首次尝试连接MQTT（当前仅WiFi承载）
  if (isMqttPathReady()) {
      mqtt_reconnect(); 
  } else {
      Serial.println("当前为4G兜底网络，MQTT等待WiFi恢复后再连接");
  }
#endif
  // 7. 启动视频流服务器
#if NETWORK_4G_MODULE_TEST
  Serial.println("[4G-TEST] 已跳过视频流启动");
#else
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");
#endif

  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
    if (netStatus.wifiConnected) {
      Serial.print("访问: http://");
      Serial.print(WiFi.localIP());
      Serial.println(" 查看视频流");
  }
  Serial.println("====================================");

  // 启动摄像头推流任务（绑定core0）
  #if !NETWORK_4G_MODULE_TEST
  xTaskCreatePinnedToCore(
    cameraTask,         // 任务函数
    "CameraTask",      // 名称
    8192,              // 堆栈大小
    NULL,              // 参数
    2,                 // 优先级
    &cameraTaskHandle, // 任务句柄
    0                  // 绑定core0
  );
  #endif
}

void cameraTask(void *pvParameters) {
  while (1) {
#if !NETWORK_4G_MODULE_TEST
    streamLoop(); // 摄像头推流主循环
#endif
    vTaskDelay(1); // 防止死循环卡死
  }
}

// --- Arduino Loop ---
void loop() {
    networkServiceLoop();
#if !NETWORK_4G_MODULE_TEST
    static unsigned long lastMqttReconnectAttempt = 0;
    unsigned long currentTime = millis();
    if (!isMqttPathReady()) {
      // 仅在WiFi就绪时重连MQTT，4G作为链路兜底不进入当前MQTT通道。
    } else if (!mqttClient.connected()) {
        if (currentTime - lastMqttReconnectAttempt > 5000) {
            lastMqttReconnectAttempt = currentTime;
            mqtt_reconnect();
        }
    } else {
        mqttClient.loop();
    }
  #endif
    if (irNavState == STATE_NAVIGATING) {
        int irVals[8];
        for (int i = 0; i < 8; ++i) irVals[i] = digitalRead(IR_PINS[i]);
        int mainDir = getMainIRDirection(irVals);
        if (mainDir >= 0) {
            motor_control_ir_auto(mainDir);
        }
        handleIRSignal();
    } else if (irNavState == STATE_MISSION) {
        motor_control(0, 0);
        motor_control(1, 0);
    } else {
        // 手动模式下持续驱动电机
        motor_control(0, speedA);
        motor_control(1, speedB);
        handleIRSignal();
    }
    delay(2); // 保持高帧率
}
