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
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- 包含需要的模块的头文件 --- 
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"

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

  // 1. 初始化电机
  setup_motors();

  // 5. 连接WiFi
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
  // Note: 视频流功能已移除，保留 MQTT 与电机控制

  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  if (WiFi.status() == WL_CONNECTED) {
      Serial.print("访问: http://");
      Serial.print(WiFi.localIP());
      Serial.println(" 查看视频流");
  }
  Serial.println("====================================");

  // 不再创建摄像头推流任务
}


// --- Arduino Loop ---
void loop() {
    wifiAutoReconnect();
    static unsigned long lastMqttReconnectAttempt = 0;
    unsigned long currentTime = millis();
    if (!mqttClient.connected()) {
        if (currentTime - lastMqttReconnectAttempt > 5000) {
            lastMqttReconnectAttempt = currentTime;
            mqtt_reconnect();
        }
    } else {
        mqttClient.loop();
    }
  // 一律使用 MQTT 控制的手动速度驱动电机
  motor_control(0, speedA);
  motor_control(1, speedB);
    delay(2); // 保持高帧率
}
