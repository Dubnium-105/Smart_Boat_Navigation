#include <Arduino.h>
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
int speedA = 0;
int speedB = 0;
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      船舶控制系统启动中...     ");
  Serial.println("====================================");
  setup_motors();
  //连接WiFi
  if (!connectWiFi()) {
    Serial.println("WiFi连接失败，重启...");
    delay(1000);
    ESP.restart();
  }
  //初始化MQTT客户端
  setupMQTT();
  // 首次尝试连接MQTT (如果WiFi连接成功)
  if (WiFi.status() == WL_CONNECTED) {
      mqtt_reconnect(); 
  }
  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  Serial.println("====================================");

}
void loop() {
  wifiAutoReconnect();
  loopMQTT();
  motor_control(0, speedA);
  motor_control(1, speedB);
    delay(2);
}
