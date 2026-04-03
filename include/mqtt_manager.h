#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>

enum NAV_STATE {
  STATE_MANUAL = 0,
  STATE_NAVIGATING = 1,
  STATE_MISSION = 2
};

#include <PubSubClient.h>
#include <WiFi.h>

// 协议版本：v2 为当前工程化版本，v1 为兼容层最低版本。
constexpr uint8_t PROTOCOL_ACTIVE_VERSION = 2;
constexpr uint8_t PROTOCOL_COMPAT_MIN_VERSION = 1;
constexpr uint8_t PROTOCOL_SCHEMA_VERSION = 2;

extern WiFiClient espClient;
extern PubSubClient mqttClient;

extern volatile int irNavState;
extern volatile int webRequestedNavState;
String get_ir_nav_mode_str();
String get_device_id();
void setNavMode(int mode);
void setupMQTT();
bool mqtt_reconnect();
void loopMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publishIRInfo(int sensorId, const char* direction, int angle); // 发送红外方向信息
void pumpProtocolRuntime(); // 驱动ACK去重缓存清理、遥测批处理上传与HTTP重试

#endif // MQTT_MANAGER_H
