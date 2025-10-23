#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include <IPAddress.h>
// =================================================================
//                  用户可配置区域
// =================================================================

// ------------------- 船舶基础信息 -------------------
// 在这里修改你的船舶名称，它将作为客户端ID的一部分
const String SHIP_NAME = "MySmartBoat";
// ------------------- Wi-Fi 连接配置 -------------------
// 在这里添加你的Wi-Fi网络凭据，可以添加多个
const struct WifiCredential {
  const char* ssid;
  const char* password;
} wifiCredentials[] = {
  {"WIFI_NAME_1", "PASSWORD_1"},
  {"WIFI_NAME_2", "PASSWORD_2"},
  // 可以继续添加更多...
};
// 是否启用静态IP地址 (true/false)
const bool USE_FIXED_IP = false;
// 如果启用静态IP，请在此处配置
const IPAddress STATIC_IP(192, 168, 1, 188);
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);
const IPAddress DNS(114, 114, 114, 114);
// ------------------- MQTT 服务器配置 -------------------
// 你的MQTT服务器地址 (域名或IP)
const char* MQTT_SERVER = "your_mqtt_broker.com";
// MQTT服务器端口
const int MQTT_PORT = 1883;
// MQTT数据包缓冲区大小 (字节)
const int MAX_MQTT_PACKET_SIZE = 1024;
#endif // CONFIG_H
