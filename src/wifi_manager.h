#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

// 固定IP设置
#define USE_FIXED_IP false
extern IPAddress staticIP;
extern IPAddress gateway;
extern IPAddress subnet;
extern IPAddress dns;

/**
 * @brief 尝试连接到预定义的WiFi网络列表中的一个。
 * 
 * @return true 如果成功连接到WiFi。
 * @return false 如果所有尝试都失败。
 */
bool connectWiFi();

/**
 * @brief 检查WiFi状态并在断开时自动重连。
 * 
 * @return true 如果WiFi处于连接状态或重连成功。
 * @return false 如果重连失败。
 */
bool wifiAutoReconnect();

#endif // WIFI_MANAGER_H