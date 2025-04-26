#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

// 固定IP设置
#define USE_FIXED_IP true
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

#endif // WIFI_MANAGER_H