#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

// WiFi连接函数
bool connectWiFi();
bool wifiAutoReconnect();
void printWiFiStatus();
bool isWiFiConnected();
void getWiFiInfo(String& ssidStr, String& ipStr, int& rssi);

#endif
