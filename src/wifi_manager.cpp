#include "wifi_manager.h"
#include <Arduino.h>

// 固定IP地址定义
IPAddress staticIP(192, 168, 31, 170);  // 固定IP地址
IPAddress gateway(192, 168, 31, 1);     // 网关
IPAddress subnet(255, 255, 255, 0);     // 子网掩码
IPAddress dns(8, 8, 8, 8);              // DNS服务器

// WiFi连接配置
struct WifiCredential {
  const char* ssid;
  const char* password;
};

const WifiCredential wifiCredentials[] = {
  {"room@407", "room@407"},
  {"xbox", "12345678"},
  {"xbox", "z1139827642"}
  // 可以添加更多WiFi凭据
};
const int numWifiOptions = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

bool connectWiFi() {
  Serial.println("开始连接WiFi...");
  bool connected = false;
  for (int i = 0; i < numWifiOptions; i++) {
    Serial.printf("尝试连接WiFi: %s\n", wifiCredentials[i].ssid);
    
    WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);
    WiFi.setSleep(false);  // 禁用WiFi睡眠模式以提高响应性
    
    // 尝试连接当前WiFi，最多等待约3秒 (12 * 250ms)
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 12) {
      delay(250);
      Serial.print(".");
      attempts++;
    } 
    
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.println("");
      Serial.printf("连接成功: %s\n", wifiCredentials[i].ssid);
      
      // 设置固定IP
      if (USE_FIXED_IP) {
        if (!WiFi.config(staticIP, gateway, subnet, dns)) {
          Serial.println("固定IP配置失败");
        } else {
          Serial.printf("IP地址: %s\n", WiFi.localIP().toString().c_str());
        }
      } else {
         Serial.printf("IP地址: %s\n", WiFi.localIP().toString().c_str());
      }
      break; // 连接成功，跳出循环
    } else {
      Serial.println("");
      Serial.printf("连接失败: %s\n", wifiCredentials[i].ssid);
      WiFi.disconnect(); // 断开当前失败的连接尝试
      delay(500); // 短暂等待后尝试下一个
    }
  }
  
  if (!connected) {
    Serial.println("无法连接到任何WiFi网络。");
  }
  return connected;
}

bool wifiAutoReconnect() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    Serial.println("WiFi断开，尝试自动重连...");
    if (!connectWiFi()) {
        Serial.println("WiFi重连失败，等待下次循环...");
        delay(1000); // 可适当延长等待，避免频繁重试
        return false;
    } else {
        Serial.println("WiFi重连成功");
        return true;
    }
}