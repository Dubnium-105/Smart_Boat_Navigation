#include "wifi_manager.h"

// 多WiFi配置 - 支持多个WiFi AP自动切换
struct WiFiCredential {
    const char* ssid;
    const char* password;
};

// 在此添加多个WiFi账号
WiFiCredential wifiList[] = {
    {"Netcore-231AF8", "1145141919"},
    {"xbox", "12345678"},
    // {"WIFI3_SSID", "WIFI3_PASSWORD"},
};

const int wifiCount = sizeof(wifiList) / sizeof(wifiList[0]);
int currentWiFiIndex = 0;

// WiFi重连状态变量
static unsigned long lastReconnectAttempt = 0;
static const unsigned long reconnectInterval = 5000; // 5秒重试间隔
static bool isReconnecting = false;

bool connectWiFi() {
    Serial.println("开始连接WiFi...");
    Serial.printf("尝试连接 SSID: %s\n", wifiList[currentWiFiIndex].ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiList[currentWiFiIndex].ssid, wifiList[currentWiFiIndex].password);
    
    int attempts = 0;
    const int maxAttempts = 20; // 最多尝试20次，每次500ms，总共10秒
    
    Serial.print("连接中");
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("✓ WiFi连接成功!");
        printWiFiStatus();
        isReconnecting = false;
        return true;
    } else {
        Serial.println();
        Serial.println("✗ WiFi连接失败!");
        Serial.printf("状态码: %d\n", WiFi.status());
        
        // 尝试下一个WiFi
        currentWiFiIndex = (currentWiFiIndex + 1) % wifiCount;
        Serial.printf("切换到下一个WiFi: %s\n", wifiList[currentWiFiIndex].ssid);
        
        return false;
    }
}

bool wifiAutoReconnect() {
    unsigned long currentTime = millis();
    
    // 检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED) {
        // 如果不在重连状态，或者已经到了重连时间
        if (!isReconnecting || (currentTime - lastReconnectAttempt >= reconnectInterval)) {
            if (!isReconnecting) {
                Serial.println("检测到WiFi断开连接，启动自动重连...");
                isReconnecting = true;
            }
            
            lastReconnectAttempt = currentTime;
            Serial.printf("WiFi重连尝试... (间隔: %lu秒)\n", reconnectInterval / 1000);
            
            WiFi.disconnect();
            delay(100);
            
            if (connectWiFi()) {
                Serial.println("✓ WiFi自动重连成功!");
                isReconnecting = false;
                return true;
            } else {
                Serial.println("✗ WiFi重连失败，将继续重试...");
                return false;
            }
        }
        return false; // 正在重连过程中
    } else {
        // WiFi已连接
        if (isReconnecting) {
            Serial.println("✓ WiFi重连过程完成，连接稳定");
            isReconnecting = false;
        }
        return true;
    }
}

void printWiFiStatus() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("=== WiFi状态信息 ===");
        Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
        Serial.printf("IP地址: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("网关: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("子网掩码: %s\n", WiFi.subnetMask().toString().c_str());
        Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("信号强度: %d dBm\n", WiFi.RSSI());
        Serial.printf("MAC地址: %s\n", WiFi.macAddress().c_str());
        Serial.println("==================");
    } else {
        Serial.printf("WiFi未连接 (状态: %d)\n", WiFi.status());
    }
}

bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void getWiFiInfo(String& ssidStr, String& ipStr, int& rssi) {
    if (WiFi.status() == WL_CONNECTED) {
        ssidStr = WiFi.SSID();
        ipStr = WiFi.localIP().toString();
        rssi = WiFi.RSSI();
    } else {
        ssidStr = "未连接";
        ipStr = "0.0.0.0";
        rssi = -100;
    }
}
