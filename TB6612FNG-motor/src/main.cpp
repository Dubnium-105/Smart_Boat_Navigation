/**
 * ESP32-S3 TB6612FNG电机控制系统 - 主程序
 * 
 * 功能:
 * - 初始化WiFi、MQTT、TB6612FNG电机驱动
 * - 支持串口键盘控制和MQTT远程控制
 * - 系统状态监控和报告
 */
#include <Arduino.h>
#include "MotorControl.h"
#include "mqtt_manager.h"
#include "wifi_manager.h"

// --- 模块功能开关 ---
bool ENABLE_MOTOR = true;       // 是否启用电机控制
bool ENABLE_WIFI = true;        // 是否启用WiFi
bool ENABLE_MQTT = true;        // 是否启用MQTT

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println("\n\n====================================");
    Serial.println("    ESP32-TB6612电机控制系统启动    ");
    Serial.println("====================================");

    // 1. 初始化电机控制系统
    if (ENABLE_MOTOR) {
        MotorControl::initialize();
        Serial.println("✓ TB6612FNG电机驱动初始化完成");
    }

    // 2. 连接WiFi
    if (ENABLE_WIFI) {
        if (!connectWiFi()) {
            Serial.println("WiFi连接失败，启用离线模式");
            ENABLE_MQTT = false; // 关闭MQTT功能
        } else {
            Serial.println("✓ WiFi连接成功");
        }
    }

    // 3. 初始化MQTT客户端
    if (ENABLE_MQTT && WiFi.status() == WL_CONNECTED) {
        setupMQTT();
        Serial.println("✓ MQTT客户端初始化完成");
        
        // 首次尝试连接MQTT
        if (mqtt_reconnect()) {
            Serial.println("✓ MQTT服务器连接成功");
        } else {
            Serial.println("⚠ MQTT服务器连接失败，将在后台重试");
        }
    }

    Serial.println("====================================");
    Serial.println("        系统初始化完成!           ");
    Serial.println("====================================");
    Serial.println("控制说明:");
    Serial.println("- 串口控制: W/S控制电机A, E/D控制电机B");
    Serial.println("- MQTT控制: 发送JSON到/motor主题");
    Serial.println("  格式: {\"speedA\": -50, \"speedB\": 80}");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("- 设备IP: ");
        Serial.println(WiFi.localIP());
    }
    Serial.println("====================================");
}

// --- Arduino Loop ---
void loop() {
    unsigned long currentTime = millis();
    
    // 1. WiFi连接检查和自动重连
    static unsigned long lastWiFiCheck = 0;
    if (ENABLE_WIFI && currentTime - lastWiFiCheck > 10000) { // 每10秒检查一次WiFi状态
        lastWiFiCheck = currentTime;
        
        if (!wifiAutoReconnect()) {
            // WiFi断开，暂时禁用MQTT
            if (ENABLE_MQTT) {
                ENABLE_MQTT = false;
            }
        } else {
            // WiFi已连接，重新启用MQTT（如果之前被禁用）
            if (!ENABLE_MQTT && WiFi.status() == WL_CONNECTED) {
                ENABLE_MQTT = true;
                setupMQTT();
            }
        }
    }

    // 2. 处理本地键盘控制（始终可用）
    if (ENABLE_MOTOR) {
        MotorControl::handleKeyboardInput();
    }

    // 3. 处理MQTT连接和消息
    static unsigned long lastMqttReconnectAttempt = 0;
    
    if (ENABLE_MQTT && WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected()) {
            if (currentTime - lastMqttReconnectAttempt > 5000) {
                lastMqttReconnectAttempt = currentTime;
                Serial.println("MQTT断开连接，尝试重连...");
                mqtt_reconnect();
            }
        } else {
            loopMQTT();
        }
    }

    delay(2); // 保持高响应性
}
