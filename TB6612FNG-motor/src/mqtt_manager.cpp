#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_random.h>
#include "MotorControl.h"

// 函数前置声明
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publishSystemStatus();

// MQTT配置  
const char* mqttServer = "8.210.4.26";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 系统状态变量
float currentFPS = 0.0;

void setupMQTT() {
    randomSeed(esp_random());
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqtt_callback);
    mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
    
    Serial.println("MQTT客户端已配置");
    Serial.printf("MQTT服务器: %s:%d\n", mqttServer, mqttPort);
    Serial.printf("缓冲区大小: %d 字节\n", MAX_MQTT_PACKET_SIZE);
}

bool mqtt_reconnect() {
    // 首先检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("MQTT连接失败: WiFi未连接");
        return false;
    }
    
    Serial.println("=== MQTT连接诊断开始 ===");
    Serial.printf("WiFi状态: 已连接 (%s)\n", WiFi.localIP().toString().c_str());
    Serial.printf("信号强度: %d dBm\n", WiFi.RSSI());
    Serial.printf("尝试连接: %s:%d\n", mqttServer, mqttPort);
    
    // 简单的网络连通性测试
    WiFiClient testClient;
    Serial.print("测试网络连通性...");
    if (testClient.connect(mqttServer, mqttPort)) {
        Serial.println(" ✓ 成功");
        testClient.stop();
    } else {
        Serial.println(" ✗ 失败");
        Serial.println("错误: 无法连接到MQTT服务器");
        Serial.println("请检查:");
        Serial.println("1. 服务器地址是否正确: " + String(mqttServer));
        Serial.println("2. 端口号是否正确: " + String(mqttPort));
        Serial.println("3. 服务器是否在线");
        Serial.println("4. 防火墙是否阻止连接");
        return false;
    }
    
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 5) {
        attempts++;
        Serial.printf("第%d次MQTT连接尝试...", attempts);
        
        // 创建随机客户端ID
        String clientId = "ESP32-TB6612-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println(" ✓ 成功!");
            
            // 订阅主题
            bool sub1 = mqttClient.subscribe("/motor");
            bool sub2 = mqttClient.subscribe("/restart");
            bool sub3 = mqttClient.subscribe("/check_mqtt");
            
            Serial.printf("订阅结果: motor=%s, restart=%s, check=%s\n", 
                         sub1?"OK":"FAIL", sub2?"OK":"FAIL", sub3?"OK":"FAIL");
            
            // 发布上线消息
            mqttClient.publish("/ESP32_info", "{\"msg\":\"ESP32-TB6612已上线\",\"mode\":\"manual_control\"}");
            mqttClient.publish("/motor/status", "电机控制已就绪");
            
            publishSystemStatus();
            Serial.println("=== MQTT连接成功 ===");
            return true;
        } else {
            int errorCode = mqttClient.state();
            Serial.printf(" ✗ 失败 (错误码: %d)\n", errorCode);
            
            switch(errorCode) {
                case -4: Serial.println("  原因: 连接超时"); break;
                case -3: Serial.println("  原因: 连接丢失"); break;
                case -2: Serial.println("  原因: 连接失败 - 检查服务器地址"); break;
                case -1: Serial.println("  原因: 客户端断开"); break;
                case 1: Serial.println("  原因: 协议版本不支持"); break;
                case 2: Serial.println("  原因: 客户端ID被拒绝"); break;
                case 3: Serial.println("  原因: 服务器不可用"); break;
                case 4: Serial.println("  原因: 用户名/密码错误"); break;
                case 5: Serial.println("  原因: 未授权"); break;
                default: Serial.printf("  原因: 未知错误 %d\n", errorCode); break;
            }
            
            delay(1000);
        }
    }
    
    Serial.println("=== MQTT连接失败 ===");
    Serial.println("建议排查步骤:");
    Serial.println("1. ping " + String(mqttServer));
    Serial.println("2. telnet " + String(mqttServer) + " " + String(mqttPort));
    Serial.println("3. 尝试使用公共MQTT服务器测试");
    Serial.println("   如: broker.hivemq.com:1883");
    return false;
}

void publishSystemStatus() {
    if (!mqttClient.connected()) return;
    
    String status = "{\"fps\":" + String(currentFPS) + 
                   ",\"ip\":\"" + WiFi.localIP().toString() + 
                   "\",\"driver\":\"TB6612FNG\"}";
    mqttClient.publish("/esp32/status", status.c_str());
}

void loopMQTT() {
    if (!mqttClient.connected()) {
        // 连接断开处理
    } else {
        mqttClient.loop();
        
        // 定期发布系统状态
        static unsigned long lastStatusTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastStatusTime > 5000) {
            lastStatusTime = currentTime;
            publishSystemStatus();
        }
    }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
    
    // 转换为字符串
    char message[256];
    if (length < sizeof(message) - 1) {
        memcpy(message, payload, length);
        message[length] = '\0';
        Serial.printf("消息内容: %s\n", message);
    } else {
        Serial.println("消息过长");
        return;
    }

    // 处理电机控制消息
    if (strcmp(topic, "/motor") == 0) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            return;
        }
        
        // 检查电机速度参数
        if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
            int speedA = doc["speedA"].as<int>();
            int speedB = doc["speedB"].as<int>();
            speedA = constrain(speedA, -100, 100);
            speedB = constrain(speedB, -100, 100);
            
            Serial.printf("通过MQTT设置电机速度: A=%d, B=%d\n", speedA, speedB);
            
            // 映射到-255到255
            int motorASpeed = map(speedA, -100, 100, -255, 255);
            int motorBSpeed = map(speedB, -100, 100, -255, 255);
            
            // 控制电机
            MotorControl::setMotorASpeed(motorASpeed);
            MotorControl::setMotorBSpeed(motorBSpeed);
            
            // 发送状态确认
            String response = "{\"speedA\":" + String(speedA) + 
                            ",\"speedB\":" + String(speedB) + 
                            ",\"deadzone\":20,\"status\":\"success\"}";
            mqttClient.publish("/motor/status", response.c_str());
        }
    }
    // 处理连接检查
    else if (strcmp(topic, "/check_mqtt") == 0) {
        mqttClient.publish("/check_mqtt_reply", "pong");
        Serial.println("收到ping，已回复pong");
    }
    // 处理重启命令
    else if (strcmp(topic, "/restart") == 0) {
        Serial.println("收到重启命令，准备重启...");
        mqttClient.publish("/ESP32_info", "即将重启ESP32-TB6612");
        delay(1000);
        ESP.restart();
    }
}
