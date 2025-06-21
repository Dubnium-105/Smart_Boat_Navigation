#include "mqtt_manager.h"
#include "motor_control.h" // 需要调用电机控制函数
#include <ArduinoJson.h>   // JSON处理库
#include <esp_random.h>    // 用于生成随机客户端ID
#include <HTTPUpdate.h>

// 添加导航状态相关占位符变量
enum NAV_STATE {
  STATE_MANUAL = 0,     // 手动控制模式
  STATE_NAVIGATING = 1  // 自动导航模式
};

// 全局导航状态变量
int irNavState = STATE_MANUAL;
unsigned long irStateStartTime = 0;

// 占位函数，之前由navigation.h提供，现在直接内联在此处
String get_ir_nav_mode_str() {
  return irNavState == STATE_MANUAL ? "手动控制" : "红外导航";
}

// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240; // 增加缓冲区以防大数据包

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 声明来自stream.cpp的函数
extern void configureStreamServer(bool enable, const char* serverUrl);

// 全局保存本机clientId
String g_clientId = "";

void setupMQTT() {
  randomSeed(esp_random()); // 使用ESP32硬件随机数生成器
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE); // 设置缓冲区大小
  Serial.println("MQTT客户端已配置");
}

// 函数声明


bool mqtt_reconnect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");
    // 创建一个随机的客户端ID
    String clientId = "ESP32-CAM-";
    clientId += "黑色大船"; // TODO: 可根据实际硬件唯一标识生成
    g_clientId = clientId; // 保存全局
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");
      
      // 订阅所有需要的主题
      mqttClient.subscribe("/motor"); // 订阅电机控制主题
      mqttClient.subscribe("/stream/config"); // 订阅视频流配置主题
      mqttClient.subscribe("/restart");
      mqttClient.subscribe("/ota"); // 订阅OTA升级主题
      mqttClient.subscribe("/check_mqtt"); 
      mqttClient.subscribe("/motor_control"); // 订阅电机开关控制主题
      mqttClient.subscribe("/navigation"); // 订阅控制模式切换主题      // 构造JSON格式上线信息，包含WiFi和系统状态
      DynamicJsonDocument doc(512);
      doc["msg"] = "ESP32-CAM已上线";
      doc["ip"] = WiFi.localIP().toString();
      doc["wifi_ssid"] = WiFi.SSID();
      doc["wifi_rssi"] = WiFi.RSSI();
      doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
      doc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
      doc["mode"] = get_ir_nav_mode_str();
      doc["clientId"] = g_clientId;
      String infoStr;
      serializeJson(doc, infoStr);
      mqttClient.publish("/ESP32_info", infoStr.c_str());
      mqttClient.publish("/motor/status", "电机控制已就绪");
      

      
      return true;
    } else {
      Serial.print("MQTT连接失败, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" 1秒后重试");
      delay(1000); // 等待1秒再重试
    }
  }
  if (!mqttClient.connected()) {
      Serial.println("MQTT连接失败次数过多。");
  }
  return mqttClient.connected();
}



void loopMQTT() {
  if (!mqttClient.connected()) {
    // 如果未连接，则尝试重连（可以增加重连间隔逻辑）
    // mqtt_reconnect(); // 在主循环中更精细地控制重连时机
  } else {
    // 处理MQTT消息
    mqttClient.loop();
    
  
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
  
  // 将接收到的数据转换为字符串以便调试 (确保缓冲区足够大)
  char message[256]; // 增加缓冲区大小
  if (length < sizeof(message) - 1) {
      memcpy(message, payload, length);
      message[length] = '\0'; // 添加字符串结束符
      Serial.printf("消息内容: %s\n", message);
  } else {
      Serial.println("消息过长，无法完整显示。");
      // 可以选择只处理部分消息或忽略
  }

  // 处理电机控制消息
  if (strcmp(topic, "/motor") == 0) {
    // 使用ArduinoJson解析JSON数据
    DynamicJsonDocument doc(256); // 使用DynamicJsonDocument替代JsonDocument
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    // 新增：clientId过滤
    if (doc.containsKey("clientId") && doc["clientId"].is<const char*>()) {
      String targetId = doc["clientId"].as<const char*>();
      if (targetId != g_clientId) {
        Serial.printf("忽略非本机指令，目标clientId=%s，本机clientId=%s\n", targetId.c_str(), g_clientId.c_str());
        return;
      }
    } else {
      Serial.println("未指定clientId，忽略指令");
      return;
    }
    // 检查是否包含电机速度参数
    if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
       // 获取电机PWM值（直接使用，不映射）
       int speedA = doc["speedA"].as<int>();
       int speedB = doc["speedB"].as<int>();
       speedA = constrain(speedA, -255, 255); // 限制PWM范围到-255~+255
       speedB = constrain(speedB, -255, 255); // 限制PWM范围到-255~+255
       
       Serial.printf("通过MQTT设置电机PWM: A=%d, B=%d\n", speedA, speedB);
       motor_control(0, speedA); // 控制电机A，直接传入PWM值
       motor_control(1, speedB); // 控制电机B，直接传入PWM值
       
       // 发送电机状态确认
       DynamicJsonDocument resDoc(128);
       resDoc["speedA"] = speedA;
       resDoc["speedB"] = speedB;
       resDoc["pwm_direct"] = true; // 标识使用直接PWM控制
       resDoc["timestamp"] = millis();
       
       String resStr;
       serializeJson(resDoc, resStr);
       mqttClient.publish("/motor/status", resStr.c_str());
    } else {
       Serial.println("JSON中缺少speedA或speedB字段或类型错误，应为整数。");
    }
  }
// 处理导航状态切换消息
  else if (strcmp(topic, "/navigation") == 0) {
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error && doc["command"].is<const char*>()) {
      String cmd = doc["command"].as<const char*>();
      if (cmd == "manual") {
        irNavState = STATE_MANUAL;
        irStateStartTime = millis();
        Serial.println("导航状态: 切换到手动控制模式");
      } else if (cmd == "navigate") {
        irNavState = STATE_NAVIGATING;
        irStateStartTime = millis();
        Serial.println("导航状态: 切换到自动导航模式");
      } else {
        Serial.printf("收到未知导航命令: %s，未做处理\n", cmd.c_str());
        // 可选：回复错误信息到MQTT或采取其他措施
      }
    }
  }
  // 处理MQTT连接检查
  else if (strcmp(topic, "/check_mqtt") == 0) {
    mqttClient.publish("/check_mqtt_reply", "pong");
    Serial.println("收到/check_mqtt，已回复/pong");
    JsonDocument infoDoc;
    infoDoc["msg"] = "MQTT连接检查响应";
    infoDoc["ip"] = WiFi.localIP().toString();
    infoDoc["wifi_ssid"] = WiFi.SSID();
    infoDoc["wifi_rssi"] = WiFi.RSSI();
    infoDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    infoDoc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
    infoDoc["mode"] = get_ir_nav_mode_str();
    infoDoc["clientId"] = g_clientId;
    String infoStr;
    serializeJson(infoDoc, infoStr);
    mqttClient.publish("/ESP32_info", infoStr.c_str());
    Serial.println("已发送ESP32系统信息到/ESP32_info");
  }
  // 处理OTA升级命令
  else if (strcmp(topic, "/ota") == 0) {
    String otaUrl;
    if (length < 200) {
      otaUrl = String((const char*)payload, length);
      otaUrl.trim();
      Serial.printf("收到OTA升级命令，URL: %s\n", otaUrl.c_str());
      mqttClient.publish("/OTA_info", (String("开始OTA升级，URL:") + otaUrl).c_str());
      // 真正的OTA升级流程
      WiFiClient otaClient;
      t_httpUpdate_return ret = httpUpdate.update(otaClient, otaUrl);
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          mqttClient.publish("/OTA_info", (String("OTA失败: ") + httpUpdate.getLastErrorString()).c_str());
          Serial.printf("OTA失败: %s\n", httpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          mqttClient.publish("/OTA_info", "OTA无可用更新");
          Serial.println("OTA无可用更新");
          break;
        case HTTP_UPDATE_OK:
          mqttClient.publish("/OTA_info", "OTA升级成功，正在重启...");
          Serial.println("OTA升级成功，正在重启...");
          delay(1000);
          ESP.restart();
          break;
      }
    } else {
      mqttClient.publish("/OTA_info", "OTA URL过长，忽略");
    }
  }
  // 处理重启命令
  else if (strcmp(topic, "/restart") == 0) {
    Serial.println("收到重启命令，准备重启...");
    mqttClient.publish("/OTA_info", "即将重启ESP32");
    delay(1000);
    ESP.restart();
  }
  // 处理电机控制开关消息
  if (strcmp(topic, "/motor_control") == 0) {
    DynamicJsonDocument doc(128);
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error && doc["key"].is<const char*>()) {
      String key = doc["key"].as<const char*>();
      if (key == "shutdown") {
        // 关闭电机，速度归零
        motor_control(0, 0);
        motor_control(1, 0);
        Serial.println("收到电机关闭命令，已自动归零速度");
        // 发送状态确认
        DynamicJsonDocument resDoc(128);
        resDoc["speedA"] = 0;
        resDoc["speedB"] = 0;
        resDoc["pwm_direct"] = true;
        resDoc["timestamp"] = millis();
        String resStr;
        serializeJson(resDoc, resStr);
        mqttClient.publish("/motor/status", resStr.c_str());
      }
    }
    return;
  }
}