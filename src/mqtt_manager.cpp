#include "mqtt_manager.h"
#include "motor_control.h" // 需要调用电机控制函数
#include <ArduinoJson.h>   // JSON处理库
#include <esp_random.h>    // 用于生成随机客户端ID

// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240; // 增加缓冲区以防大数据包

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setupMQTT() {
  randomSeed(esp_random()); // 使用ESP32硬件随机数生成器
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE); // 设置缓冲区大小
  Serial.println("MQTT客户端已配置");
}

bool mqtt_reconnect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");
    // 创建一个随机的客户端ID
    String clientId = "ESP32-CAM-";
    clientId += String(random(0xffff), HEX); 
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");
      // 重新订阅主题
      mqttClient.subscribe("/motor"); // 订阅电机控制主题
      mqttClient.publish("/ESP32_info", "ESP32-CAM已上线"); // 发布上线消息
      mqttClient.publish("/motor/status", "电机控制已就绪"); // 发布电机状态
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
    JsonDocument doc; // 使用动态内存分配的JsonDocument以适应不同大小
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    
    // 检查是否包含电机速度参数
    if (doc.containsKey("speedA") && doc.containsKey("speedB")) {
       // 确保值是整数类型
       if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
            int speedA = doc["speedA"].as<int>();
            int speedB = doc["speedB"].as<int>();
            Serial.printf("通过MQTT设置电机速度: A=%d, B=%d\n", speedA, speedB);
            motor_control(0, speedA); // 控制电机A
            motor_control(1, speedB); // 控制电机B
       } else {
           Serial.println("JSON中的speedA或speedB类型错误，应为整数。");
       }
    } else {
        Serial.println("接收到的JSON缺少speedA或speedB字段。");
    }
  }
  // 可以添加对其他主题的处理逻辑
}