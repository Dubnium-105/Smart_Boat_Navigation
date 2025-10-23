#include "mqtt_manager.h"
#include "config.h"
#include "motor_control.h" // 需要调用电机控制函数
#include <ArduinoJson.h>   // JSON处理库
#include <esp_random.h>    // 用于生成随机客户端ID
String g_clientId = "";
void publishStatusInfo(const char* extraMsg);
void setupMQTT() {
  randomSeed(esp_random());
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
  Serial.println("MQTT客户端已配置");
  // 注意：此时尚未建立与 Broker 的连接，避免在此处发布消息
}
// 统一MQTT状态信息反馈函数前向声明
void publishStatusInfo(const char* extraMsg) {
  JsonDocument infoDoc;
  infoDoc["msg"] = extraMsg ? extraMsg : "设备状态信息";
  infoDoc["ip"] = WiFi.localIP().toString();
  infoDoc["wifi_ssid"] = WiFi.SSID();
  infoDoc["wifi_rssi"] = WiFi.RSSI();
  infoDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  infoDoc["clientId"] = g_clientId;
  infoDoc["timestamp"] = millis();
  String infoStr;
  serializeJson(infoDoc, infoStr);
  mqttClient.publish("/ESP32_info", infoStr.c_str());
}
bool mqtt_reconnect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");
    // 创建一个随机的客户端ID
    String clientId = "ESP32-";
    clientId += SHIP_NAME;
    g_clientId = clientId; // 保存全局
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");
  // 订阅控制相关主题
  mqttClient.subscribe("/motor"); // 电机控制
  mqttClient.subscribe("/restart");
  mqttClient.subscribe("/check_mqtt"); 
  mqttClient.subscribe("/motor_control"); // 电机开关控制
  mqttClient.subscribe("/check_mqtt_reply"); // 前端对上线广播的回执
  publishStatusInfo("设备已上线");
      // 连接成功后，向 /check_mqtt 发送一次上线广播，告知网页端
      {
        JsonDocument onlineDoc;
        onlineDoc["cmd"] = "device_online";
        onlineDoc["clientId"] = g_clientId;
        onlineDoc["timestamp"] = millis();
        String onlineStr;
        serializeJson(onlineDoc, onlineStr);
        mqttClient.publish("/check_mqtt", onlineStr.c_str());
      }
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
  static unsigned long lastMqttReconnectAttempt = 0;
  unsigned long now = millis();
  if (!mqttClient.connected()) {
    if (now - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = now;
      mqtt_reconnect();
    }
  } else {
    mqttClient.loop();
  }
}
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
  // 将接收到的数据转换为字符串以便调试 (确保缓冲区足够大)
  char message[256]; 
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
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    // 新增：clientId过滤
    if (doc["clientId"].is<const char*>()) {
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
       extern int speedA, speedB;
       speedA = doc["speedA"].as<int>();
       speedB = doc["speedB"].as<int>();
       speedA = constrain(speedA, -255, 255);
       speedB = constrain(speedB, -255, 255);
       Serial.printf("通过MQTT设置电机PWM: A=%d, B=%d\n", speedA, speedB);
       motor_control(0, speedA);
       motor_control(1, speedB);
       JsonDocument resDoc;
       resDoc["speedA"] = speedA;
       resDoc["speedB"] = speedB;
       resDoc["pwm_direct"] = true;
       resDoc["timestamp"] = millis();
       String resStr;
       serializeJson(resDoc, resStr);
       mqttClient.publish("/motor/status", resStr.c_str());
    } else {
       Serial.println("JSON中缺少speedA或speedB字段或类型错误，应为整数。");
    }
  }
  // 处理MQTT连接检查与设备上线广播
  else if (strcmp(topic, "/check_mqtt") == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("/check_mqtt JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }

    const char* cmd = doc["cmd"] | "";
    if (strcmp(cmd, "ping") == 0) {
      // 前端健康检查请求 → 回复 pong 并附加一次状态
      mqttClient.publish("/check_mqtt_reply", "{\"msg\":\"pong\"}");
      Serial.println("收到/check_mqtt ping，已回复/pong");
      publishStatusInfo("MQTT连接检查响应");
    } else if (strcmp(cmd, "device_online") == 0) {
      // 忽略自身发出的上线广播，不做回应
      // 可在此扩展：记录其他设备上线事件
      Serial.println("收到设备上线广播(device_online)，已忽略");
    } else {
      Serial.printf("未知的 /check_mqtt cmd: %s\n", cmd);
    }
  }
  // 处理重启命令
  else if (strcmp(topic, "/restart") == 0) {
    Serial.println("收到重启命令，准备重启...");
    delay(1000);
    ESP.restart();
  }
  // 处理电机控制开关消息
  if (strcmp(topic, "/motor_control") == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error && doc["key"].is<const char*>()) {
      String key = doc["key"].as<const char*>();
      if (key == "shutdown") {
        extern int speedA, speedB;
        speedA = 0;
        speedB = 0;
        motor_control(0, 0);
        motor_control(1, 0);
        Serial.println("收到电机关闭命令，停止");
        JsonDocument resDoc;
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

  // 处理网页端对设备上线的回执
  if (strcmp(topic, "/check_mqtt_reply") == 0) {
    JsonDocument rep;
    DeserializationError err = deserializeJson(rep, payload, length);
    if (err) {
      Serial.print("/check_mqtt_reply JSON解析失败: ");
      Serial.println(err.c_str());
      return;
    }
    // 仅处理发给本设备的回执（to == g_clientId）
    const char* to = rep["to"] | "";
    if (to[0] && g_clientId == String(to)) {
      const char* webId = rep["webId"] | "";
      const char* msg = rep["msg"] | "";
      Serial.printf("收到网页端回执: webId=%s, msg=%s\n", webId, msg);
    }
  }
}

WiFiClient espClient;
PubSubClient mqttClient(espClient);