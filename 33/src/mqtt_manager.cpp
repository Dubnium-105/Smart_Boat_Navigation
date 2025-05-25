#include "mqtt_manager.h"
#include "motor_control.h"
#include "camera_module.h"
#include <ArduinoJson.h>
#include <esp_random.h>
#include <esp_camera.h>

// MQTT 配置
const char* mqtt_server = "47.93.157.148";
const int mqtt_port = 1883; // 修正为标准MQTT端口
const int MAX_MQTT_PACKET_SIZE = 10240;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setupMQTT() {
  randomSeed(esp_random());
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
  Serial.println("MQTT客户端已配置");
}

bool mqtt_reconnect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");
      mqttClient.subscribe("/motor");
      mqttClient.subscribe("/camera_control");
      mqttClient.publish("/motor/status", "电机控制已就绪");
      return true;
    } else {
      Serial.print("MQTT连接失败, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" 1秒后重试");
      delay(1000);
    }
  }
  if (!mqttClient.connected()) {
      Serial.println("MQTT连接失败次数过多。");
  }
  return mqttClient.connected();
}

void loopMQTT() {
  if (!mqttClient.connected()) {
    // 可在主循环中决定是否自动重连
  } else {
    // 提高消息处理速度：多次调用loop，减少延迟
    for (int i = 0; i < 5; ++i) {
      mqttClient.loop();
    }
    send_camera_frame_mqtt();
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    char message[256] = {0};
    if (length < sizeof(message) - 1) {
        memcpy(message, payload, length);
        message[length] = '\0';
    } else {
        strncpy(message, "消息过长", sizeof(message)-1);
    }
    Serial.printf("收到MQTT消息: 主题=%s, 内容=%s\n", topic, message);
    // 处理电机控制消息
    if (strcmp(topic, "/motor") == 0) {
        DynamicJsonDocument doc(256);
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            return;
        }
        if (doc.containsKey("speedA") && doc.containsKey("speedB")) {
            if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
                int speedA = doc["speedA"].as<int>();
                int speedB = doc["speedB"].as<int>();
                Serial.printf("已执行电机指令: speedA=%d, speedB=%d\n", speedA, speedB);
                motor_control(0, speedA);
                motor_control(1, speedB);
            } else {
                Serial.println("JSON中的speedA或speedB类型错误，应为整数。");
            }
        } else {
            Serial.println("接收到的JSON缺少speedA或speedB字段。");
        }
    } else if (strcmp(topic, "/camera") == 0) {
        Serial.printf("收到/camera图片数据，长度: %u 字节\n", length);
        // 可选：如需保存或处理图片，可在此处实现
    } else if (strcmp(topic, "/camera_control") == 0) {
        // 解析JSON，控制摄像头采集开关
        StaticJsonDocument<64> doc;
        DeserializationError err = deserializeJson(doc, payload, length);
        if (!err && doc.containsKey("capture")) {
            bool enable = doc["capture"].as<bool>();
            set_camera_capture_enabled(enable);
            Serial.printf("收到/camera_control指令，采集已%s\n", enable ? "开启" : "关闭");
        } else {
            Serial.println("/camera_control指令解析失败或缺少capture字段");
        }
    }
}
