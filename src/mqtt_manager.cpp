#include "mqtt_manager.h"
#include "motor_control.h" // 需要调用电机控制函数
#include <ArduinoJson.h>   // JSON处理库
#include <esp_random.h>    // 用于生成随机客户端ID

// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 102400; // 增加缓冲区以防大数据包

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 外部变量声明
extern bool useStreamServer;
extern const char* streamServerUrl;
extern float currentFPS;
extern volatile bool systemIsBusy;

// 声明来自stream.cpp的函数
extern void configureStreamServer(bool enable, const char* serverUrl);

void setupMQTT() {
  randomSeed(esp_random()); // 使用ESP32硬件随机数生成器
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE); // 设置缓冲区大小
  Serial.println("MQTT客户端已配置");
}

// 函数声明
void publishSystemStatus();

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
      
      // 订阅所有需要的主题
      mqttClient.subscribe("/motor"); // 订阅电机控制主题
      mqttClient.subscribe("/stream/config"); // 订阅视频流配置主题
      
      // 构造JSON格式上线信息，包含WiFi和系统状态
      DynamicJsonDocument doc(512);
      doc["msg"] = "ESP32-CAM已上线 - 手动控制模式";
      doc["ip"] = WiFi.localIP().toString();
      doc["wifi_ssid"] = WiFi.SSID();
      doc["wifi_rssi"] = WiFi.RSSI();
      doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
      doc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
      doc["mode"] = "manual_control"; // 指示当前为手动控制模式

      String infoStr;
      serializeJson(doc, infoStr);
      mqttClient.publish("/ESP32_info", infoStr.c_str());
      mqttClient.publish("/motor/status", "电机控制已就绪 - 手动模式");
      
      // 发布系统状态
      publishSystemStatus();
      
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

void publishSystemStatus() {
  if (!mqttClient.connected()) return;
  
  DynamicJsonDocument doc(256);
  doc["fps"] = currentFPS;
  doc["ip"] = WiFi.localIP().toString();
  doc["busy"] = systemIsBusy;
  doc["stream_server"] = useStreamServer;
  if (useStreamServer) {
    doc["stream_server_url"] = streamServerUrl;
  }
  
  String statusStr;
  serializeJson(doc, statusStr);
  mqttClient.publish("/esp32/status", statusStr.c_str());
}

void loopMQTT() {
  if (!mqttClient.connected()) {
    // 如果未连接，则尝试重连（可以增加重连间隔逻辑）
    // mqtt_reconnect(); // 在主循环中更精细地控制重连时机
  } else {
    // 处理MQTT消息
    mqttClient.loop();
    
    // 定期发布系统状态
    static unsigned long lastStatusTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastStatusTime > 5000) { // 每5秒更新一次状态
      lastStatusTime = currentTime;
      publishSystemStatus();
    }
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
    
    // 检查是否包含电机速度参数
    if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
       // 获取电机速度值
       int speedA = doc["speedA"].as<int>();
       int speedB = doc["speedB"].as<int>();
       speedA = constrain(speedA, -100, 100); // 限制速度范围
       speedB = constrain(speedB, -100, 100); // 限制速度范围
       
       Serial.printf("通过MQTT设置电机速度: A=%d, B=%d\n", speedA, speedB);
       motor_control(0, speedA); // 控制电机A
       motor_control(1, speedB); // 控制电机B
       
       // 发送电机状态确认
       DynamicJsonDocument resDoc(128);
       resDoc["speedA"] = speedA;
       resDoc["speedB"] = speedB;
       resDoc["timestamp"] = millis();
       
       String resStr;
       serializeJson(resDoc, resStr);
       mqttClient.publish("/motor/status", resStr.c_str());
    } else {
       Serial.println("JSON中缺少speedA或speedB字段或类型错误，应为整数。");
    }
  }
  // 处理视频流配置消息
  else if (strcmp(topic, "/stream/config") == 0) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    
    if (doc["stream_server"].is<JsonObject>()) {
      bool enable = doc["stream_server"]["enable"].as<bool>();
      
      if (doc["stream_server"]["url"].is<const char*>()) {
        const char* url = doc["stream_server"]["url"];
        configureStreamServer(enable, url);
      } else {
        configureStreamServer(enable, NULL); // 修复：传递NULL作为第二个参数
      }
      
      // 发送配置确认
      DynamicJsonDocument resDoc(256);
      resDoc["stream_server"]["enable"] = useStreamServer;
      resDoc["stream_server"]["url"] = streamServerUrl;
      
      String resStr;
      serializeJson(resDoc, resStr);
      mqttClient.publish("/stream/status", resStr.c_str());
    }
  }
}