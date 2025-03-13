#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include "esp_camera.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <base64.h> // 添加Base64编码库
#include "img_converters.h" // 用于图像格式转换

// 函数前置声明
void setup_camera();
void setup_wifi();
void setup_mpu();
void motor_control(uint8_t ch, int speed);
void process_command(String cmd);
void send_sensor_data();
void send_image();  // 修改为仅发送图像，不寻找最亮点
bool mqtt_reconnect();

// 网络配置
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"xbox", "12345678"}
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiServer tcpServer(5000);
WiFiClient tcpClient;

// 硬件配置
#define MOTOR_A_IN1 38
#define MOTOR_A_IN2 39
#define MOTOR_B_IN3 40
#define MOTOR_B_IN4 41
MPU6050 mpu;

// 摄像头配置
#define PWDN_GPIO_NUM     -1  // 无电源控制引脚
#define RESET_GPIO_NUM    -1  // 无硬件复位引脚
#define XCLK_GPIO_NUM     15  // 摄像头外部时钟
#define SIOD_GPIO_NUM     4   // I2C SDA
#define SIOC_GPIO_NUM     5   // I2C SCL
#define Y9_GPIO_NUM       16  // 数据线Y9
#define Y8_GPIO_NUM       17  // 数据线Y8
#define Y7_GPIO_NUM       18  // 数据线Y7
#define Y6_GPIO_NUM       12  // 数据线Y6
#define Y5_GPIO_NUM       10  // 数据线Y5
#define Y4_GPIO_NUM       8   // 数据线Y4
#define Y3_GPIO_NUM       9   // 数据线Y3
#define Y2_GPIO_NUM       11  // 数据线Y2
#define VSYNC_GPIO_NUM    6   // 垂直同步
#define HREF_GPIO_NUM     7   // 行同步
#define PCLK_GPIO_NUM     13  // 像素时钟

// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240; // 增加MQTT消息大小限制

// 图像传输配置
#define IMAGE_QUALITY 30     // JPEG压缩质量 (10-63)，越低质量越差但尺寸越小
#define FRAME_SIZE FRAMESIZE_QQVGA // 图像尺寸 (QVGA=320x240, QQVGA=160x120)
#define MAX_FRAME_RATE 5     // 最大帧率限制

// 全局变量
unsigned long last_frame_time = 0;
bool auto_send_image = true;  // 是否自动发送图像

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
    
    // 处理图像请求
    if (strcmp(topic, "/camera/request") == 0) {
        Serial.println("收到图像请求，准备发送图像");
        send_image();
        return;
    }
    
    // 处理图像流控制
    if (strcmp(topic, "/camera/stream") == 0) {
        if (length > 0 && payload[0] == '1') {
            auto_send_image = true;
            Serial.println("开启自动图像流");
        } else {
            auto_send_image = false;
            Serial.println("关闭自动图像流");
        }
        return;
    }
    
    // 处理电机控制命令
    if (strcmp(topic, "/motor") == 0) {
        JsonDocument doc;  // 使用JsonDocument替代StaticJsonDocument
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            return;
        }
        
        Serial.println("处理电机控制命令");
        
        // 处理 boot 命令
        if (doc["boot"].is<const char*>()) {
            const char* val = doc["boot"];
            Serial.printf("收到boot命令: %s\n", val);
            if (strcmp(val, "up") == 0) {
                motor_control(0, 10);
                motor_control(1, 10);
            } else if (strcmp(val, "down") == 0) {
                motor_control(0, -10);
                motor_control(1, -10);
            }
        }
        
        // 处理直接速度设置
        if (doc.containsKey("speedA") && doc.containsKey("speedB")) {
            int speedA = doc["speedA"];
            int speedB = doc["speedB"];
            Serial.printf("设置电机速度: A=%d, B=%d\n", speedA, speedB);
            motor_control(0, speedA);
            motor_control(1, speedB);
        }
    }
}

bool mqtt_reconnect() {
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 5) {
        attempts++;
        Serial.print("尝试MQTT连接...");
        // 创建随机客户端ID
        String clientId = "ESP32CAM-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("已连接到MQTT服务器");
            // 订阅主题
            mqttClient.subscribe("/motor");
            mqttClient.subscribe("/motor_control");
            mqttClient.subscribe("/camera/request");
            mqttClient.subscribe("/camera/stream");
            
            // 发布状态消息
            mqttClient.publish("/ESP32_info", "设备已上线");
            mqttClient.publish("/motor/status", "设备已连接");
            return true;
        } else {
            Serial.print("MQTT连接失败, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" 5秒后重试");
            delay(5000);
        }
    }
    return false;
}

// 完整的摄像头初始化
void setup_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 40000000;
    config.pixel_format = PIXFORMAT_JPEG; // 直接使用JPEG格式
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = IMAGE_QUALITY;
    config.fb_count = 2; // 使用2个帧缓冲区提高性能
  
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      ESP.restart();
    }
    
    // 获取传感器对象并设置参数
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, -1);    // -2 to 2 (降低饱和度有助于检测白色)
        s->set_special_effect(s, 0); // 0 = no effect
        s->set_whitebal(s, 1);       // 1 = enable
        s->set_awb_gain(s, 1);       // 1 = enable
        s->set_wb_mode(s, 0);        // 0 = auto
        s->set_exposure_ctrl(s, 1);  // 1 = enable
        s->set_aec2(s, 0);           // 0 = disable
        s->set_gain_ctrl(s, 1);      // 1 = enable
        s->set_agc_gain(s, 0);       // 0 = gain
        s->set_gainceiling(s, (gainceiling_t)0); // 0 = 2x
        s->set_bpc(s, 1);            // 1 = enable
        s->set_wpc(s, 1);            // 1 = enable
        s->set_raw_gma(s, 1);        // 1 = enable
        s->set_lenc(s, 1);           // 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable
        s->set_vflip(s, 0);          // 0 = disable
        s->set_dcw(s, 1);            // 1 = enable
        
        // 优化白色检测的设置
        s->set_aec_value(s, 200);    // 适中的曝光值，避免过曝
        s->set_ae_level(s, 0);       // 自动曝光水平
    }
}

// 图像处理和发送相关函数 - 使用Base64编码
void send_image() {
    // 帧率控制
    unsigned long current_time = millis();
    if (current_time - last_frame_time < (1000 / MAX_FRAME_RATE)) {
        return; // 限制帧率
    }
    last_frame_time = current_time;
    
    Serial.println("获取图像...");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("获取图像失败");
        return;
    }
    
    Serial.printf("获取图像成功: %dx%d, 格式: %d, 大小: %d字节\n", 
                fb->width, fb->height, fb->format, fb->len);
    
    // 创建JSON对象包含图像信息
    JsonDocument doc;  // 使用JsonDocument替代StaticJsonDocument
    doc["width"] = fb->width;
    doc["height"] = fb->height;
    doc["format"] = fb->format;
    doc["size"] = fb->len;
    
    // 将图像数据转换为Base64编码
    String base64Image = base64::encode(fb->buf, fb->len);
    
    // 释放图像缓冲区
    esp_camera_fb_return(fb);
    
    // 发送图像信息
    String jsonStr;
    serializeJson(doc, jsonStr);
    mqttClient.publish("/camera/info", jsonStr.c_str());
    
    // 计算需要分割的块数
    const int max_chunk_size = 4096; // MQTT消息大小限制
    int num_chunks = (base64Image.length() + max_chunk_size - 1) / max_chunk_size;
    
    // 发送图像数据
    for (int i = 0; i < num_chunks; i++) {
        int start_pos = i * max_chunk_size;
        int end_pos = min(start_pos + max_chunk_size, (int)base64Image.length());
        String chunk = base64Image.substring(start_pos, end_pos);
        
        // 创建包含块索引和总块数的主题
        char chunk_topic[32];
        sprintf(chunk_topic, "/camera/image/%d/%d", i, num_chunks);
        
        // 发送图像块
        bool success = mqttClient.publish(chunk_topic, chunk.c_str());
        if (!success) {
            Serial.printf("发送图像块 %d/%d 失败\n", i+1, num_chunks);
        }
    }
    
    Serial.printf("图像发送完成，共%d块\n", num_chunks);
}

void setup_wifi() {
    for(int i=0; i<3; i++){
        const char* ssid = networks[i][0];
        const char* pass = networks[i][1];
        
        WiFi.begin(ssid, pass);
        Serial.printf("Trying %s...", ssid);
        
        int retries = 0;
        while(WiFi.status() != WL_CONNECTED && retries < 15){
          delay(1000);
          Serial.print(".");
          retries++;
        }
        
        if(WiFi.status() == WL_CONNECTED){
          Serial.printf("\nConnected to %s\n", ssid);
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());
          return;
        }
        Serial.println("\nFailed");
    }
    Serial.println("All networks failed!");
}

void setup_mpu() {
    Wire.begin(19, 20); // SDA, SCL
    mpu.initialize();
}

void setup_motors() {
  ledcSetup(0, 5000, 8);
  ledcAttachPin(MOTOR_A_IN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_A_IN2, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(MOTOR_B_IN3, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(MOTOR_B_IN4, 3);
}

void process_command(String cmd) {
    if (cmd == "PING") {
        tcpClient.print("PONG\n");
        return;
    }
    if(cmd.startsWith("MOTOR")){
      int firstComma = cmd.indexOf(',');
      int secondComma = cmd.indexOf(',', firstComma+1);
      
      int left = cmd.substring(firstComma+1, secondComma).toInt();
      int right = cmd.substring(secondComma+1).toInt();
      
      motor_control(0, left);
      motor_control(1, right);
      
      // 回传确认
      tcpClient.print("ACK,MOTOR," + String(left) + "," + String(right) + "\n");
    }
}

void motor_control(uint8_t motor, int speed) {
  speed = constrain(speed, -100, 100);
  
  if(motor == 0){ // MOTOR_A
    if(speed > 0){
      ledcWrite(0, map(speed, 0,100, 0,255));
      ledcWrite(1, 0);
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, map(abs(speed),0,100,0,255));
    }
  } else { // MOTOR_B
    if(speed > 0){
      ledcWrite(2, map(speed,0,100,0,255));
      ledcWrite(3, 0);
    } else {
      ledcWrite(2, 0);
      ledcWrite(3, map(abs(speed),0,100,0,255));
    }
  }
}

void send_sensor_data() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  JsonDocument doc;  // 使用JsonDocument替代StaticJsonDocument
  doc["ax"] = ax;
  doc["ay"] = ay;
  doc["az"] = az;
  doc["gx"] = gx;
  doc["gy"] = gy;
  doc["gz"] = gz;
  
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  mqttClient.publish("/sensor/data", jsonStr.c_str());
  if(tcpClient.connected()) {
      tcpClient.println(jsonStr);
  }
  
  // 如果启用了自动发送图像，则发送图像
  if (auto_send_image) {
    send_image();
  }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32-CAM 启动 ===");
    delay(1000);
    
    randomSeed(analogRead(0)); // 初始化随机数生成器
    
    Serial.println("初始化摄像头...");
    setup_camera();
    Serial.println("摄像头初始化完成");
    
    Serial.println("连接WiFi...");
    setup_wifi();
    
    Serial.println("初始化MPU...");
    setup_mpu();
    
    Serial.println("初始化电机...");
    setup_motors();
    
    // 初始化MQTT
    Serial.println("配置MQTT...");
    mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE); // 增加MQTT缓冲区大小
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqtt_callback);
    
    Serial.println("启动TCP服务器...");
    tcpServer.begin();
    
    Serial.println("设置完成，开始运行");
    
    // 发送一条测试消息
    if (mqttClient.connected()) {
      mqttClient.publish("/ESP32_info", "设备启动完成");
    } else {
      Serial.println("MQTT未连接，尝试重连");
      mqtt_reconnect();
    }
}

void loop() {
    static unsigned long lastMsg = 0;
    static unsigned long lastReconnectAttempt = 0;
    unsigned long now = millis();
    
    // 检查WiFi连接
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接断开，尝试重连");
      setup_wifi();
      return;
    }
    
    // 处理MQTT连接
    if (!mqttClient.connected()) {
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;
        Serial.println("MQTT连接断开，尝试重连");
        if (mqtt_reconnect()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      // MQTT连接正常，处理消息
      mqttClient.loop();
    }
    
    // 处理TCP客户端
    if (tcpClient && !tcpClient.connected()) {
        Serial.println("TCP客户端断开连接");
        tcpClient.stop();
        delay(100);
    }

    // 接受新连接
    if (!tcpClient || !tcpClient.connected()) {
      WiFiClient newClient = tcpServer.available();
      if (newClient) {
          tcpClient = newClient;
          tcpClient.setTimeout(100);
          Serial.println("新TCP客户端已连接");
      }
    } else {
      // 处理TCP客户端数据
      if (tcpClient.available()) {
        String cmd = tcpClient.readStringUntil('\n');
        if (cmd.length() > 0) {
          Serial.printf("收到TCP命令: %s\n", cmd.c_str());
          process_command(cmd);
        }
      }
    }
    
    // 定期发送传感器数据
    if (now - lastMsg > 100) {  // 减少间隔到100毫秒，提高响应速度
      lastMsg = now;
      send_sensor_data();
    }
}