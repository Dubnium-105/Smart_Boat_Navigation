#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MCP23X17.h>
 
// 函数前置声明
void setup_wifi();
void setup_mpu();
void setup_ir_sensors();
void motor_control(uint8_t ch, int speed);
void send_sensor_data();
bool mqtt_reconnect();
void process_ir_data();

// 网络配置
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"xbox", "12345678"}
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 硬件配置
#define MOTOR_A_IN1 38
#define MOTOR_A_IN2 39
#define MOTOR_B_IN3 40
#define MOTOR_B_IN4 41
MPU6050 mpu;

// MCP23017配置
Adafruit_MCP23X17 mcp;

// 定义两个I2C总线引脚
#define MPU_SDA 19   // MPU6050的SDA引脚
#define MPU_SCL 20   // MPU6050的SCL引脚
#define MCP_SDA 21   // MCP23017的SDA引脚
#define MCP_SCL 47   // MCP23017的SCL引脚

// 创建第二个I2C实例（Wire1）
TwoWire mcpWire = TwoWire(1);  // 使用I2C端口1

// 红外传感器方向矢量配置
const float IR_VECTORS[16][2] = {
    {1.0, 0.0}, {0.9239, 0.3827}, {0.7071, 0.7071}, {0.3827, 0.9239},
    {0.0, 1.0}, {-0.3827, 0.9239}, {-0.7071, 0.7071}, {-0.9239, 0.3827},
    {-1.0, 0.0}, {-0.9239, -0.3827}, {-0.7071, -0.7071}, {-0.3827, -0.9239},
    {0.0, -1.0}, {0.3827, -0.9239}, {0.7071, -0.7071}, {0.9239, -0.3827}
};

// 全局变量
bool ir_sensor_status[16] = {0};
float target_vector_x = 0.0;
float target_vector_y = 0.0;
float target_angle = 0.0;
float target_strength = 0.0;

// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
    
    if (strcmp(topic, "/motor") == 0) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            return;
        }
        
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
        String clientId = "ESP32-" + String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("已连接到MQTT服务器");
            mqttClient.subscribe("/motor");
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

void setup_ir_sensors() {
    mcpWire.begin(MCP_SDA, MCP_SCL);  // 初始化MCP的I2C总线
    if (!mcp.begin_I2C(MCP_ADDR, &mcpWire)) {  // 显式指定Wire实例
        Serial.println("MCP23017初始化失败！");
        while (1);
    }
    
    // 设置16个引脚为输入模式
    for (int i = 0; i < 16; i++) {
        mcp.pinMode(i, INPUT_PULLUP);
    }
    Serial.println("红外传感器初始化完成");
}

void process_ir_data() {
    for (int i = 0; i < 16; i++) {
        ir_sensor_status[i] = !mcp.digitalRead(i);
    }
    
    float sum_x = 0.0, sum_y = 0.0;
    int active_sensors = 0;
    
    for (int i = 0; i < 16; i++) {
        if (ir_sensor_status[i]) {
            sum_x += IR_VECTORS[i][0];
            sum_y += IR_VECTORS[i][1];
            active_sensors++;
        }
    }
    
    if (active_sensors > 0) {
        target_vector_x = sum_x;
        target_vector_y = sum_y;
        target_strength = sqrt(sum_x * sum_x + sum_y * sum_y);
        
        if (target_strength > 0) {
            target_vector_x /= target_strength;
            target_vector_y /= target_strength;
        }
        
        float angle_rad = atan2(target_vector_y, target_vector_x);
        target_angle = angle_rad * 180.0 / PI;
        if (target_angle < 0) target_angle += 360.0;
    } else {
        target_vector_x = 0.0;
        target_vector_y = 0.0;
        target_strength = 0.0;
        target_angle = -1.0;
    }
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
    Wire.begin(MPU_SDA, MPU_SCL);  // 使用MPU的专用I2C引脚
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

void motor_control(uint8_t motor, int speed) {
  speed = constrain(speed, -100, 100);
  
  if(motor == 0){
    if(speed > 0){
      ledcWrite(0, map(speed, 0,100, 0,255));
      ledcWrite(1, 0);
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, map(abs(speed),0,100,0,255));
    }
  } else {
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
    process_ir_data();
    
    JsonDocument doc;
    doc["ax"] = ax;
    doc["ay"] = ay;
    doc["az"] = az;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;
    
    JsonArray irArray = doc.createNestedArray("ir_sensors");
    for (int i = 0; i < 16; i++) irArray.add(ir_sensor_status[i]);
    
    doc["target_angle"] = target_angle;
    doc["target_strength"] = target_strength;
    doc["target_x"] = target_vector_x;
    doc["target_y"] = target_vector_y;
    
    int left_speed = 0, right_speed = 0;
    if (target_strength > 0.1) {
        const int base_speed = 50;
        if ((target_angle >= 315 || target_angle <= 45)) {
            float turn_factor = (target_angle >= 315) ? map(target_angle, 315, 360, 0, 1) : -map(target_angle, 0, 45, 0, 1);
            left_speed = base_speed - turn_factor * 20;
            right_speed = base_speed + turn_factor * 20;
        } else if (target_angle > 45 && target_angle <= 135) {
            float turn_factor = map(target_angle, 45, 135, 0, 1);
            left_speed = base_speed;
            right_speed = base_speed - turn_factor * base_speed * 2;
        } else if (target_angle > 135 && target_angle <= 225) {
            float turn_factor = map(target_angle, 135, 225, -1, 1);
            left_speed = -base_speed - turn_factor * 20;
            right_speed = -base_speed + turn_factor * 20;
        } else {
            float turn_factor = map(target_angle, 225, 315, 1, 0);
            left_speed = base_speed - turn_factor * base_speed * 2;
            right_speed = base_speed;
        }
    }
    
    doc["recommended_left"] = left_speed;
    doc["recommended_right"] = right_speed;
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    mqttClient.publish("/sensor/data", jsonStr.c_str());
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32红外追踪系统启动 ===");
    delay(1000);
    randomSeed(analogRead(0));
    
    Serial.println("初始化I2C...");
    Wire.begin(19, 20);
    
    setup_ir_sensors();
    setup_wifi();
    setup_mpu();
    setup_motors();
    
    mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqtt_callback);
    
    if (!mqttClient.connected()) mqtt_reconnect();
}

void loop() {
    static unsigned long lastMsg = 0;
    unsigned long now = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接断开，尝试重连");
      setup_wifi();
      return;
    }
    
    if (!mqttClient.connected()) {
      if (now - lastMsg > 5000) {
        lastMsg = now;
        mqtt_reconnect();
      }
    } else {
      mqttClient.loop();
    }
    
    if (now - lastMsg > 100) {
      lastMsg = now;
      process_ir_data();
      send_sensor_data();
    }
}