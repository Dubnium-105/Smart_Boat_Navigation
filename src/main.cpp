#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>       // 陀螺仪和加速度计库
#include <PubSubClient.h>  // MQTT客户端库
#include <ArduinoJson.h>   // JSON处理库
#include <Adafruit_MCP23X17.h> // I/O扩展模块库
 
// 函数前置声明
void setup_wifi();         // WiFi连接设置
void setup_mpu();          // 陀螺仪初始化
void setup_ir_sensors();   // 红外传感器初始化
void motor_control(uint8_t ch, int speed); // 电机控制
void send_sensor_data();   // 发送传感器数据到MQTT
bool mqtt_reconnect();     // MQTT重连
void process_ir_data();    // 处理红外传感器数据

// 网络配置 - 多个WiFi网络凭据
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"xbox", "12345678"}
};

WiFiClient espClient;       // WiFi客户端实例
PubSubClient mqttClient(espClient);  // MQTT客户端

// 硬件配置
#define MOTOR_A_IN1 38     // A电机控制引脚1
#define MOTOR_A_IN2 39     // A电机控制引脚2
#define MOTOR_B_IN3 40     // B电机控制引脚1
#define MOTOR_B_IN4 41     // B电机控制引脚2
MPU6050 mpu;               // 陀螺仪实例

// MCP23017配置 - I/O扩展模块
Adafruit_MCP23X17 mcp;

// 定义两个I2C总线引脚
#define MPU_SDA 19   // MPU6050的SDA引脚
#define MPU_SCL 20   // MPU6050的SCL引脚
#define MCP_SDA 21   // MCP23017的SDA引脚
#define MCP_SCL 47   // MCP23017的SCL引脚
#define MCP_ADDR 0x20  // MCP23017的I2C地址，默认为0x20

// 创建第二个I2C实例（Wire1）
TwoWire mcpWire = TwoWire(1);  // 使用I2C端口1

// 红外传感器方向矢量配置
// 16个传感器围成一个圆，每个传感器对应一个方向向量
// 数组中的每个元素代表传感器方向的单位向量[x, y]坐标
// 向量以极坐标均匀分布，0度指向正前方，然后每22.5度放置一个传感器
const float IR_VECTORS[16][2] = {
    {1.0, 0.0}, {0.9239, 0.3827}, {0.7071, 0.7071}, {0.3827, 0.9239},
    {0.0, 1.0}, {-0.3827, 0.9239}, {-0.7071, 0.7071}, {-0.9239, 0.3827},
    {-1.0, 0.0}, {-0.9239, -0.3827}, {-0.7071, -0.7071}, {-0.3827, -0.9239},
    {0.0, -1.0}, {0.3827, -0.9239}, {0.7071, -0.7071}, {0.9239, -0.3827}
};

// 全局变量
bool ir_sensor_status[16] = {0};  // 存储16个红外传感器的状态
float target_vector_x = 0.0;      // 目标方向向量的X分量
float target_vector_y = 0.0;      // 目标方向向量的Y分量
float target_angle = 0.0;         // 目标方向角度（度）
float target_strength = 0.0;      // 目标信号强度

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
    
    // 设置16个引脚为输入模式（带上拉电阻）
    for (int i = 0; i < 16; i++) {
        mcp.pinMode(i, INPUT_PULLUP);
    }
    Serial.println("红外传感器初始化完成");
}

void process_ir_data() {
    // 读取16个红外传感器的状态，取反是因为红外检测到物体时输出为低电平
    for (int i = 0; i < 16; i++) {
        ir_sensor_status[i] = !mcp.digitalRead(i);
    }
    
    // 初始化目标方向矢量的计算
    float sum_x = 0.0, sum_y = 0.0;
    int active_sensors = 0;
    
    // 计算所有激活传感器的方向矢量之和
    for (int i = 0; i < 16; i++) {
        if (ir_sensor_status[i]) {
            sum_x += IR_VECTORS[i][0];  // 累加X分量
            sum_y += IR_VECTORS[i][1];  // 累加Y分量
            active_sensors++;
        }
    }
    
    if (active_sensors > 0) {
        // 保存原始向量和
        target_vector_x = sum_x;
        target_vector_y = sum_y;
        
        // 计算信号强度（向量长度）
        target_strength = sqrt(sum_x * sum_x + sum_y * sum_y);
        
        // 归一化方向向量为单位向量
        if (target_strength > 0) {
            target_vector_x /= target_strength;
            target_vector_y /= target_strength;
        }
        
        // 计算目标角度（极坐标角度）
        float angle_rad = atan2(target_vector_y, target_vector_x);
        target_angle = angle_rad * 180.0 / PI;  // 转换为度
        if (target_angle < 0) target_angle += 360.0;  // 转换为0-360度范围
    } else {
        // 如果没有传感器被激活，重置所有值
        target_vector_x = 0.0;
        target_vector_y = 0.0;
        target_strength = 0.0;
        target_angle = -1.0;  // -1表示没有目标
    }
}

void setup_wifi() {
    for(int i=0; i<3; i++){
        const char* ssid = networks[i][0];
        const char* pass = networks[i][1];
        
        WiFi.begin(ssid, pass);
        Serial.printf("连接 %s...", ssid);
        
        int retries = 0;
        while(WiFi.status() != WL_CONNECTED && retries < 15){
          delay(50);
          Serial.print(".");
          retries++;
        }
        
        if(WiFi.status() == WL_CONNECTED){
          Serial.println("====================================");
          Serial.printf("\nConnected to %s\n", ssid);
          Serial.print("RSSI: ");
          Serial.println(WiFi.RSSI());
          Serial.print("MAC: ");
          Serial.println(WiFi.macAddress());
          Serial.print("Gateway: ");
          Serial.print(WiFi.gatewayIP());
          Serial.print("Subnet: ");
          Serial.println(WiFi.subnetMask());
          Serial.print("DNS: ");
          Serial.println(WiFi.dnsIP());
          Serial.print("Hostname: ");
          Serial.println(WiFi.getHostname());
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());
          Serial.println("===================================="); 
            return;
        }
        Serial.println("连接失败");
    }
    Serial.println("全部尝试失败!");
}

void setup_mpu() {
    Wire.begin(MPU_SDA, MPU_SCL);  // 使用MPU的专用I2C引脚
    mpu.initialize();
}

void setup_motors() {
  // 设置四个PWM通道用于控制两个电机
  ledcSetup(0, 5000, 8);  // 电机A方向1，5kHz频率，8位分辨率
  ledcAttachPin(MOTOR_A_IN1, 0);
  ledcSetup(1, 5000, 8);  // 电机A方向2
  ledcAttachPin(MOTOR_A_IN2, 1);
  ledcSetup(2, 5000, 8);  // 电机B方向1
  ledcAttachPin(MOTOR_B_IN3, 2);
  ledcSetup(3, 5000, 8);  // 电机B方向2
  ledcAttachPin(MOTOR_B_IN4, 3);
}

void motor_control(uint8_t motor, int speed) {
  // 限制速度值在-100到100之间
  speed = constrain(speed, -100, 100);
  
  // motor=0控制A电机，motor=1控制B电机
  if(motor == 0){
    if(speed > 0){  // 正向旋转
      ledcWrite(0, map(speed, 0,100, 0,255));  // 将速度从0-100映射到0-255范围
      ledcWrite(1, 0);
    } else {  // 反向旋转
      ledcWrite(0, 0);
      ledcWrite(1, map(abs(speed),0,100,0,255));
    }
  } else {
    if(speed > 0){  // 正向旋转
      ledcWrite(2, map(speed,0,100,0,255));
      ledcWrite(3, 0);
    } else {  // 反向旋转
      ledcWrite(2, 0);
      ledcWrite(3, map(abs(speed),0,100,0,255));
    }
  }
}

void send_sensor_data() {
    // 获取MPU6050陀螺仪和加速度计数据
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // 处理红外传感器数据
    process_ir_data();
    
    // 创建JSON文档用于MQTT发布
    JsonDocument doc;
    
    // 添加加速度计和陀螺仪数据
    doc["ax"] = ax;  // X轴加速度
    doc["ay"] = ay;  // Y轴加速度
    doc["az"] = az;  // Z轴加速度
    doc["gx"] = gx;  // X轴角速度
    doc["gy"] = gy;  // Y轴角速度
    doc["gz"] = gz;  // Z轴角速度
    
    // 添加16个红外传感器的状态数组
    JsonArray irArray = doc.createNestedArray("ir_sensors");
    for (int i = 0; i < 16; i++) irArray.add(ir_sensor_status[i]);
    
    // 添加目标方向和强度信息
    doc["target_angle"] = target_angle;        // 目标角度（0-360度）
    doc["target_strength"] = target_strength;  // 信号强度
    doc["target_x"] = target_vector_x;         // 单位向量X分量
    doc["target_y"] = target_vector_y;         // 单位向量Y分量
    
    // 基于红外传感器目标方向计算推荐的电机速度
    int left_speed = 0, right_speed = 0;
    
    // 只有当有效目标存在时才计算电机速度（信号强度大于阈值）
    if (target_strength > 0.1) {
        const int base_speed = 50;  // 基础速度
        
        // 目标在前方区域（315-360度或0-45度）
        if ((target_angle >= 315 || target_angle <= 45)) {
            // 计算转向因子：在315-360度范围内从0增至1，在0-45度范围内从0减至-1
            float turn_factor = (target_angle >= 315) ? 
                map(target_angle, 315, 360, 0, 1) : -map(target_angle, 0, 45, 0, 1);
                
            // 根据转向因子调整左右轮速度实现转向
            left_speed = base_speed - turn_factor * 20;
            right_speed = base_speed + turn_factor * 20;
        } 
        // 目标在右侧区域（45-135度）
        else if (target_angle > 45 && target_angle <= 135) {
            // 目标从右前到正右方的转向系数，从0增加到1
            float turn_factor = map(target_angle, 45, 135, 0, 1);
            
            // 保持左轮正常速度，减小右轮速度以实现向右转弯
            // 当目标在正右方时，右轮速度为负，原地右转
            left_speed = base_speed;
            right_speed = base_speed - turn_factor * base_speed * 2;
        }
        // 目标在后方区域（135-225度）
        else if (target_angle > 135 && target_angle <= 225) {
            // 目标在后方，需要后退并旋转
            // 从左后方到正后方再到右后方的转向因子，从-1到1
            float turn_factor = map(target_angle, 135, 225, -1, 1);
            
            // 负速度代表后退，并根据转向因子调整左右轮差速
            left_speed = -base_speed - turn_factor * 20;
            right_speed = -base_speed + turn_factor * 20;
        }
        // 目标在左侧区域（225-315度）
        else {
            // 目标从左后到正左方的转向系数，从1减小到0
            float turn_factor = map(target_angle, 225, 315, 1, 0);
            
            // 减小左轮速度，保持右轮正常速度以实现向左转弯
            // 当目标在正左方时，左轮速度为负，原地左转
            left_speed = base_speed - turn_factor * base_speed * 2;
            right_speed = base_speed;
        }
    }
    
    // 将推荐的电机速度添加到JSON文档
    doc["recommended_left"] = left_speed;
    doc["recommended_right"] = right_speed;
    
    // 序列化JSON并发布到MQTT
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
    static unsigned long lastMqttReconnect = 0;
    static unsigned long lastSensorDataSend = 0;
    unsigned long now = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接断开，尝试重连");
      setup_wifi();
      return;
    }
    
    if (!mqttClient.connected()) {
      if (now - lastMqttReconnect > 5000) {
        lastMqttReconnect = now;
        mqtt_reconnect();
      }
    } else {
      mqttClient.loop();
    }
    
    if (now - lastSensorDataSend > 100) {
      lastSensorDataSend = now;
      process_ir_data();
      send_sensor_data();
    }
}