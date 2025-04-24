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
void IRAM_ATTR handleInterrupt(); // 中断处理函数

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
#define MCP_INT_PIN 10     // MCP23017的INTB引脚连接到ESP32的G10
MPU6050 mpu;               // 陀螺仪实例

// MCP23017配置 - I/O扩展模块
Adafruit_MCP23X17 mcp;

// 定义两个I2C总线引脚
#define MPU_SDA 19   // MPU6050的SDA引脚
#define MPU_SCL 20   // MPU6050的SCL引脚
#define MCP_SDA 21   // MCP23017的SDA引脚
#define MCP_SCL 47   // MCP23017的SCL引脚
#define MCP_ADDR 0x27  // MCP23017的I2C地址，默认为0x20

// 创建第二个I2C实例（Wire1）
TwoWire mcpWire = TwoWire(1);  // 使用I2C端口1

// 中断相关变量
volatile bool interruptOccurred = false;  // 中断标志
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // 互斥锁，用于同步中断和主循环

// 红外传感器方向矢量配置
// 8个传感器围成一个圆，每个传感器对应一个方向向量
// 数组中的每个元素代表传感器方向的单位向量[x, y]坐标
// 传感器实际安装顺序为 0 1 7 6 5 4 3 2，与引脚对应关系如下:
// PB0 -> 0度    PB1 -> 45度   PB7 -> 90度   PB6 -> 135度
// PB5 -> 180度  PB4 -> 225度  PB3 -> 270度  PB2 -> 315度
const float IR_VECTORS[8][2] = {
    {1.0, 0.0},        // 0度 - 正前方 - PB0
    {0.7071, 0.7071},  // 45度 - 右前方 - PB1
    {0.7071, -0.7071}, // 315度 - 左前方 - PB2
    {0.0, -1.0},       // 270度 - 正左方 - PB3
    {-0.7071, -0.7071},// 225度 - 左后方 - PB4
    {-1.0, 0.0},       // 180度 - 正后方 - PB5
    {-0.7071, 0.7071}, // 135度 - 右后方 - PB6
    {0.0, 1.0},        // 90度 - 正右方 - PB7
};

// 全局变量
bool ir_sensor_status[8] = {0};  // 存储8个红外传感器的状态
float target_vector_x = 0.0;      // 目标方向向量的X分量
float target_vector_y = 0.0;      // 目标方向向量的Y分量
float target_angle = 0.0;         // 目标方向角度（度）
float target_strength = 0.0;      // 目标信号强度

// 滤波和稳定性相关变量
#define HISTORY_SIZE 5            // 历史数据缓冲区大小
float history_angles[HISTORY_SIZE] = {0};  // 角度历史记录
float history_strengths[HISTORY_SIZE] = {0}; // 强度历史记录
float history_vector_x[HISTORY_SIZE] = {0};  // X分量历史记录
float history_vector_y[HISTORY_SIZE] = {0};  // Y分量历史记录
int history_index = 0;            // 当前历史记录索引
bool history_filled = false;      // 历史记录是否已填满
#define SENSOR_DEBOUNCE_MS 3      // 传感器去抖延时（毫秒）（减小以提高检测速度）
#define MIN_STRENGTH_THRESHOLD 0.2 // 最小信号强度阈值（降低以提高检测灵敏度）
#define SAMPLING_INTERVAL_MS 30   // 采样间隔（毫秒）
// MQTT配置
// 批量读取相关变量 mqttServer = "emqx.link2you.top";
uint16_t input_port_state = 0;    // 存储MCP23017的GPIOB端口状态
bool batch_read_enabled = true;   // 是否启用批量读取模式

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
        
        if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
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
            Serial.println(" 1秒后重试");
            delay(1000);
        }
    }
    return false;
}

void setup_ir_sensors() {
    // 以更高的频率初始化MCP的I2C总线（标准是100kHz，我们提高到400kHz）
    mcpWire.begin(MCP_SDA, MCP_SCL, 400000);
    
    // 扫描I2C总线，查找所有设备
    Serial.println("扫描I2C设备...");
    byte error, address;
    int deviceCount = 0;
    for(address = 1; address < 127; address++) {
        mcpWire.beginTransmission(address);
        error = mcpWire.endTransmission();
        if (error == 0) {
            Serial.printf("  发现I2C设备: 0x%02X\n", address);
            deviceCount++;
            
            // 如果找到的设备地址与我们预期的MCP地址不同，更新MCP地址
            if(address != MCP_ADDR && (address == 0x20 || address == 0x21 || 
               address == 0x22 || address == 0x23 || address == 0x24 || 
               address == 0x25 || address == 0x26 || address == 0x27)) {
                Serial.printf("  更新MCP地址从0x%02X到0x%02X\n", MCP_ADDR, address);
            }
        }
    }
    
    if (deviceCount == 0) {
        Serial.println("未发现I2C设备！检查连接和上拉电阻。");
    }
    
    // 尝试初始化MCP23017，最多尝试5次
    bool mcpInitialized = false;
    for(int attempt = 1; attempt <= 5; attempt++) {
        Serial.printf("尝试初始化MCP23017 (第%d次)...\n", attempt);
        
        // 尝试使用检测到的地址进行初始化
        for(address = 0x20; address <= 0x27; address++) { // MCP23017可能的地址范围
            Serial.printf("  尝试地址: 0x%02X...", address);
            if(mcp.begin_I2C(address, &mcpWire)) {
                Serial.println("成功!");
                mcpInitialized = true;
                break;
            } else {
                Serial.println("失败");
            }
        }
        
        if(mcpInitialized) break;
        
        // 检查配线，等待一段时间后重试
        Serial.println("初始化失败。请检查:");
        Serial.println("1. MCP23017是否正确上电 (VDD和VSS)");
        Serial.println("2. I2C引脚连接是否正确 (SDA接" + String(MCP_SDA) + ", SCL接" + String(MCP_SCL) + ")");
        Serial.println("3. I2C引脚是否有10K上拉电阻");
        Serial.println("4. 地址引脚(A0-A2)是否设置正确");
        delay(1000);
    }
    
    if (!mcpInitialized) {
        Serial.println("MCP23017初始化失败！系统将继续运行，但红外感应功能不可用");
        return; // 不再执行死循环，允许程序继续运行
    }
    
    // 设置B组8个引脚为输入模式（带上拉电阻）- 注意这里改为8-15引脚（GPIOB组）
    for (int i = 8; i < 16; i++) {
        mcp.pinMode(i, INPUT_PULLUP);
    }
    
    // 配置中断功能
    Serial.println("配置MCP23017中断功能...");
    
    // 设置所有B组引脚为中断触发源
    for (int i = 8; i < 16; i++) {
        // 使能中断功能
        mcp.setupInterruptPin(i, CHANGE); // 当引脚状态变化时触发中断
    }
    
    // 设置MCP23017的中断配置
    // INTPOL = 1 (中断引脚电平高电平有效)
    // ODR = 0 (中断引脚为推挽输出模式)
    // 设置中断为非镜像模式，INTA对应PORTA, INTB对应PORTB
    mcp.setupInterrupts(true, false, CHANGE);
    
    // 设置ESP32的中断引脚和中断处理函数
    pinMode(MCP_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), handleInterrupt, FALLING);
    
    // 读取一次B组引脚状态，清除可能的中断标志
    input_port_state = mcp.readGPIOAB() >> 8; // 获取B组状态(高8位)
    
    Serial.println("中断配置完成，现在系统将通过中断响应传感器状态变化");
    Serial.println("红外传感器初始化完成");
}

void process_ir_data() {
    static unsigned long last_change_time = 0;  // 上次传感器状态变化时间
    unsigned long now = millis();
    
    // 批量读取B组引脚状态(8-15)
    uint16_t port_state = mcp.readGPIOAB();
    // 获取GPIOB的状态(高8位)
    uint8_t b_port = port_state >> 8;
    
    // 由于红外检测到物体时输出为低电平，需要反转状态
    b_port = ~b_port & 0xFF;
    
    // 更新传感器状态
    for (int i = 0; i < 8; i++) {
        // 从端口状态中提取每个传感器的状态位
        bool current_state = (b_port >> i) & 0x01;
        
        // 只有当状态稳定超过去抖时间后，才更新实际状态
        if (now - last_change_time > SENSOR_DEBOUNCE_MS) {
            ir_sensor_status[i] = current_state;
        }
    }
    
    // 记录状态变化时间
    if (b_port != (input_port_state & 0xFF)) {
        last_change_time = now;
        input_port_state = b_port;
    }
    
    // 检查是否所有传感器都被激活 - 这可能是干扰或异常情况
    bool all_sensors_active = true;
    for (int i = 0; i < 8; i++) {
        if (!ir_sensor_status[i]) {
            all_sensors_active = false;
            break;
        }
    }
    
    // 如果所有传感器都被激活，可能是干扰或异常，忽略这次数据
    if (all_sensors_active) {
        static unsigned long last_all_active_warning = 0;
        // 每2秒只打印一次警告，避免刷屏
        if (now - last_all_active_warning > 2000) {
            Serial.println("警告: 所有传感器同时激活，可能是干扰或异常情况，忽略本次数据");
            last_all_active_warning = now;
        }
        
        // 将目标信息设为无效
        target_vector_x = 0.0;
        target_vector_y = 0.0;
        target_strength = 0.0;
        target_angle = -1.0;
        
        // 退出函数，不进行后续处理
        return;
    }
    
    // 初始化目标方向矢量的计算
    float sum_x = 0.0, sum_y = 0.0;
    int active_sensors = 0;
    
    // 计算所有激活传感器的方向矢量之和
    for (int i = 0; i < 8; i++) {
        if (ir_sensor_status[i]) {
            sum_x += IR_VECTORS[i][0];  // 累加X分量
            sum_y += IR_VECTORS[i][1];  // 累加Y分量
            active_sensors++;
        }
    }
    
    // 临时变量用于存储当前计算结果
    float temp_vector_x = 0.0;
    float temp_vector_y = 0.0;
    float temp_strength = 0.0;
    float temp_angle = -1.0;
    
    if (active_sensors > 0) {
        // 计算信号强度（向量长度）
        temp_strength = sqrt(sum_x * sum_x + sum_y * sum_y);
        
        // 归一化方向向量为单位向量
        if (temp_strength > 0) {
            temp_vector_x = sum_x / temp_strength;
            temp_vector_y = sum_y / temp_strength;
            
            // 计算目标角度（极坐标角度）
            float angle_rad = atan2(temp_vector_y, temp_vector_x);
            temp_angle = angle_rad * 180.0 / PI;  // 转换为度
            if (temp_angle < 0) temp_angle += 360.0;  // 转换为0-360度范围
        }
    } else {
        // 如果没有传感器被激活，所有值保持为0或-1
    }
    
    // 过滤弱信号
    if (temp_strength < MIN_STRENGTH_THRESHOLD) {
        temp_strength = 0;
        temp_angle = -1;
        temp_vector_x = 0;
        temp_vector_y = 0;
    }
    
    // 将当前数据添加到历史记录
    history_vector_x[history_index] = temp_vector_x;
    history_vector_y[history_index] = temp_vector_y;
    history_strengths[history_index] = temp_strength;
    history_angles[history_index] = temp_angle;
    
    // 更新历史记录索引
    history_index = (history_index + 1) % HISTORY_SIZE;
    if (history_index == 0) {
        history_filled = true;  // 历史记录缓冲区已填满
    }
    
    // 应用滑动平均滤波，但仅当有有效数据时
    if (history_filled || history_index > 0) {
        // 计算有效样本数量
        int valid_samples = history_filled ? HISTORY_SIZE : history_index;
        
        // 初始化加权和
        float sum_vector_x = 0;
        float sum_vector_y = 0;
        float sum_strengths = 0;
        int valid_angles = 0;
        
        // 计算加权平均，最新的样本权重最高
        for (int i = 0; i < valid_samples; i++) {
            int idx = (history_index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;  // 从最新到最旧
            float weight = (valid_samples - i) / (float)valid_samples;  // 线性衰减权重
            
            // 只统计有效角度
            if (history_angles[idx] >= 0) {
                sum_vector_x += history_vector_x[idx] * weight;
                sum_vector_y += history_vector_y[idx] * weight;
                sum_strengths += history_strengths[idx] * weight;
                valid_angles++;
            }
        }
        
        // 只有当有足够的有效数据时才更新全局变量
        if (valid_angles > 0) {
            target_vector_x = sum_vector_x / valid_angles;
            target_vector_y = sum_vector_y / valid_angles;
            target_strength = sum_strengths / valid_angles;
            
            // 从平均向量重新计算角度，这比直接平均角度更准确
            float angle_rad = atan2(target_vector_y, target_vector_x);
            target_angle = angle_rad * 180.0 / PI;
            if (target_angle < 0) target_angle += 360.0;
        } else {
            // 如果没有有效数据，重置所有值
            target_vector_x = 0.0;
            target_vector_y = 0.0;
            target_strength = 0.0;
            target_angle = -1.0;
        }
    }
    
    // 在函数结束前添加串口输出
    static unsigned long lastSerialOutput = 0;
    
    // 每500毫秒输出一次，避免串口输出太频繁
    if (now - lastSerialOutput > 500) {
        lastSerialOutput = now;
        
        // 打印传感器状态
        Serial.print("传感器状态: [");
        for (int i = 0; i < 8; i++) {
            Serial.print(ir_sensor_status[i] ? "1" : "0");
            if (i < 7) Serial.print(", ");
        }
        Serial.print("] ");
        
        // 打印目标信息
        if (target_angle >= 0 && target_strength > 0.1) {
            Serial.printf("目标: 角度=%.1f°, 强度=%.2f, 矢量=(%.2f, %.2f)\n", 
                          target_angle, target_strength, target_vector_x, target_vector_y);
        } else {
            Serial.println("目标: 未检测到");
        }
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
    Serial.println("初始化MPU6050...");
    Wire.begin(MPU_SDA, MPU_SCL);  // 使用MPU的专用I2C引脚
    
    // 扫描I2C总线，查找所有设备
    Serial.println("扫描MPU I2C总线...");
    byte error, address;
    int deviceCount = 0;
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("  发现I2C设备: 0x%02X", address);
            if(address == 0x68 || address == 0x69) {
                Serial.println(" (可能是MPU6050)");
            } else {
                Serial.println();
            }
            deviceCount++;
        }
    }
    
    if (deviceCount == 0) {
        Serial.println("MPU6050 I2C总线上未发现设备！检查连接和上拉电阻。");
    }
    
    // 尝试初始化MPU6050，最多尝试3次
    bool mpuInitialized = false;
    for(int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("尝试初始化MPU6050 (第%d次)...\n", attempt);
        
        // 尝试使用可能的地址初始化MPU6050
        bool connectionResult = mpu.testConnection();
        
        if(connectionResult) {
            Serial.println("MPU6050连接成功！");
            mpu.initialize();
            mpuInitialized = true;
            
            // 验证设置 - 注意：移除了dmpInitialize调用，因为当前库不支持
            Serial.println("MPU6050基本初始化成功");
            
            break;
        } else {
            Serial.println("MPU6050连接失败");
            
            // 尝试单独联系0x69地址（备用地址）
            Wire.beginTransmission(0x69);
            if(Wire.endTransmission() == 0) {
                Serial.println("检测到MPU6050在备用地址0x69，尝试使用此地址");
                mpu = MPU6050(0x69);  // 使用备用地址创建新实例
            } else {
                delay(1000);  // 延迟后再次尝试
            }
        }
    }
    
    if (!mpuInitialized) {
        Serial.println("MPU6050初始化失败！系统将继续运行，但姿态感应功能不可用");
        // 创建虚拟数据以避免错误
    } else {
        // 设置MPU6050的配置
        Serial.println("配置MPU6050...");
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        
        Serial.println("MPU6050初始化完成");
    }
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
    
    // 添加8个红外传感器的状态数组 - 使用新的推荐语法
    JsonArray irArray = doc["ir_sensors"].to<JsonArray>();
    for (int i = 0; i < 8; i++) irArray.add(ir_sensor_status[i]);
    
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
        const float max_turn_ratio = 1.8;  // 最大转向比率
        const float pivot_threshold = 0.9;  // 原地转向阈值
        
        // 计算目标方向与船头朝向的夹角
        // 将目标角度转换为相对角度，使前方为0度，右为90度，后为180度，左为270度
        float relative_angle = target_angle;
        
        // 计算转向因子（-1.0到1.0）
        // 正值表示向右转，负值表示向左转，绝对值越大表示转向越急
        float turn_factor = 0.0;
        
        // 前方区域（-90到90度）
        if ((relative_angle >= 0 && relative_angle <= 90) || (relative_angle >= 270 && relative_angle <= 360)) {
            // 调整角度到-90到90范围
            float adjusted_angle = (relative_angle > 180) ? relative_angle - 360 : relative_angle;
            // 角度越大，转向系数越大
            turn_factor = adjusted_angle / 90.0;
        }
        // 后方区域（90到270度）
        else {
            // 调整角度到-180到180范围
            float adjusted_angle = (relative_angle > 270) ? relative_angle - 360 : relative_angle;
            adjusted_angle = (adjusted_angle > 180) ? 360 - adjusted_angle : adjusted_angle;
            // 后方区域的转向系数正负取决于最近的转向方向
            if (relative_angle > 90 && relative_angle < 180) {
                turn_factor = 1.0;  // 向右转最快到达
            } else if (relative_angle > 180 && relative_angle < 270) {
                turn_factor = -1.0; // 向左转最快到达
            } else if (relative_angle == 180) {
                // 正后方可以随机选择一个方向，或根据之前的状态选择
                turn_factor = (millis() % 2 == 0) ? 1.0 : -1.0;
            }
        }
        
        // 根据转向因子调整左右电机速度
        // 使用平滑的速度曲线来实现更精确的转向控制
        float turn_strength = abs(turn_factor);
        float turn_direction = (turn_factor >= 0) ? 1.0 : -1.0;
        
        // 前方区域使用差速转向
        if ((relative_angle >= 315 || relative_angle < 45) ||
            (relative_angle >= 135 && relative_angle < 225)) {
            // 前进或后退，根据方向调整差速
            bool is_forward = (relative_angle >= 315 || relative_angle < 45);
            int direction = is_forward ? 1 : -1;
            
            // 应用非线性转向曲线，提高小角度修正的精度
            float turn_adjustment = base_speed * turn_strength * max_turn_ratio;
            
            if (turn_strength > pivot_threshold) {
                // 大角度时考虑原地转向
                float pivot_factor = (turn_strength - pivot_threshold) / (1.0 - pivot_threshold);
                float reduced_speed = base_speed * (1.0 - pivot_factor * 0.8);
                
                if (turn_direction > 0) {
                    // 向右转
                    left_speed = direction * reduced_speed;
                    right_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor);
                    if (pivot_factor > 0.5) right_speed = -right_speed;  // 原地右转
                } else {
                    // 向左转
                    right_speed = direction * reduced_speed;
                    left_speed = direction * reduced_speed * (1.0 - 2.0 * pivot_factor);
                    if (pivot_factor > 0.5) left_speed = -left_speed;  // 原地左转
                }
            } else {
                // 小角度时使用传统差速
                if (turn_direction > 0) {
                    // 向右转
                    left_speed = direction * base_speed;
                    right_speed = direction * (base_speed - turn_adjustment);
                } else {
                    // 向左转
                    right_speed = direction * base_speed;
                    left_speed = direction * (base_speed - turn_adjustment);
                }
            }
        }
        // 侧面区域使用原地旋转 + 前进相结合
        else {
            // 计算侧向程度（0 = 正前/正后，1 = 正左/正右）
            float side_factor = 0.0;
            if (relative_angle >= 45 && relative_angle < 135) {
                // 右侧
                side_factor = sin(radians(relative_angle));
            } else if (relative_angle >= 225 && relative_angle < 315) {
                // 左侧
                side_factor = sin(radians(relative_angle));
            }
            
            // 使用平滑过渡从差速转向到原地旋转
            if (turn_direction > 0) {
                // 向右转
                left_speed = base_speed * (1.0 - side_factor * 0.5);
                right_speed = base_speed * (1.0 - side_factor * 1.5);
                if (side_factor > 0.7) right_speed = -right_speed * (side_factor - 0.7) / 0.3;
            } else {
                // 向左转
                right_speed = base_speed * (1.0 - side_factor * 0.5);
                left_speed = base_speed * (1.0 - side_factor * 1.5);
                if (side_factor > 0.7) left_speed = -left_speed * (side_factor - 0.7) / 0.3;
            }
        }
        
        // 确保速度值在有效范围内
        left_speed = constrain(left_speed, -100, 100);
        right_speed = constrain(right_speed, -100, 100);
    }
    
    // 将推荐的电机速度添加到JSON文档
    doc["recommended_left"] = left_speed;
    doc["recommended_right"] = right_speed;
    
    // 序列化JSON并发布到MQTT
    String jsonStr;
    serializeJson(doc, jsonStr);
    mqttClient.publish("/sensor/data", jsonStr.c_str());
}

// 中断处理函数
void IRAM_ATTR handleInterrupt() {
    // 在中断服务程序中尽量减少处理，只设置标志位
    portENTER_CRITICAL_ISR(&mux);
    interruptOccurred = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32红外追踪系统启动 ===");
    delay(1000);
    randomSeed(esp_random());  // 使用ESP32的硬件随机数生成器，避免使用ADC引脚
    
    Serial.println("初始化I2C...");
    Wire.begin(MPU_SDA, MPU_SCL);
    
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
    static unsigned long lastInterruptCheck = 0;
    unsigned long now = millis();
    
    // 检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi连接断开，尝试重连");
      setup_wifi();
      return;
    }
    
    // 检查MQTT连接状态
    if (!mqttClient.connected()) {
      if (now - lastMqttReconnect > 5000) {
        lastMqttReconnect = now;
        mqtt_reconnect();
      }
    } else {
      mqttClient.loop();
    }
    
    // 检查是否有中断发生
    bool shouldProcess = false;
    portENTER_CRITICAL(&mux);
    if (interruptOccurred) {
        interruptOccurred = false;
        shouldProcess = true;
    }
    portEXIT_CRITICAL(&mux);
    
    // 如果有中断发生，立即处理传感器数据
    if (shouldProcess) {
        //Serial.println("检测到中断，处理传感器数据");
        process_ir_data();
        send_sensor_data();
        lastSensorDataSend = now;
    }
    // 即使没有中断，也定期检查传感器状态（作为备份机制）
    else if (now - lastSensorDataSend > 100) {
        lastSensorDataSend = now;
        process_ir_data();
        send_sensor_data();
    }
    
    // 周期性清除并重新读取中断状态，防止中断标志丢失
    if (now - lastInterruptCheck > 1000) {
        lastInterruptCheck = now;
        // 读取一次中断状态寄存器，清除任何挂起的中断
        mcp.getLastInterruptPin();
        mcp.clearInterrupts();
    }
}