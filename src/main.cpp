#include <WiFi.h>
#include <PubSubClient.h>  // MQTT客户端库
#include <ArduinoJson.h>   // JSON处理库
 
// 定义导航状态机的状态
enum NavigationState {
    STATE_STANDBY,        // 待机状态 - 不移动，等待命令
    STATE_NAVIGATING,     // 导航中 - 主动寻找并跟踪红外信号
    STATE_ARRIVED,        // 已到达终点
    STATE_RETURNING,      // 返回起点
    STATE_MANUAL,         // 手动控制
    STATE_ERROR           // 错误状态 - 如信号丢失、电量不足等
};

// 当前导航状态
NavigationState navState = STATE_STANDBY;
unsigned long stateStartTime = 0;     // 状态开始时间
unsigned long signalLostTime = 0;     // 信号丢失时间
bool signalLocked = false;            // 是否锁定目标信号

// 函数前置声明
void setup_wifi();         // WiFi连接设置
void setup_ir_sensors();   // 红外传感器初始化
void setup_motors();       // TB6612FNG电机驱动初始化
void motor_control(uint8_t ch, int speed); // TB6612FNG电机控制
int apply_motor_deadzone(int speed);  // 电机死区处理
void send_sensor_data();   // 发送传感器数据到MQTT
bool mqtt_reconnect();     // MQTT重连
void process_ir_data();    // 处理红外传感器数据
void IRAM_ATTR handleIRInterrupt(void *arg); // 红外传感器中断处理函数
void update_navigation_state(); // 更新导航状态
void execute_navigation_action(); // 基于当前状态执行动作

// 网络配置 - 多个WiFi网络凭据
const char* networks[][2] = {
    {"room@407", "room@407"},
    {"motorola", "1145141919"},
    {"IQOO Neo9 Pro", "NM_nm101030"}
};

WiFiClient espClient;       // WiFi客户端实例
PubSubClient mqttClient(espClient);  // MQTT客户端

// TB6612FNG电机驱动器硬件配置
#define AIN1_PIN 16        // 电机A方向控制1
#define AIN2_PIN 15        // 电机A方向控制2
#define BIN1_PIN 17        // 电机B方向控制1
#define BIN2_PIN 19        // 电机B方向控制2
#define PWMA_PIN 7        // 电机A PWM控制
#define PWMB_PIN 45        // 电机B PWM控制
#define STBY_PIN 21        // TB6612FNG待机控制引脚

// 电机死区设置
#define MOTOR_DEADZONE 10  // 死区范围：-10到+10之间的速度值会被设为0

// 红外传感器引脚定义
#define IR_SENSOR_0 14     // 0度 - 正前方 (对应原PB0)
#define IR_SENSOR_1 13     // 45度 - 右前方 (对应原PB1)
#define IR_SENSOR_2 12     // 315度 - 左前方 (对应原PB2)
#define IR_SENSOR_3 11     // 270度 - 正左方 (对应原PB3)
#define IR_SENSOR_4 10     // 225度 - 左后方 (对应原PB4)
#define IR_SENSOR_5 9     // 180度 - 正后方 (对应原PB5)
#define IR_SENSOR_6 46     // 135度 - 右后方 (对应原PB6)
#define IR_SENSOR_7 3     // 90度 - 正右方 (对应原PB7)

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
#define MIN_STRENGTH_THRESHOLD 0.8 // 最小信号强度阈值（降低以提高检测灵敏度）
#define SAMPLING_INTERVAL_MS 30   // 采样间隔（毫秒）

// 红外传感器状态变量
volatile uint8_t interrupt_pin = 0;  // 记录最后触发中断的传感器编号

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
            
            // 如果通过MQTT直接控制电机，自动切换到手动模式
            if (navState != STATE_MANUAL) {
                navState = STATE_MANUAL;
                stateStartTime = millis();
                Serial.println("导航状态: 切换到手动控制模式");
            }
        }
    }
    else if (strcmp(topic, "/navigation") == 0) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            Serial.print("JSON解析失败: ");
            Serial.println(error.c_str());
            return;
        }
        
        // 处理导航命令
        if (doc["command"].is<String>()) {
            String command = doc["command"].as<String>();
            Serial.printf("收到导航命令: %s\n", command.c_str());
            
            if (command == "standby") {
                navState = STATE_STANDBY;
                motor_control(0, 0);  // 停止电机A
                motor_control(1, 0);  // 停止电机B
                Serial.println("导航状态: 切换到待机模式");
            }
            else if (command == "navigate") {
                navState = STATE_NAVIGATING;
                stateStartTime = millis();
                Serial.println("导航状态: 开始导航");
            }
            else if (command == "return") {
                navState = STATE_RETURNING;
                stateStartTime = millis();
                Serial.println("导航状态: 开始返航");
            }            else if (command == "manual") {
                navState = STATE_MANUAL;
                Serial.println("导航状态: 切换到手动控制模式");
            }
            else if (command == "clear_history") {
                // 清除历史数据缓存
                for (int i = 0; i < HISTORY_SIZE; i++) {
                    history_vector_x[i] = 0;
                    history_vector_y[i] = 0;
                    history_strengths[i] = 0;
                    history_angles[i] = -1;
                }
                history_index = 0;
                history_filled = false;
                
                // 清除当前目标数据
                target_vector_x = 0.0;
                target_vector_y = 0.0;
                target_strength = 0.0;
                target_angle = -1.0;
                
                Serial.println("✨ 已手动清除历史数据缓存");
            }
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
            mqttClient.subscribe("/navigation");  // 订阅导航控制主题
            mqttClient.publish("/ESP32_info", "设备已上线");
            mqttClient.publish("/motor/status", "设备已连接");
            mqttClient.publish("/navigation/status", "{\"state\":\"standby\"}");  // 发布初始导航状态
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
    Serial.println("初始化红外传感器...");
    
    // 定义所有传感器引脚
    const uint8_t ir_pins[8] = {
        IR_SENSOR_0, IR_SENSOR_1, IR_SENSOR_2, IR_SENSOR_3, 
        IR_SENSOR_4, IR_SENSOR_5, IR_SENSOR_6, IR_SENSOR_7
    };
      // 初始化所有引脚为输入上拉模式（红外传感器通常是低电平有效的）
    for (int i = 0; i < 8; i++) {
        pinMode(ir_pins[i], INPUT_PULLUP);
        
        // 设置中断，当引脚状态变化时触发
        attachInterruptArg(digitalPinToInterrupt(ir_pins[i]), handleIRInterrupt, (void*)(uintptr_t)i, CHANGE);
        
        // 读取初始状态（由于红外检测到时为低电平，需要进行反转）
        ir_sensor_status[i] = !digitalRead(ir_pins[i]);
    }
    
    Serial.println("红外传感器初始化完成，8个传感器已设置中断");
}

void process_ir_data() {
    static unsigned long last_change_time = 0;  // 上次传感器状态变化时间
    unsigned long now = millis();
    
    // 获取各传感器状态
    const uint8_t ir_pins[8] = {
        IR_SENSOR_0, IR_SENSOR_1, IR_SENSOR_2, IR_SENSOR_3, 
        IR_SENSOR_4, IR_SENSOR_5, IR_SENSOR_6, IR_SENSOR_7
    };
    
    // 直接从引脚读取所有传感器状态
    bool sensor_changed = false;
    for (int i = 0; i < 8; i++) {
        // 读取引脚状态（由于红外检测到时为低电平，需要进行反转）
        bool current_state = !digitalRead(ir_pins[i]);
        
        // 检测状态是否变化
        if (current_state != ir_sensor_status[i]) {
            if (now - last_change_time > SENSOR_DEBOUNCE_MS) {
                ir_sensor_status[i] = current_state;
                sensor_changed = true;
            }
        }
    }
    
    // 如果有状态变化，更新最后变化时间
    if (sensor_changed) {
        last_change_time = now;
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
    }      // 如果所有传感器都是0，则清空历史记录
    if (active_sensors == 0) {
        static unsigned long allZeroStartTime = 0;
        static bool allZeroTimerStarted = false;
        
        if (!allZeroTimerStarted) {
            allZeroStartTime = now;
            allZeroTimerStarted = true;
        } else if (now - allZeroStartTime > 2000) {  // 连续2秒所有传感器都是0
            // 重置所有历史记录
            for (int i = 0; i < HISTORY_SIZE; i++) {
                history_vector_x[i] = 0;
                history_vector_y[i] = 0;
                history_strengths[i] = 0;
                history_angles[i] = -1;  // 无效角度
            }
            history_index = 0;
            history_filled = false;  // 标记历史记录为空
            
            // 立即清除当前目标数据
            target_vector_x = 0.0;
            target_vector_y = 0.0;
            target_strength = 0.0;
            target_angle = -1.0;
            
            allZeroTimerStarted = false;  // 重置计时器
            
            static unsigned long lastClearMessage = 0;
            if (now - lastClearMessage > 5000) {  // 每5秒最多打印一次
                Serial.println("📧 已清除历史数据缓存（所有传感器连续2秒为0）");
                lastClearMessage = now;
            }
            return;  // 直接返回，不进行后续处理
        }    } else {
        // 有传感器激活，重置计时器
        static bool allZeroTimerStarted = false;
        allZeroTimerStarted = false;
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
        if (valid_angles > 0 && active_sensors > 0) {
            // 如果当前有活跃的传感器且历史记录中有有效角度，才更新全局变量
            target_vector_x = sum_vector_x / valid_angles;
            target_vector_y = sum_vector_y / valid_angles;
            target_strength = sum_strengths / valid_angles;
            
            // 从平均向量重新计算角度，这比直接平均角度更准确
            float angle_rad = atan2(target_vector_y, target_vector_x);
            target_angle = angle_rad * 180.0 / PI;
            if (target_angle < 0) target_angle += 360.0;
        } else {
            // 如果没有有效数据或当前无活跃传感器，无条件重置所有值
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
        
        // 调试信息：显示当前活跃传感器数量和历史数据状态
        Serial.printf("活跃传感器: %d, 历史样本: %d/%d, ", 
                     active_sensors, 
                     history_filled ? HISTORY_SIZE : history_index,
                     HISTORY_SIZE);
        
        // 打印目标信息
        if (target_angle >= 0 && target_strength > 0.1 && (target_vector_x != 0 || target_vector_y != 0)) {
            Serial.printf("目标: 角度=%.1f°, 强度=%.2f, 矢量=(%.2f, %.2f)", 
                          target_angle, target_strength, target_vector_x, target_vector_y);
            
            // 显示数据来源
            if (active_sensors == 0) {
                Serial.print(" [来源:历史数据]");
            } else {
                Serial.print(" [来源:当前+历史]");
            }
            Serial.println();
        } else {
            Serial.println("目标: 未检测到");
            // 确保在输出"未检测到"时，所有目标相关值都被重置
            target_angle = -1.0;
            target_strength = 0.0;
            target_vector_x = 0.0;
            target_vector_y = 0.0;
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

void setup_motors() {
    Serial.println("初始化TB6612FNG电机驱动器...");
    
    // 设置引脚模式
    pinMode(STBY_PIN, OUTPUT);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    pinMode(PWMA_PIN, OUTPUT);
    pinMode(PWMB_PIN, OUTPUT);
    
    // 启用电机驱动器
    digitalWrite(STBY_PIN, HIGH);
    
    // 初始化所有输出为低电平
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMA_PIN, 0);
    analogWrite(PWMB_PIN, 0);
    
    Serial.println("TB6612FNG电机驱动初始化完成");
    Serial.printf("电机死区设置: ±%d (速度绝对值小于此值时电机停止)\n", MOTOR_DEADZONE);
}

// 电机死区处理函数
int apply_motor_deadzone(int speed) {
    if (abs(speed) < MOTOR_DEADZONE) {
        return 0;  // 在死区范围内，速度设为0
    }
    return speed;
}

void motor_control(uint8_t motor, int speed) {
    // 限制速度值在-100到100之间
    speed = constrain(speed, -100, 100);
    
    // 应用死区处理
    speed = apply_motor_deadzone(speed);
    
    // 将速度从-100~100映射到-255~255范围
    int pwm_value = map(abs(speed), 0, 100, 0, 255);
    
    // motor=0控制A电机，motor=1控制B电机
    if (motor == 0) {  // 电机A
        if (speed > 0) {  // 正向旋转
            digitalWrite(AIN1_PIN, HIGH);
            digitalWrite(AIN2_PIN, LOW);
            analogWrite(PWMA_PIN, pwm_value);
        } else if (speed < 0) {  // 反向旋转
            digitalWrite(AIN1_PIN, LOW);
            digitalWrite(AIN2_PIN, HIGH);
            analogWrite(PWMA_PIN, pwm_value);
        } else {  // 停止电机
            digitalWrite(AIN1_PIN, LOW);
            digitalWrite(AIN2_PIN, LOW);
            analogWrite(PWMA_PIN, 0);
        }
    } else if (motor == 1) {  // 电机B
        if (speed > 0) {  // 正向旋转
            digitalWrite(BIN1_PIN, HIGH);
            digitalWrite(BIN2_PIN, LOW);
            analogWrite(PWMB_PIN, pwm_value);
        } else if (speed < 0) {  // 反向旋转
            digitalWrite(BIN1_PIN, LOW);
            digitalWrite(BIN2_PIN, HIGH);
            analogWrite(PWMB_PIN, pwm_value);
        } else {  // 停止电机
            digitalWrite(BIN1_PIN, LOW);
            digitalWrite(BIN2_PIN, LOW);
            analogWrite(PWMB_PIN, 0);
        }
    }
}

void send_sensor_data() {
    // 处理红外传感器数据
    process_ir_data();
    
    // 创建JSON文档用于MQTT发布
    JsonDocument doc;
    
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
    
    // 添加导航状态信息
    const char* stateNames[] = {
        "standby",     // STATE_STANDBY
        "navigating",  // STATE_NAVIGATING
        "arrived",     // STATE_ARRIVED
        "returning",   // STATE_RETURNING
        "manual",      // STATE_MANUAL
        "error"        // STATE_ERROR
    };
    doc["nav_state"] = stateNames[navState];
    doc["state_duration"] = (millis() - stateStartTime) / 1000; // 当前状态持续时间（秒）
    
    if (navState == STATE_ERROR) {
        doc["error_code"] = 1; // 简单错误代码，可以根据需要扩展
        doc["error_message"] = "信号丢失";
    }
    
    // 序列化JSON并发布到MQTT
    String jsonStr;
    serializeJson(doc, jsonStr);
    mqttClient.publish("/sensor/data", jsonStr.c_str());
}

// 中断处理函数
void IRAM_ATTR handleIRInterrupt(void* arg) {
    // 获取是哪个传感器触发的中断
    uint8_t pin_num = (uint8_t)(uintptr_t)arg;
    
    // 在中断服务程序中尽量减少处理，只设置标志位
    portENTER_CRITICAL_ISR(&mux);
    interruptOccurred = true;
    interrupt_pin = pin_num;
    portEXIT_CRITICAL_ISR(&mux);
}

// 导航状态机状态更新函数
void update_navigation_state() {
    unsigned long now = millis();
    static unsigned long lastStateUpdate = 0;
    
    // 每100ms检查一次状态转换条件
    if (now - lastStateUpdate < 100) {
        return;
    }
    lastStateUpdate = now;
    
    // 记录之前的状态，用于检测状态变化
    NavigationState prevState = navState;
      switch (navState) {
        case STATE_STANDBY:
            // 待机状态无需自动转换，等待外部命令
            break;
              case STATE_NAVIGATING:
            {
                // 检查是否已到达终点（连续检测到强信号）
                static unsigned long strongSignalStartTime = 0;
                
                if (target_strength > 0.9 && target_angle >= 0) {
                    if (strongSignalStartTime == 0) {
                        // 第一次检测到强信号
                        strongSignalStartTime = now;
                    } else if (now - strongSignalStartTime > 3000) {
                        // 连续3秒检测到强信号，认为已到达终点
                        navState = STATE_ARRIVED;
                        stateStartTime = now;
                        
                        // 停止电机
                        motor_control(0, 0);
                        motor_control(1, 0);
                    }
                } else {
                    // 重置强信号计时器
                    strongSignalStartTime = 0;
                }
                
                // 检查是否丢失信号
                if (target_strength <= 0.1 || target_angle < 0) {
                    if (signalLostTime == 0) {
                        // 第一次检测到信号丢失
                        signalLostTime = now;
                    } else if (now - signalLostTime > 5000) {
                        // 连续5秒检测不到信号，进入错误状态
                        navState = STATE_ERROR;
                        stateStartTime = now;
                        
                        // 停止电机
                        motor_control(0, 0);
                        motor_control(1, 0);
                    }
                } else {
                    signalLostTime = 0; // 有信号，重置丢失计时器
                }
            }
            break;
            
        case STATE_ARRIVED:
            // 到达终点后5秒，自动切换到待机状态
            if (now - stateStartTime > 5000) {
                navState = STATE_STANDBY;
                stateStartTime = now;
            }
            break;
              case STATE_RETURNING:
            {
                // 返航状态的逻辑 - 可以寻找另一个红外信标
                // 简单实现：如果180度方向有信号，认为找到起点
                static unsigned long returnSignalTime = 0;
                
                if (target_angle >= 170 && target_angle <= 190 && target_strength > 0.8) {
                    if (returnSignalTime == 0) {
                        returnSignalTime = now;
                    } else if (now - returnSignalTime > 3000) {
                        // 连续3秒在后方检测到信号，认为已返回起点
                        navState = STATE_STANDBY;
                        stateStartTime = now;
                        
                        // 停止电机
                        motor_control(0, 0);
                        motor_control(1, 0);
                    }
                } else {
                    // 未检测到返回信号，重置计时器
                    returnSignalTime = 0;
                    // 可以实现转圈搜索等策略
                }
                
                // 如果长时间未返回（如2分钟），切换到待机模式
                if (now - stateStartTime > 120000) {
                    navState = STATE_STANDBY;
                    stateStartTime = now;
                }
            }
            break;
            
        case STATE_MANUAL:
            // 手动模式无需自动转换，除非30秒无操作
            if (now - stateStartTime > 30000) {
                // 30秒无操作，转到待机模式
                navState = STATE_STANDBY;
                stateStartTime = now;
                
                // 停止电机
                motor_control(0, 0);
                motor_control(1, 0);
            }
            break;
            
        case STATE_ERROR:
            // 错误状态10秒后自动尝试恢复到待机状态
            if (now - stateStartTime > 10000) {
                navState = STATE_STANDBY;
                stateStartTime = now;
            }
            break;
    }
    
    // 如果状态发生变化，打印日志并发送MQTT通知
    if (prevState != navState) {
        const char* stateNames[] = {
            "待机",     // STATE_STANDBY
            "导航中",   // STATE_NAVIGATING
            "已到达",   // STATE_ARRIVED
            "返航中",   // STATE_RETURNING
            "手动控制", // STATE_MANUAL
            "错误状态"  // STATE_ERROR
        };
        
        Serial.printf("导航状态变化: %s -> %s\n", 
                      stateNames[prevState], 
                      stateNames[navState]);
        
        // 状态变化时发送MQTT通知
        const char* mqttStateNames[] = {
            "standby", "navigating", "arrived", "returning", "manual", "error"
        };
        
        char stateMsg[100];
        snprintf(stateMsg, sizeof(stateMsg), 
                 "{\"state\":\"%s\",\"previous\":\"%s\",\"time\":%lu}", 
                 mqttStateNames[navState], 
                 mqttStateNames[prevState], 
                 now);
        
        mqttClient.publish("/navigation/status", stateMsg);
    }
}

// 高级自动控制算法 - 改进的精确追踪算法
void calculate_advanced_control(int &left_speed, int &right_speed) {
    if (target_strength <= 0.1 || target_angle < 0) {
        // 无目标时的搜索行为
        left_speed = 20;   // 缓慢右转搜索
        right_speed = -20;
        return;
    }
    
    // 动态速度参数
    const int base_speed = 55;
    const float max_turn_ratio = 1.5;
    const float pivot_threshold = 0.85;
    const float approach_threshold = 0.7;
    
    // 计算目标方向的转向系数
    float turn_factor = 0.0;
    float angle_rad = target_angle * PI / 180.0;
    
    // 计算最短转向角度
    float shortest_angle = target_angle;
    if (shortest_angle > 180) {
        shortest_angle = 360 - shortest_angle;
        turn_factor = -sin(angle_rad);  // 左转
    } else {
        turn_factor = sin(angle_rad);   // 右转
    }
    
    // 根据信号强度调整基础速度
    int dynamic_speed = base_speed;
    if (target_strength > approach_threshold) {
        // 接近目标时减速
        dynamic_speed = base_speed * (1.0 - (target_strength - approach_threshold) / (1.0 - approach_threshold) * 0.4);
    }
    
    // 精确的转向控制
    float turn_strength = abs(turn_factor);
    
    if (turn_strength > pivot_threshold) {
        // 大角度时原地转向
        int pivot_speed = dynamic_speed * 0.6;
        if (turn_factor > 0) {  // 右转
            left_speed = pivot_speed;
            right_speed = -pivot_speed / 2;
        } else {  // 左转
            left_speed = -pivot_speed / 2;
            right_speed = pivot_speed;
        }
    } else {
        // 小角度时差速转向
        float speed_diff = dynamic_speed * turn_strength * max_turn_ratio;
        
        if (target_angle >= 315 || target_angle < 45) {
            // 前方区域 - 直行为主，微调转向
            left_speed = dynamic_speed;
            right_speed = dynamic_speed;
            
            if (turn_factor > 0.1) {  // 需要右转
                right_speed -= speed_diff * 0.5;
            } else if (turn_factor < -0.1) {  // 需要左转
                left_speed -= speed_diff * 0.5;
            }
        } else if (target_angle >= 45 && target_angle < 135) {
            // 右侧区域
            left_speed = dynamic_speed;
            right_speed = dynamic_speed - speed_diff;
        } else if (target_angle >= 135 && target_angle < 225) {
            // 后方区域 - 根据哪侧更近选择转向方向
            int pivot_speed = dynamic_speed * 0.7;
            if (target_angle < 180) {  // 右后方，右转
                left_speed = pivot_speed;
                right_speed = -pivot_speed / 3;
            } else {  // 左后方，左转
                left_speed = -pivot_speed / 3;
                right_speed = pivot_speed;
            }
        } else {
            // 左侧区域
            left_speed = dynamic_speed - speed_diff;
            right_speed = dynamic_speed;
        }
    }
    
    // 限制速度范围
    left_speed = constrain(left_speed, -100, 100);
    right_speed = constrain(right_speed, -100, 100);
}

// 根据当前导航状态执行相应动作
void execute_navigation_action() {
    int left_speed = 0, right_speed = 0;
    
    switch (navState) {
        case STATE_STANDBY:
            // 待机状态 - 停止所有电机
            motor_control(0, 0);
            motor_control(1, 0);
            break;
            
        case STATE_NAVIGATING:
            // 导航状态 - 使用高级自动控制算法
            calculate_advanced_control(left_speed, right_speed);
            motor_control(0, left_speed);
            motor_control(1, right_speed);
            break;
            
        case STATE_ARRIVED:
            // 已到达终点 - 停止所有电机
            motor_control(0, 0);
            motor_control(1, 0);
            break;
            
        case STATE_RETURNING:
            // 返航状态 - 使用简化的返航算法
            {
                static bool turned180 = false;
                static unsigned long turnStartTime = 0;
                
                if (!turned180) {
                    // 首先执行180度转向
                    if (turnStartTime == 0) {
                        turnStartTime = millis();
                    }
                    
                    if (millis() - turnStartTime < 2500) {
                        motor_control(0, -35);  // 原地左转
                        motor_control(1, 35);
                    } else {
                        turned180 = true;
                        motor_control(0, 0);    // 短暂停止
                        motor_control(1, 0);
                        delay(200);
                    }
                } else {
                    // 转向完成，寻找起点信标
                    if (target_strength > 0.15 && target_angle >= 0) {
                        // 有信号时使用简化的控制算法
                        const int return_speed = 40;
                        
                        if (target_angle >= 315 || target_angle < 45) {
                            left_speed = return_speed;
                            right_speed = return_speed;
                        } else if (target_angle >= 45 && target_angle < 135) {
                            left_speed = return_speed;
                            right_speed = return_speed / 3;
                        } else if (target_angle >= 135 && target_angle < 225) {
                            left_speed = return_speed / 2;
                            right_speed = -return_speed / 2;
                        } else {
                            left_speed = return_speed / 3;
                            right_speed = return_speed;
                        }
                        
                        motor_control(0, left_speed);
                        motor_control(1, right_speed);
                    } else {
                        // 无信号时搜索
                        motor_control(0, 20);
                        motor_control(1, -20);
                    }
                }
            }
            break;
            
        case STATE_MANUAL:
            // 手动模式 - 电机控制由外部MQTT命令决定
            // 不在这里设置电机速度，由MQTT回调处理
            break;
            
        case STATE_ERROR:
            // 错误状态 - 停止电机并可能闪烁LED警告
            motor_control(0, 0);
            motor_control(1, 0);
            
            // 可选：添加错误指示（如闪烁LED）
            static unsigned long errorBlinkTime = 0;
            if (millis() - errorBlinkTime > 500) {
                errorBlinkTime = millis();
                // 这里可以添加LED闪烁代码
                Serial.println("ERROR: 系统处于错误状态");
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32红外追踪系统启动 ===");
    delay(1000);
    randomSeed(esp_random());  // 使用ESP32的硬件随机数生成器，避免使用ADC引脚
    
    setup_ir_sensors();
    setup_wifi();
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
    static unsigned long lastStatusDisplay = 0;
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
    
    // 周期性读取传感器状态，防止错过中断
    if (now - lastInterruptCheck > 1000) {
        lastInterruptCheck = now;
        process_ir_data();
    }
    
    // 更新导航状态机 - 检查状态转换条件并执行状态转换
    update_navigation_state();
    
    // 执行与当前状态相关的动作
    execute_navigation_action();
    
    // 每5秒显示一次当前导航状态
    if (now - lastStatusDisplay > 5000) {
        lastStatusDisplay = now;
        const char* stateNames[] = {
            "待机",     // STATE_STANDBY
            "导航中",   // STATE_NAVIGATING
            "已到达",   // STATE_ARRIVED
            "返航中",   // STATE_RETURNING
            "手动控制", // STATE_MANUAL
            "错误状态"  // STATE_ERROR
        };
        
        Serial.printf("当前导航状态: %s (已持续 %.1f 秒)\n", 
                     stateNames[navState], 
                     (now - stateStartTime) / 1000.0);
                     
        // 如果处于导航中状态，显示目标信息
        if (navState == STATE_NAVIGATING && target_angle >= 0) {
            Serial.printf("目标: 角度=%.1f°, 强度=%.2f\n", target_angle, target_strength);
        }
    }
}