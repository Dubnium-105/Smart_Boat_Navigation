#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// 本地服务器配置
const char* ssid = "room@407";
const char* password = "room@407";
const char* mqttServer = "8.210.4.26";  // 本地服务器IP
const int mqttPort = 1883;  // 默认MQTT端口
const char* mqttUser = "boat";
const char* mqttPassword = "1111";

void callback(char* topic, byte* payload, unsigned int length);
void set_motors(int left_percent, int right_percent);
void reconnect();
// 电机引脚定义（L298N控制引脚）
#define MOTOR_L_DIR1 D1  // IN1（GPIO5）
#define MOTOR_L_DIR2 D2  // IN2（GPIO4）
#define MOTOR_R_DIR1 D3  // IN3（GPIO0）
#define MOTOR_R_DIR2 D4  // IN4（GPIO2）
#define MOTOR_L_PWM   D5  // ENA（左电机使能，GPIO5）
#define MOTOR_R_PWM   D6  // ENB（右电机使能，GPIO6）

WiFiClient espClient;
PubSubClient client(espClient);

// 电机速度（-100 到 100）
int motorL_percent = 0;
int motorR_percent = 0;

// 最大PWM值，防止电机烧毁
const int maxPWM = 175;  // 最大PWM值
int motorL_speed = 0;
int motorR_speed = 0;

// 定时变量
unsigned long lastMillis = 0;
const long interval = 5000;  // 每5秒打印并发布一次数据

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_L_DIR1, OUTPUT);
  pinMode(MOTOR_L_DIR2, OUTPUT);
  pinMode(MOTOR_R_DIR1, OUTPUT);
  pinMode(MOTOR_R_DIR2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);

  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  // 连接MQTT服务器
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  reconnect();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (String(topic) == "/control") {
    int commaIndex = message.indexOf(',');
    if (commaIndex > 0) {
      int left = message.substring(0, commaIndex).toInt();  // 获取左电机速度百分比
      int right = message.substring(commaIndex + 1).toInt();  // 获取右电机速度百分比
      set_motors(left, right);
    }
  }
}

void set_motors(int left_percent, int right_percent) {
  // 计算实际PWM速度值（-100到100转换为0到maxPWM范围）
  motorL_percent = constrain(left_percent, -100, 100);
  motorR_percent = constrain(right_percent, -100, 100);

  // 设置电机方向
  digitalWrite(MOTOR_L_DIR1, motorL_percent >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_L_DIR2, motorL_percent >= 0 ? LOW : HIGH);
  digitalWrite(MOTOR_R_DIR1, motorR_percent >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_DIR2, motorR_percent >= 0 ? LOW : HIGH);

  // 转换百分比到PWM范围 (-100~100 -> 0~maxPWM)
  motorL_speed = map(abs(motorL_percent), 0, 100, 0, maxPWM);
  motorR_speed = map(abs(motorR_percent), 0, 100, 0, maxPWM);

  // 设置电机PWM值
  analogWrite(MOTOR_L_PWM, motorL_speed);
  analogWrite(MOTOR_R_PWM, motorR_speed);

  // 输出电机速度
  Serial.print("Left Motor Speed: ");
  Serial.print(motorL_speed);
  Serial.print(" | Right Motor Speed: ");
  Serial.println(motorR_speed);
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP8266Client", mqttUser, mqttPassword)) {
      client.subscribe("/control");
    } else {
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  
  // 每隔一定时间打印电机速度并发布到ESP32_info话题
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;

    // 打印电机速度
    Serial.print("Left Motor Speed: ");
    Serial.print(motorL_speed);
    Serial.print(" | Right Motor Speed: ");
    Serial.println(motorR_speed);

    // 发布数据到ESP32_info话题
    String payload = String("Left Motor Speed: ") + motorL_speed + " | Right Motor Speed: " + motorR_speed;
    client.publish("ESP32_info", payload.c_str());
  }

  client.loop();
}