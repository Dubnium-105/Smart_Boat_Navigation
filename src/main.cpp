/**
 * ESP32-S3摄像头最亮点检测 - 高性能优化版
 * 
 * 配置说明:
 * - 摄像头: ESP32S3_EYE
 * - 输出格式: 灰度图 (GRAYSCALE)
 * - 分辨率: QVGA (320x240)
 * - 时钟频率: 10MHz
 * - 内存优化设计，防止栈溢出
 * - 两核心独立任务
 */
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera_pins.h"  // 包含摄像头引脚定义
#include <PubSubClient.h>     // MQTT客户端库
#include <ArduinoJson.h>      // JSON处理库
#include "navigation.h"

// 多任务相关
TaskHandle_t serialMonitorTaskHandle = NULL;
SemaphoreHandle_t frameAccessMutex = NULL;

// 电机控制引脚定义
#define MOTOR_A_IN1 38     // A电机控制引脚1
#define MOTOR_A_IN2 39     // A电机控制引脚2
#define MOTOR_B_IN3 40     // B电机控制引脚1
#define MOTOR_B_IN4 41     // B电机控制引脚2

// 共享数据
volatile int sharedBrightX = 0;
volatile int sharedBrightY = 0;
volatile bool newBrightPointAvailable = false;
volatile bool systemIsBusy = false;      // 系统繁忙标志



// MQTT配置
const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 固定IP设置
#define USE_FIXED_IP true
IPAddress staticIP(192, 168, 31, 170);  // 固定IP地址
IPAddress gateway(192, 168, 31, 1);     // 网关
IPAddress subnet(255, 255, 255, 0);     // 子网掩码
IPAddress dns(8, 8, 8, 8);              // DNS服务器

// WiFi连接配置
struct WifiCredential {
  const char* ssid;
  const char* password;
};

const WifiCredential wifiCredentials[] = {
  {"room@407", "room@407"},
  {"xbox", "12345678"},
  {"xbox", "z1139827642"}
};
const int numWifiOptions = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

// 声明推流相关外部变量和函数
extern httpd_handle_t stream_httpd;
extern float currentFPS;
extern void startSimpleCameraStream();
extern unsigned long frameCount;
extern unsigned long lastFPSCalculationTime;
extern void find_brightest(const uint8_t* gray, int width, int height, int& out_x, int& out_y);
extern void print_ascii_frame(int width, int height, int bx, int by);
/**
 * 打印当前相机配置信息
 */
void printCameraSettings() {
  Serial.println("\n==== 相机配置信息 ====");
  sensor_t *s = esp_camera_sensor_get();
  
  // 打印帧大小
  switch(s->status.framesize) {
    case FRAMESIZE_QQVGA: Serial.println("当前分辨率: 160x120 (QQVGA)"); break;
    case FRAMESIZE_QVGA: Serial.println("当前分辨率: 320x240 (QVGA)"); break;
    case FRAMESIZE_VGA: Serial.println("当前分辨率: 640x480 (VGA)"); break;
    default: Serial.printf("当前分辨率: 其他 (%d)\n", s->status.framesize);
  }
  
  Serial.printf("JPEG品质: %d\n", s->status.quality);
  Serial.printf("亮度: %d\n", s->status.brightness);
  Serial.printf("对比度: %d\n", s->status.contrast);
  Serial.printf("饱和度: %d\n", s->status.saturation);
  Serial.printf("垂直翻转: %s\n", s->status.vflip ? "是" : "否");
  Serial.printf("水平镜像: %s\n", s->status.hmirror ? "是" : "否");
  Serial.printf("自动白平衡: %s\n", s->status.awb ? "开启" : "关闭");
  Serial.printf("自动增益: %s\n", s->status.agc ? "开启" : "关闭");
  Serial.printf("自动曝光: %s\n", s->status.aec ? "开启" : "关闭");
  Serial.println("=====================");
}

// 低优先级串口监控任务函数 - 分配到核心0，降低与视频流的冲突
void serialMonitorTask(void *parameter) {
  unsigned long lastDiagTime = 0;
  unsigned long lastBrightTime = 0;
  const int BRIGHT_REFRESH_INTERVAL = 800;  // 增加刷新间隔到800ms，减少串口负担

  while (true) {
    unsigned long currentTime = millis();

    // 每10秒输出一次诊断信息
    if (currentTime - lastDiagTime >= 10000) {
      // 检查是否系统繁忙，如果繁忙则跳过此次诊断
      if (!systemIsBusy) {
        Serial.printf("摄像头运行中，当前帧率: %.2f FPS\n", currentFPS);
        Serial.printf("当前IP: %s\n", WiFi.localIP().toString().c_str());
      }
      lastDiagTime = currentTime;
    }

    // 定期输出最亮点信息，间隔增大
    if (currentTime - lastBrightTime >= BRIGHT_REFRESH_INTERVAL) {
      // 只在系统不繁忙时尝试获取互斥锁
      if (!systemIsBusy && xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (newBrightPointAvailable) {
          // 安全复制共享数据以减少锁持有时间
          int local_x = sharedBrightX;
          int local_y = sharedBrightY;
          newBrightPointAvailable = false;
          xSemaphoreGive(frameAccessMutex);
          
          // 锁外处理数据，减少锁竞争
          Serial.printf("[最亮点] x=%d, y=%d\n", local_x, local_y);
          print_ascii_frame(24, 12, local_x * 24 / 320, local_y * 12 / 240);  // 使用更小的显示尺寸
        } else {
          xSemaphoreGive(frameAccessMutex);
        }
      }
      lastBrightTime = currentTime;
    }

    // 更长的延时，减轻CPU负担
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 在每个帧处理时手动调用的函数
void processFrame(camera_fb_t *fb) {
  static uint32_t last_frame_time = 0;
  uint32_t now = millis();
  
  // 降低处理频率，间隔增大到200ms
  if (now - last_frame_time > 200) {
    last_frame_time = now;
    
    // 尝试获取互斥锁，超时时间短以避免阻塞视频流
    if (xSemaphoreTake(frameAccessMutex, 10) == pdTRUE) {
      if (fb && fb->format == PIXFORMAT_GRAYSCALE) {
        int bx, by;
        find_brightest(fb->buf, fb->width, fb->height, bx, by);
        
        // 更新共享变量
        sharedBrightX = bx;
        sharedBrightY = by;
        newBrightPointAvailable = true;
      }
      xSemaphoreGive(frameAccessMutex);
    }
  }
}

// 系统健康监视任务 - 监控堆栈和内存
void systemMonitorTask(void* parameter) {
  while(true) {
    // 检查堆栈水位
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    
    // 如果堆栈剩余空间小于阈值，标记系统繁忙
    systemIsBusy = (uxHighWaterMark < 1024);  // 1KB阈值
    
    // 如果堆栈非常低，则打印警告
    if (uxHighWaterMark < 512) {
      Serial.printf("警告: 堆栈空间低: %d字节\n", uxHighWaterMark * 4);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));  // 每2秒检查一次
  }
}

/**
 * 电机初始化函数 - 配置PWM通道和引脚
 */
void setup_motors() {
  Serial.println("初始化电机控制...");
  
  // 设置四个PWM通道用于控制两个电机
  ledcSetup(0, 5000, 8);  // 电机A方向1，5kHz频率，8位分辨率
  ledcAttachPin(MOTOR_A_IN1, 0);
  ledcSetup(1, 5000, 8);  // 电机A方向2
  ledcAttachPin(MOTOR_A_IN2, 1);
  ledcSetup(2, 5000, 8);  // 电机B方向1
  ledcAttachPin(MOTOR_B_IN3, 2);
  ledcSetup(3, 5000, 8);  // 电机B方向2
  ledcAttachPin(MOTOR_B_IN4, 3);
  
  // 初始化时停止所有电机
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  
  Serial.println("电机控制初始化完成");
}

/**
 * 控制电机运动
 * @param motor 电机选择 (0=电机A, 1=电机B)
 * @param speed 速度值 (-100到100，负值表示反向)
 */
void motor_control(uint8_t motor, int speed) {
  // 确保速度在-100到100范围内
  speed = constrain(speed, -100, 100);
  
  // 将速度值映射到PWM范围
  int pwmValue = 0;
  if (speed > 0) {
    pwmValue = map(speed, 0, 100, 0, 255);
  } else if (speed < 0) {
    pwmValue = map(speed, -100, 0, -255, 0);
  }
  
  // 控制相应的电机
  // 这里根据实际电机控制代码实现
  // 例如: ledcWrite(motor, abs(pwmValue));
  Serial.printf("电机 %d 速度: %d\n", motor, speed);
}

/**
 * MQTT消息回调函数，处理接收到的MQTT消息
 * 支持通过/motor主题控制电机
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);
  
  // 将接收到的数据转换为字符串以便调试
  char message[256] = {0};
  if (length < 255) {
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("消息内容: %s\n", message);
  }
  
  // 处理电机控制消息
  if (strcmp(topic, "/motor") == 0) {
    // 使用ArduinoJson解析JSON数据
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }
    
    // 检查是否包含电机速度参数
    if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
      int speedA = doc["speedA"];
      int speedB = doc["speedB"];
      Serial.printf("设置电机速度: A=%d, B=%d\n", speedA, speedB);
      motor_control(0, speedA);
      motor_control(1, speedB);
    }
  }
}

/**
 * MQTT重连函数
 * 尝试连接到MQTT服务器并订阅相关主题
 */
bool mqtt_reconnect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");
    String clientId = "ESP32-CAM-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");
      mqttClient.subscribe("/motor");
      mqttClient.publish("/ESP32_info", "ESP32-CAM已上线");
      mqttClient.publish("/motor/status", "电机控制已就绪");
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

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // 创建互斥锁以保护帧访问
  frameAccessMutex = xSemaphoreCreateMutex();
  if (frameAccessMutex == NULL) {
    Serial.println("错误: 无法创建互斥锁!");
    ESP.restart();
  }

  // 配置摄像头参数
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;  // 10MHz时钟频率
  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // 灰度图格式
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  // 更改为WHEN_EMPTY，更稳定
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;  // 降低一点质量以减轻处理负担
  config.fb_count = 1;  // 单一帧缓冲，避免PSRAM过载

  // PSRAM检测及配置
  if (psramFound()) {
    Serial.println("找到PSRAM，启用相关功能");
  } else {
    Serial.println("未找到PSRAM，将使用有限的功能");
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败，错误代码: 0x%x\n", err);
    ESP.restart();
    return;
  }
  Serial.println("摄像头初始化成功");

  // 获取并配置摄像头传感器
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("获取传感器失败!");
    return;
  }

  // 摄像头参数优化
  s->set_brightness(s, 0);      // 标准亮度
  s->set_contrast(s, 0);        // 标准对比度
  s->set_saturation(s, 0);      // 标准饱和度
  s->set_sharpness(s, 0);       // 标准锐度
  s->set_denoise(s, 1);         // 开启降噪
  s->set_quality(s, 10);        // 质量等级
  s->set_special_effect(s, 0);  // 无特殊效果
  s->set_vflip(s, 1);           // 垂直翻转
  s->set_hmirror(s, 0);         // 不水平镜像
  
  // 增益和曝光控制
  s->set_gain_ctrl(s, 1);       // 自动增益控制
  s->set_exposure_ctrl(s, 1);   // 自动曝光控制
  s->set_agc_gain(s, 0);        // 最低增益
  s->set_awb_gain(s, 1);        // 自动白平衡增益
  s->set_lenc(s, 1);            // 镜头校正
  
  // 确保设置为QVGA
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  // 延迟一段时间让摄像头稳定
  delay(300);

  // 尝试连接WiFi
  bool connected = false;
  for (int i = 0; i < numWifiOptions; i++) {
    Serial.printf("尝试连接WiFi: %s\n", wifiCredentials[i].ssid);
    
    WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);
    WiFi.setSleep(false);  // 禁用WiFi睡眠模式以提高响应性
    
    // 尝试连接当前WiFi，最多等待3秒
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 12) {
      delay(250);
      Serial.print(".");
      attempts++;
    } 
    
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.println("");
      Serial.printf("连接成功: %s\n", wifiCredentials[i].ssid);
      
      // 设置固定IP
      if (USE_FIXED_IP) {
        if (!WiFi.config(staticIP, gateway, subnet, dns)) {
          Serial.println("固定IP配置失败");
        } else {
          Serial.printf("IP地址: %s\n", WiFi.localIP().toString().c_str());
        }
      }
      break;
    } else {
      Serial.println("");
      Serial.printf("连接失败: %s\n", wifiCredentials[i].ssid);
    }
  }
  
  if (!connected) {
    Serial.println("无法连接WiFi，重启设备...");
    ESP.restart();
  }

  // 打印摄像头配置
  printCameraSettings();
  
  // 初始化性能监控变量
  lastFPSCalculationTime = esp_timer_get_time();
  frameCount = 0;

  // 启动视频流服务器
  startSimpleCameraStream();

  // 创建串口监控任务 - 较低优先级
  xTaskCreatePinnedToCore(
    serialMonitorTask,    // 任务函数
    "SerialMonitor",      // 任务名称
    4096,                 // 降低堆栈大小以节省内存
    NULL,                 // 参数
    1,                    // 低优先级
    &serialMonitorTaskHandle,    // 任务句柄
    0                     // 在核心0上运行
  );
  
  // 创建系统监控任务
  xTaskCreatePinnedToCore(
    systemMonitorTask,    // 任务函数
    "SysMonitor",         // 任务名称
    2048,                 // 小堆栈大小
    NULL,                 // 参数
    2,                    // 中等优先级
    NULL,                 // 不需要任务句柄
    0                     // 在核心0上运行
  );

  // 初始化电机
  setup_motors();
  
  // 配置MQTT
  randomSeed(esp_random());  // 使用ESP32硬件随机数生成器
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
  
  // 尝试连接到MQTT服务器
  if (connected) {
    mqtt_reconnect();
  }

  // 初始化导航系统
  initNavigation();

  Serial.println("====================================");
  Serial.println("       摄像头系统启动完成!          ");
  Serial.println("====================================");
  Serial.print("访问: http://");
  Serial.print(WiFi.localIP());
  Serial.println(" 查看视频流");
  Serial.println("====================================");
}

// 当前循环函数已实现MQTT连接和基本图像处理，现在添加任务状态机逻辑
void loop() {
  // 定时获取图像并处理
  static unsigned long lastFrameTime = 0;
  static unsigned long lastMqttReconnect = 0;
  static unsigned long lastNavigationUpdate = 0;  // 导航更新时间
  unsigned long currentTime = millis();

  // 检查MQTT连接状态
  if (!mqttClient.connected()) {
    if (currentTime - lastMqttReconnect > 5000) {  // 每5秒尝试重连一次
      lastMqttReconnect = currentTime;
      mqtt_reconnect();
    }
  } else {
    mqttClient.loop();  // 处理MQTT消息
  }

  // 只在不处于繁忙状态时获取和处理帧
  if (!systemIsBusy && currentTime - lastFrameTime >= 150) {  // 150ms间隔
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // 处理帧数据，获取最亮点信息
      processFrame(fb);
      
      // 安全地获取最亮点数据
      int brightX = -1, brightY = -1;
      if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        brightX = sharedBrightX;
        brightY = sharedBrightY;
        xSemaphoreGive(frameAccessMutex);
      }
      
      // 根据当前状态执行导航逻辑
      if (currentTime - lastNavigationUpdate >= 300) {  // 每300ms更新一次导航逻辑
        // 调用独立的导航状态机处理各种任务状态
        navigationStateMachine(fb, brightX, brightY);
        lastNavigationUpdate = currentTime;
      }
      
      // 处理完毕，返回帧缓冲区
      esp_camera_fb_return(fb);
    }
    lastFrameTime = currentTime;
  }
  
  // 短暂延时，让其他任务有机会运行
  delay(20);
}
