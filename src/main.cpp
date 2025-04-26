/**
<<<<<<< HEAD
 * ESP32 摄像头 - 简化版
 * 
 * 配置说明:
 * - 摄像头: ESP32S3_EYE
 * - 输出格式: 灰度图 (GRAYSCALE)
 * - 分辨率: QVGA (320x240)
 * - 时钟频率: 20MHz
 * - 自动增益功能已开启
 * - 简化视频流服务，直接输出到IP
 * 
 * 功能:
 * - 简化的WiFi摄像头流媒体服务
 * - 启动后立即提供视频流
 * - 通过http://摄像头IP/访问视频流
=======
 * ESP32-S3摄像头智能船导航系统 - 主程序
 * 
 * 功能:
 * - 初始化摄像头、WiFi、MQTT、电机、导航系统
 * - 启动视频流服务器
 * - 创建后台任务 (串口监控、系统监控)
 * - 主循环处理摄像头帧、MQTT消息和导航逻辑
>>>>>>> b6f6602 (feat: Implement camera setup, motor control, and MQTT management)
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
<<<<<<< HEAD

// 选择摄像头型号
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
#include "camera_pins.h"

// 固定IP设置
#define USE_FIXED_IP true
IPAddress staticIP(192, 168, 31, 170);  // 固定IP地址
IPAddress gateway(192, 168, 31, 1);     // 网关，根据您的路由器设置调整
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

// 函数声明
void setupLedFlash(int pin);

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
    case FRAMESIZE_SVGA: Serial.println("当前分辨率: 800x600 (SVGA)"); break;
    case FRAMESIZE_XGA: Serial.println("当前分辨率: 1024x768 (XGA)"); break;
    case FRAMESIZE_SXGA: Serial.println("当前分辨率: 1280x1024 (SXGA)"); break;
    case FRAMESIZE_UXGA: Serial.println("当前分辨率: 1600x1200 (UXGA)"); break;
    case FRAMESIZE_QXGA: Serial.println("当前分辨率: 2048x1536 (QXGA)"); break;
    case FRAMESIZE_HQVGA: Serial.println("当前分辨率: 240x176 (HQVGA)"); break;
    case FRAMESIZE_HD: Serial.println("当前分辨率: 1280x720 (HD)"); break;
    case FRAMESIZE_FHD: Serial.println("当前分辨率: 1920x1080 (FHD)"); break;
    case FRAMESIZE_QHD: Serial.println("当前分辨率: 2560x1440 (QHD)"); break;
    case FRAMESIZE_QSXGA: Serial.println("当前分辨率: 2560x1920 (QSXGA)"); break;
    default: Serial.printf("当前分辨率: 未知 (%d)\n", s->status.framesize);
  }
  
  Serial.printf("JPEG品质: %d\n", s->status.quality);
  Serial.printf("亮度: %d\n", s->status.brightness);
  Serial.printf("对比度: %d\n", s->status.contrast);
  Serial.printf("饱和度: %d\n", s->status.saturation);
  Serial.printf("特殊效果: %d\n", s->status.special_effect);
  Serial.printf("垂直翻转: %s\n", s->status.vflip ? "是" : "否");
  Serial.printf("水平镜像: %s\n", s->status.hmirror ? "是" : "否");
  Serial.printf("自动白平衡: %s\n", s->status.awb ? "开启" : "关闭");
  Serial.printf("自动增益: %s\n", s->status.agc ? "开启" : "关闭");
  Serial.printf("自动曝光: %s\n", s->status.aec ? "开启" : "关闭");
  Serial.println("=====================");
}

/**
 * 诊断摄像头性能状态
 */
void diagnoseCameraPerformance() {
  Serial.println("\n===== 摄像头性能诊断 =====");
  
  // 1. 内存使用情况
  Serial.printf("可用HEAP内存: %d字节\n", esp_get_free_heap_size());
  
  // 如果有PSRAM，检查PSRAM使用情况
  if(psramFound()) {
    Serial.printf("可用PSRAM内存: %d字节\n", ESP.getFreePsram());
    Serial.printf("PSRAM总大小: %d字节\n", ESP.getPsramSize());
    Serial.printf("PSRAM使用率: %.1f%%\n", 100.0f * (ESP.getPsramSize() - ESP.getFreePsram()) / ESP.getPsramSize());
  }
  
  // 2. CPU负载
  UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("当前任务剩余堆栈: %d字节\n", stackHighWaterMark * 4); // 4字节为单位
  
  // 3. 网络状态
  Serial.printf("WiFi信号强度(RSSI): %d dBm\n", WiFi.RSSI());
  Serial.printf("当前WiFi通道: %d\n", WiFi.channel());
  Serial.printf("WiFi传输功率: %d dBm\n", WiFi.getTxPower());
  
  // 4. 帧率测量
  unsigned long currentTime = millis();
  // 每秒更新一次FPS计算
  if (currentTime - lastFPSCalculationTime >= 1000) {
    currentFPS = frameCount * 1000.0 / (currentTime - lastFPSCalculationTime);
    frameCount = 0;
    lastFPSCalculationTime = currentTime;
  }
  Serial.printf("当前帧率: %.2f FPS\n", currentFPS);
  
  // 5. 摄像头参数测试
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    framesize_t currentSize = s->status.framesize;
    int quality = s->status.quality;
    Serial.printf("帧大小: %d，品质: %d\n", currentSize, quality);
    
    int clockSpeed = ESP.getCpuFreqMHz();
    Serial.printf("CPU时钟频率: %d MHz\n", clockSpeed);
  }
  Serial.println("============================");
  
  // 增加帧计数
  frameCount++;
=======
#include "freertos/semphr.h"
#include "esp_http_server.h" // 添加HTTP服务器头文件

// --- 包含新模块的头文件 ---
#include "camera_setup.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "motor_control.h"
#include "navigation.h"
#include "tasks.h" // 包含任务创建函数和外部声明

// --- 声明外部函数和变量 (来自其他未模块化的文件) ---
extern "C" { // 如果find_brightest.cpp是C代码编译的，需要这个
    #include "find_brightest.h" // 包含find_brightest函数声明
}
// extern void find_brightest(const uint8_t* gray, int width, int height, int& out_x, int& out_y); // 或者直接声明
extern void startSimpleCameraStream(); // 来自 stream.cpp
extern httpd_handle_t stream_httpd;    // 来自 stream.cpp
extern float currentFPS;               // 来自 stream.cpp (或由tasks.h提供)
extern unsigned long frameCount;       // 来自 stream.cpp
extern unsigned long lastFPSCalculationTime; // 来自 stream.cpp

// --- 全局共享变量和同步原语 ---
SemaphoreHandle_t frameAccessMutex = NULL;
volatile int sharedBrightX = -1; // 初始化为无效值
volatile int sharedBrightY = -1;
volatile bool newBrightPointAvailable = false;
volatile bool systemIsBusy = false;      // 系统繁忙标志 (由systemMonitorTask更新)

// --- 帧处理函数 ---
// 在每个帧处理时手动调用的函数 (可以考虑移入camera_processing模块)
void processFrame(camera_fb_t *fb) {
  static uint32_t last_frame_time = 0;
  uint32_t now = millis();
  
  // 降低处理频率，例如每200ms处理一次最亮点查找
  if (now - last_frame_time > 200) {
    last_frame_time = now;
    
    // 尝试获取互斥锁，超时时间短以避免阻塞视频流
    if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (fb && fb->format == PIXFORMAT_GRAYSCALE) {
        int bx, by;
        // 调用最亮点查找函数 (假设已正确包含或声明)
        find_brightest(fb->buf, fb->width, fb->height, bx, by);
        
        // 更新共享变量
        sharedBrightX = bx;
        sharedBrightY = by;
        newBrightPointAvailable = true; // 标记有新数据
      }
      xSemaphoreGive(frameAccessMutex); // 释放锁
    } else {
        // Serial.println("processFrame: 未能获取锁"); // 调试信息
    }
  }
>>>>>>> b6f6602 (feat: Implement camera setup, motor control, and MQTT management)
}


// --- Arduino Setup ---
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n====================================");
  Serial.println("      智能船导航系统启动中...     ");
  Serial.println("====================================");

<<<<<<< HEAD
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
  config.xclk_freq_hz = 20000000;  // 20MHz时钟频率
  config.frame_size = FRAMESIZE_QVGA;  // QVGA分辨率(320x240)
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // 改为灰度图格式
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;
  config.fb_count = 2;  // 增加帧缓冲数量以提高流畅度

  // PSRAM处理
  if (psramFound()) {
    config.jpeg_quality = 15;
    config.fb_count = 3;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  // OV3660传感器特殊配置
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  // 恢复摄像头默认增益设置，开启自动增益控制、自动曝光和自动白平衡
  s->set_gain_ctrl(s, 0);       // 开启自动增益
  s->set_exposure_ctrl(s, 0);   // 开启自动曝光
  s->set_agc_gain(s, 0);        // 设置自动增益
  s->set_awb_gain(s, 0);        // 开启自动白平衡增益
  s->set_aec_value(s, 100);     // 设置固定曝光值
  s->set_ae_level(s, 0);        // 设置AE级别为0
  s->set_whitebal(s, 1);        // 开启白平衡

  // 确保设置为QVGA
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // 尝试连接多个WiFi
  bool connected = false;
  for (int i = 0; i < numWifiOptions; i++) {
    Serial.printf("尝试连接WiFi: %s\n", wifiCredentials[i].ssid);
    
    WiFi.begin(wifiCredentials[i].ssid, wifiCredentials[i].password);
    WiFi.setSleep(false);
    
    // 尝试连接当前WiFi，最多等待3秒
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts <12) {
      delay(250);
      Serial.print(".");
      attempts++;
    } 
    
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.println("");
      Serial.printf("成功连接到 WiFi: %s\n", wifiCredentials[i].ssid);
      
      // 设置固定IP
      if (USE_FIXED_IP) {
        if (!WiFi.config(staticIP, gateway, subnet, dns)) {
          Serial.println("固定IP配置失败");
        } else {
          Serial.printf("固定IP地址: %s\n", staticIP.toString().c_str());
        }
      }
      
      break;
    } else {
      Serial.println("");
      Serial.printf("无法连接到 WiFi: %s\n", wifiCredentials[i].ssid);
    }
  }
  
  if (!connected) {
    Serial.println("无法连接任何WiFi网络，重启设备...");
    ESP.restart();
  }

  // 打印相机当前配置
  printCameraSettings();
  
  // 初始化性能监控变量
  lastFPSCalculationTime = millis();
  frameCount = 0;

  // 启动新的简化视频流服务器
  startSimpleCameraStream();

  Serial.println("====================================");
  Serial.println("       ESP32-CAM 视频流就绪!        ");
  Serial.println("====================================");
  Serial.print("打开浏览器访问 http://");
  Serial.print(WiFi.localIP());
  Serial.println(" 即可查看视频流");
  Serial.println("====================================");
}

void loop() {
  // 简化的监控输出，仅每10秒输出一次
  static unsigned long lastDiagTime = 0;
  static unsigned long lastBrightTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDiagTime >= 10000) {  // 每10秒
    Serial.printf("摄像头运行中，当前帧率: %.2f FPS\n", currentFPS);
    Serial.printf("当前IP: %s\n", WiFi.localIP().toString().c_str());
    lastDiagTime = currentTime;
  }

  // 每隔0.3秒输出一次最亮点ASCII图
  if (currentTime - lastBrightTime >= 300) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb && fb->format == PIXFORMAT_GRAYSCALE) {
      int bx, by;
      find_brightest(fb->buf, fb->width, fb->height, bx, by);
      Serial.printf("[最亮点] x=%d, y=%d\n", bx, by);
      print_ascii_frame(fb->width < 32 ? fb->width : 32, fb->height < 16 ? fb->height : 16, bx * (fb->width < 32 ? 1 : 32 / fb->width), by * (fb->height < 16 ? 1 : 16 / fb->height));
=======
  // 1. 创建互斥锁 (必须在任务使用前创建)
  frameAccessMutex = xSemaphoreCreateMutex();
  if (frameAccessMutex == NULL) {
    Serial.println("错误: 无法创建帧访问互斥锁! 重启...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("帧访问互斥锁已创建");

  // 2. 初始化摄像头
  if (!setupCamera()) {
    Serial.println("摄像头初始化失败，重启...");
    delay(1000);
    ESP.restart();
  }
  printCameraSettings(); // 打印摄像头配置

  // 3. 初始化电机
  setup_motors();

  // 4. 连接WiFi
  if (!connectWiFi()) {
    Serial.println("WiFi连接失败，重启...");
    delay(1000);
    ESP.restart();
  }

  // 5. 初始化MQTT客户端
  setupMQTT();
  // 首次尝试连接MQTT (如果WiFi连接成功)
  if (WiFi.status() == WL_CONNECTED) {
      mqtt_reconnect(); 
  }

  // 6. 初始化导航系统
  initNavigation();

  // 7. 初始化性能监控变量 (移到stream.cpp或全局?)
  lastFPSCalculationTime = esp_timer_get_time();
  frameCount = 0;

  // 8. 启动视频流服务器 (假设在核心1运行)
  startSimpleCameraStream(); 
  Serial.println("视频流服务器已启动");

  // 9. 创建后台任务 (固定到核心0以减少对摄像头/流的影响)
  createSerialMonitorTask(0); 
  createSystemMonitorTask(0);

  Serial.println("====================================");
  Serial.println("       系统初始化完成!          ");
  if (WiFi.status() == WL_CONNECTED) {
      Serial.print("访问: http://");
      Serial.print(WiFi.localIP());
      Serial.println(" 查看视频流");
  }
  Serial.println("====================================");
}

// --- Arduino Loop ---
void loop() {
  static unsigned long lastMqttReconnectAttempt = 0;
  static unsigned long lastFrameProcessTime = 0;
  static unsigned long lastNavigationUpdateTime = 0;
  unsigned long currentTime = millis();

  // 1. 处理MQTT连接和消息
  if (!mqttClient.connected()) {
    // 每隔5秒尝试重连MQTT
    if (currentTime - lastMqttReconnectAttempt > 5000) {
      lastMqttReconnectAttempt = currentTime;
      Serial.println("MQTT断开连接，尝试重连...");
      mqtt_reconnect();
    }
  } else {
    mqttClient.loop(); // 处理传入消息和保持连接
  }

  // 2. 获取并处理摄像头帧 (如果系统不繁忙)
  // 增加帧处理间隔，例如100ms获取一次图像
  if (!systemIsBusy && (currentTime - lastFrameProcessTime >= 100)) { 
    lastFrameProcessTime = currentTime;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // a. 处理帧数据 (例如，查找最亮点)
      processFrame(fb); 
      
      // b. 安全地获取最新的最亮点数据 (用于导航)
      int currentBrightX = -1, currentBrightY = -1;
      if (xSemaphoreTake(frameAccessMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        // 读取共享变量，即使newBrightPointAvailable是false也读取当前值
        currentBrightX = sharedBrightX;
        currentBrightY = sharedBrightY;
        // 注意：导航逻辑可能需要知道数据是否是"新"的，
        // 但为了简化，这里直接读取最新值。
        // 如果需要区分新旧，可以在获取锁后检查 newBrightPointAvailable
        xSemaphoreGive(frameAccessMutex);
      } else {
         // Serial.println("Loop: 未能获取锁读取最亮点"); // 调试信息
      }

      // c. 更新导航状态机 (增加导航更新间隔，例如300ms)
      if (currentTime - lastNavigationUpdateTime >= 300) {
          lastNavigationUpdateTime = currentTime;
          // 将当前帧和获取到的最亮点信息传递给导航状态机
          navigationStateMachine(fb, currentBrightX, currentBrightY); 
      }
      
      // d. 处理完毕，返回帧缓冲区
>>>>>>> b6f6602 (feat: Implement camera setup, motor control, and MQTT management)
      esp_camera_fb_return(fb);
    } else {
      // Serial.println("未能获取摄像头帧"); // 调试信息
    }
<<<<<<< HEAD
    lastBrightTime = currentTime;
  }
  delay(10);  // 短暂延时防止CPU过载
=======
  } else if (systemIsBusy) {
      // Serial.println("系统繁忙，跳过帧处理"); // 调试信息
      // 当系统繁忙时，可以考虑降低电机速度或执行安全操作
      // motor_control(0, 0);
      // motor_control(1, 0);
  }

  // 3. 短暂延时，让其他任务有机会运行
  // vTaskDelay(pdMS_TO_TICKS(10)); // 使用FreeRTOS延时更佳
  delay(10); // 简单延时
>>>>>>> b6f6602 (feat: Implement camera setup, motor control, and MQTT management)
}
