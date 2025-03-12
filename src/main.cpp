#include "esp_camera.h"
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT  // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
// 定义WiFi凭据结构体
struct WifiCredential {
  const char* ssid;
  const char* password;
};

// 添加多个WiFi候选
const WifiCredential wifiCredentials[] = {
  {"xbox", "12345678"},
  {"room@407", "room@407"},
  {"xbox", "z1139827642"}
};
const int numWifiOptions = sizeof(wifiCredentials) / sizeof(wifiCredentials[0]);

void startCameraServer();
void setupLedFlash(int pin);

// 添加打印相机配置函数
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

// 添加性能监控全局变量
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;
float currentFPS = 0;
unsigned long lastFPSCalculationTime = 0;

// 诊断摄像头性能瓶颈
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
  // 注意：ESP32没有简单的API获取CPU使用率，但通过下面的方法可以大致了解
  UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("当前任务剩余堆栈: %d字节\n", stackHighWaterMark * 4); // 4字节为单位
  
  // 3. 网络状态
  Serial.printf("WiFi信号强度(RSSI): %d dBm\n", WiFi.RSSI());
  Serial.printf("当前WiFi通道: %d\n", WiFi.channel());
  Serial.printf("网络MTU: %d字节\n", WiFi.getTxPower());
  
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
    Serial.printf("帧大小: %d，品质: %d - 此配置是否超出硬件能力？\n", 
                  currentSize, quality);
    
    int clockSpeed = ESP.getCpuFreqMHz();
    Serial.printf("CPU时钟频率: %d MHz\n", clockSpeed);
  }
  Serial.println("============================");
  
  // 增加帧计数
  frameCount++;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 15;
      config.fb_count = 3;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 4;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_HQVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
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
  lastFrameTime = millis();
  lastFPSCalculationTime = lastFrameTime;
  frameCount = 0;

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // 每4秒输出一次相机配置和性能诊断信息
  printCameraSettings();
  diagnoseCameraPerformance();
  delay(4000);
}
