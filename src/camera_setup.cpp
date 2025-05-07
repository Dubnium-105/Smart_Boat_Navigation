#include "camera_setup.h"
#include "esp_camera_pins.h" // 包含摄像头引脚定义
#include <Arduino.h>

bool setupCamera() {
  Serial.println("开始初始化摄像头...");
  
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
  config.frame_size = FRAMESIZE_QQVGA;  // 160x120 分辨率（QQVGA，已确认调整）
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // 灰度图格式
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  // 帧缓冲区空闲时捕获，更稳定
  config.fb_location = CAMERA_FB_IN_PSRAM;    // 优先使用PSRAM存储帧
  config.jpeg_quality = 12;  // JPEG质量 (影响灰度图处理吗？可能不影响，但保留)
  config.fb_count = 1;       // 使用单个帧缓冲，减少内存占用和延迟

  // PSRAM检测及配置
  if (!psramFound()) {
    Serial.println("警告: 未找到PSRAM，功能可能受限。将使用DRAM存储帧。");
    config.frame_size = FRAMESIZE_QQVGA; // 确保在无PSRAM时使用较小分辨率
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1; // DRAM有限，只用一个buffer
  } else {
     Serial.println("找到PSRAM，已启用。");
     // 如果有PSRAM，可以考虑使用更大的fb_count=2以提高帧率，但需测试稳定性
     // config.fb_count = 2; 
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败，错误代码: 0x%x\n", err);
    return false;
  }
  Serial.println("摄像头初始化成功");

  // 获取并配置摄像头传感器
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("获取摄像头传感器失败!");
    // 即使初始化成功，获取传感器失败也应视为失败
    esp_camera_deinit(); // 清理已初始化的部分
    return false;
  }

  // 应用优化的摄像头参数设置
  s->set_brightness(s, 0);      // 标准亮度
  s->set_contrast(s, 0);        // 标准对比度
  s->set_saturation(s, 0);      // 标准饱和度 (灰度图可能无效)
  s->set_sharpness(s, 0);       // 标准锐度
  s->set_denoise(s, 1);         // 开启降噪
  s->set_quality(s, 10);        // 图像质量 (影响JPEG压缩，对灰度图处理影响较小)
  s->set_special_effect(s, 0);  // 无特殊效果
  s->set_vflip(s, 1);           // 垂直翻转图像
  s->set_hmirror(s, 0);         // 不水平镜像
  
  // 自动控制设置
  s->set_gain_ctrl(s, 1);       // 开启自动增益控制 (AGC)
  s->set_exposure_ctrl(s, 1);   // 开启自动曝光控制 (AEC)
  s->set_awb_gain(s, 1);        // 开启自动白平衡增益 (AWB Gain)
  // s->set_agc_gain(s, 0);     // 可以设置初始增益，但自动控制会覆盖它
  // s->set_aec_value(s, ...)  // 可以设置初始曝光值，但自动控制会覆盖它
  s->set_lenc(s, 1);            // 开启镜头校正

  // 再次确认帧大小设置成功
  if (s->status.framesize != FRAMESIZE_QQVGA) {
      Serial.printf("警告: 帧大小未能成功设置为QQVGA（160x120），当前为 %d。尝试重新设置...\n", s->status.framesize);
      s->set_framesize(s, FRAMESIZE_QQVGA);
      delay(100); // 等待设置生效
      if (s->status.framesize != FRAMESIZE_QQVGA) {
          Serial.println("错误: 无法将帧大小设置为QQVGA。");
          // 可以选择返回false或继续，取决于应用要求
      }
  }
  
  Serial.println("摄像头传感器参数配置完成。");
  delay(300); // 等待摄像头稳定
  return true;
}

void printCameraSettings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("无法获取传感器信息以打印设置。");
    return;
  }
  
  Serial.println("\n==== 当前相机配置信息 ====");
  
  // 打印帧大小
  const char* framesize_str;
  switch(s->status.framesize) {
    case FRAMESIZE_QQVGA: framesize_str = "160x120 (QQVGA)"; break;
    case FRAMESIZE_QVGA:  framesize_str = "320x240 (QVGA)"; break;
    case FRAMESIZE_VGA:   framesize_str = "640x480 (VGA)"; break;
    // 添加其他支持的分辨率
    default: framesize_str = "其他"; break;
  }
  Serial.printf("分辨率: %s (%d)\n", framesize_str, s->status.framesize);
  
  // 打印像素格式
  const char* pixformat_str;
   switch(s->pixformat) {
      case PIXFORMAT_GRAYSCALE: pixformat_str = "GRAYSCALE"; break;
      case PIXFORMAT_RGB565:    pixformat_str = "RGB565"; break;
      case PIXFORMAT_JPEG:      pixformat_str = "JPEG"; break;
      // 添加其他支持的格式
      default: pixformat_str = "未知"; break;
   }
  Serial.printf("像素格式: %s (%d)\n", pixformat_str, s->pixformat);

  Serial.printf("JPEG品质: %d\n", s->status.quality);
  Serial.printf("亮度: %d\n", s->status.brightness);
  Serial.printf("对比度: %d\n", s->status.contrast);
  Serial.printf("饱和度: %d\n", s->status.saturation);
  Serial.printf("锐度: %d\n", s->status.sharpness);
  Serial.printf("降噪: %s\n", s->status.denoise ? "开启" : "关闭");
  Serial.printf("垂直翻转: %s\n", s->status.vflip ? "是" : "否");
  Serial.printf("水平镜像: %s\n", s->status.hmirror ? "是" : "否");
  Serial.printf("自动白平衡(AWB): %s\n", s->status.awb ? "开启" : "关闭");
  Serial.printf("自动白平衡增益(AWB Gain): %s\n", s->status.awb_gain ? "开启" : "关闭");
  Serial.printf("自动增益控制(AGC): %s\n", s->status.agc ? "开启" : "关闭");
  Serial.printf("自动曝光控制(AEC): %s\n", s->status.aec ? "开启" : "关闭");
  Serial.printf("镜头校正(LENC): %s\n", s->status.lenc ? "开启" : "关闭");
  Serial.println("==========================");
}