# ESP32-CAM 摄像头例程

本项目基于ESP32-CAM模块实现视频流传输及性能监控功能，支持多WiFi网络自动连接，并提供详细的摄像头配置与性能诊断信息。

## 功能特性
- 多WiFi网络自动连接（支持优先级尝试）
- 实时摄像头配置查看（分辨率、画质参数等）
- 系统性能监控（内存、CPU、帧率、网络状态）
- 自适应PSRAM配置（自动调整分辨率与画质）
- Web视频流服务（通过浏览器访问实时画面）

## 硬件要求
- ESP32-CAM模块（推荐ESP32-S3-EYE型号）
- PSRAM芯片（必须安装以支持UXGA分辨率）
- OV2640/OV3660摄像头模组
- USB转TTL串口模块（用于烧录和调试）
- 确保开发板分区方案至少保留3MB APP空间

## 开发环境配置
1. 安装Arduino IDE（1.8.13+）
2. 添加ESP32开发板支持：
   - 文件 > 首选项 > 附加开发板管理器URL添加：
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
3. 安装依赖库：
   - esp32-camera (通过库管理器安装)
   - WiFi (内置)

## 快速开始
1. 修改WiFi配置
   在代码中修改`wifiCredentials`数组，添加您的网络信息：
   ```cpp
   const WifiCredential wifiCredentials[] = {
     {"Your_SSID_1", "Your_PASSWORD_1"},
     {"Your_SSID_2", "Your_PASSWORD_2"}
   };

2.选择摄像头型号
取消注释对应的摄像头型号定义：


//#define CAMERA_MODEL_ESP32S3_EYE  // 根据实际型号选择

烧录程序

分区方案：Huge APP (3MB No OTA/1MB SPIFFS)

波特率设置为115200

使用说明
连接串口监视器查看输出

设备启动后将自动：

尝试连接预配置的WiFi网络

初始化摄像头（自动检测PSRAM）

启动Web视频流服务器

访问视频流：

复制
http://[设备IP地址]/stream
