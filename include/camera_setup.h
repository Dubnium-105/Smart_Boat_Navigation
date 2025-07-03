#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

#include "esp_camera.h"

extern camera_config_t camera_config_global;

/**
 * @brief 初始化ESP32摄像头。
 * 
 * @return true 如果初始化成功。
 * @return false 如果初始化失败。
 */
bool setupCamera();

/**
 * @brief 打印当前相机传感器的配置信息到串口。
 */
void printCameraSettings();

#endif // CAMERA_SETUP_H