#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

// 摄像头功能已移除：头文件只保留最小声明，避免项目中残留对 esp_camera 的直接依赖

bool setupCamera();
void printCameraSettings();

#endif // CAMERA_SETUP_H