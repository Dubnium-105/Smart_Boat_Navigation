#ifndef CAMERA_MODULE_H
#define CAMERA_MODULE_H

#include <esp_camera.h>

// 初始化摄像头，返回 true 表示成功，false 表示失败
bool setup_camera();

// 新增：摄像头帧通过MQTT实时推送
void send_camera_frame_mqtt();

// 新增：采集开关控制接口
void set_camera_capture_enabled(bool enabled);
bool get_camera_capture_enabled();

#endif // CAMERA_MODULE_H
