#include <Arduino.h>
#include <esp_camera.h>
#include "esp_camera_pins.h"
#include "mqtt_manager.h"
#include "camera_module.h"

bool setup_camera() {
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
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QQVGA; // 降低分辨率为QQVGA(160x120)
    config.pixel_format = PIXFORMAT_JPEG; // 直接输出JPEG格式
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 35; // 提高压缩率，数值越大图片越小，建议30~40
    config.fb_count = 1;

    if (!psramFound()) {
        Serial.println("警告: 未找到PSRAM，功能可能受限。将使用DRAM存储帧。");
        config.frame_size = FRAMESIZE_QQVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
    } else {
        Serial.println("找到PSRAM，已启用。");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("摄像头初始化失败，错误代码: 0x%x\n", err);
        return false;
    }
    Serial.println("摄像头初始化成功");

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("获取摄像头传感器失败!");
        esp_camera_deinit();
        return false;
    }

    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 1);
    s->set_denoise(s, 1);
    s->set_special_effect(s, 0);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_awb_gain(s, 1);
    s->set_lenc(s, 1);

    if (s->status.framesize != FRAMESIZE_QQVGA) {
        Serial.printf("警告: 帧大小未能成功设置为QQVGA（160x120），当前为 %d。尝试重新设置...\n", s->status.framesize);
        s->set_framesize(s, FRAMESIZE_QQVGA);
        delay(100);
        if (s->status.framesize != FRAMESIZE_QQVGA) {
            Serial.println("错误: 无法将帧大小设置为QQVGA。");
        }
    }

    Serial.println("摄像头传感器参数配置完成。");
    delay(300);
    return true;
}

// 新增：采集控制标志
static volatile bool camera_capture_enabled = false;

// 提供外部接口控制采集开关
void set_camera_capture_enabled(bool enabled) {
    camera_capture_enabled = enabled;
    Serial.printf("摄像头采集已%s\n", enabled ? "开启" : "关闭");
}

bool get_camera_capture_enabled() {
    return camera_capture_enabled;
}

// 修改采集函数：仅在采集开关为true时采集
void send_camera_frame_mqtt() {
    if (!camera_capture_enabled) return;
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        Serial.printf("采集到图像，长度: %d 字节\n", fb->len);
        bool publishSuccess = mqttClient.publish("/camera", fb->buf, fb->len);
        if (publishSuccess) {
            Serial.printf("图像已通过MQTT发布，大小: %d 字节\n", fb->len);
        } else {
            Serial.println("图像发布失败（publish返回false），请检查MQTT服务器消息体限制、网络或主题配置。");
        }
        esp_camera_fb_return(fb);
    } else {
        Serial.println("采集摄像头帧失败！");
    }
}
