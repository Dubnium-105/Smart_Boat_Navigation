/**
 * ESP32-S3摄像头智能船导航系统 - 精简初始化主程序
 * 
 * 只实现：摄像头、WiFi、MQTT、电机等硬件和通信模块的初始化
 */

#include <Arduino.h>
#include "motor_control.h"
#include "wifi_manager.h"
#include "camera_module.h"
#include "mqtt_manager.h"
#include <esp_camera.h>

// ========== 初始化 ==========
void setup() {
    Serial.begin(115200);
    Serial.println("系统启动，初始化硬件...");

    // 初始化电机
    setup_motors();

    // 连接WiFi
    Serial.println("正在连接WiFi...");
    if (!connectWiFi()) {
        Serial.println("WiFi连接失败，重启设备");
        ESP.restart();
    } else {
        Serial.print("WiFi连接成功，IP地址: ");
        Serial.println(WiFi.localIP());
    }

    // 初始化摄像头
    Serial.println("正在初始化摄像头...");
    setup_camera();
    Serial.println("摄像头初始化完成");

    // 初始化MQTT
    Serial.println("正在连接MQTT服务器...");
    setupMQTT();
    if (mqttClient.connected()) {
        Serial.println("MQTT服务器连接成功");
    } else {
        Serial.println("MQTT服务器未连接，等待自动重连...");
    }

    Serial.println("初始化完成。");
}

// ========== 主循环 ==========
void loop() {
    // WiFi自动重连
    if (!wifiAutoReconnect()) {
        static bool wasConnected = true;
        if (wasConnected) {
            Serial.println("WiFi断开，正在重连...");
            wasConnected = false;
        }
        return;
    } else {
        static bool wasConnected = false;
        if (!wasConnected) {
            Serial.print("WiFi已恢复，IP地址: ");
            Serial.println(WiFi.localIP());
            wasConnected = true;
        }
    }

    // 处理MQTT消息前，实时反馈MQTT服务器连接状态
    static bool mqttWasConnected = false;
    if (mqttClient.connected()) {
        if (!mqttWasConnected) {
            Serial.println("MQTT服务器已连接");
            mqttWasConnected = true;
        }
    } else {
        if (mqttWasConnected) {
            Serial.println("MQTT服务器断开，正在重连...");
            mqttWasConnected = false;
        }
        mqtt_reconnect(); // 自动重连
    }

    // 处理MQTT消息
    loopMQTT();

    // 采集图像并通过MQTT实时推送
    send_camera_frame_mqtt();

    // 采集图像并通过MQTT发布（可选，需在mqtt_manager中实现相关接口）
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        // 这里可以发布图像到MQTT，或做其他处理
        // 例如: mqttClient.publish("/camera", fb->buf, fb->len);
        esp_camera_fb_return(fb);
    }

    // 默认不控制电机，等待MQTT指令
    // motor_control(0, 0); // 可选：如需每次循环都强制停止
    // motor_control(1, 0);
    delay(1000); // 控制帧率或循环间隔
}