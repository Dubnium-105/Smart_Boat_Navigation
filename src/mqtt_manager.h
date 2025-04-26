#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <PubSubClient.h>
#include <WiFi.h>

extern WiFiClient espClient;
extern PubSubClient mqttClient;

/**
 * @brief 设置MQTT客户端（服务器、端口、回调函数、缓冲区大小）。
 */
void setupMQTT();

/**
 * @brief 尝试重新连接到MQTT服务器并订阅主题。
 * 
 * @return true 如果成功连接或已连接。
 * @return false 如果多次尝试后连接失败。
 */
bool mqtt_reconnect();

/**
 * @brief 处理MQTT客户端的循环，保持连接并处理传入消息。
 */
void loopMQTT();

/**
 * @brief MQTT消息回调函数，处理接收到的MQTT消息。
 *        目前支持通过 /motor 主题控制电机。
 * 
 * @param topic 接收到的消息主题。
 * @param payload 消息内容。
 * @param length 消息长度。
 */
void mqtt_callback(char* topic, byte* payload, unsigned int length);


#endif // MQTT_MANAGER_H