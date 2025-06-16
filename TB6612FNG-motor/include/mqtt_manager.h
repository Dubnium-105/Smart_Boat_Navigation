#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <WiFi.h>
#include <PubSubClient.h>

// 函数声明
void setupMQTT();
bool mqtt_reconnect();
void loopMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publishSystemStatus();

// 外部变量声明
extern PubSubClient mqttClient;

#endif
