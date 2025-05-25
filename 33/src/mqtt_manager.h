#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <PubSubClient.h>
#include <WiFi.h>

extern WiFiClient espClient;
extern PubSubClient mqttClient;

// MQTT管理模块接口
void setupMQTT();
bool mqtt_reconnect();
void loopMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);

#endif // MQTT_MANAGER_H
