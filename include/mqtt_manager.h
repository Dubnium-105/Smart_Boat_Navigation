#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>


#include <PubSubClient.h>
#include <WiFi.h>

extern WiFiClient espClient;
extern PubSubClient mqttClient;
void setupMQTT();
bool mqtt_reconnect();
void loopMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
#endif // MQTT_MANAGER_H