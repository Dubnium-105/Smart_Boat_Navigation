#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

enum NAV_STATE {
  STATE_MANUAL = 0,
  STATE_NAVIGATING = 1
};

#include <PubSubClient.h>
#include <WiFi.h>

extern WiFiClient espClient;
extern PubSubClient mqttClient;

extern volatile int irNavState;
extern volatile int webRequestedNavState;
String get_ir_nav_mode_str();
void setNavMode(int mode);
void setupMQTT();
bool mqtt_reconnect();
void loopMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publishIRInfo(int sensorId, const char* direction, int angle); // 发送红外方向信息

#endif // MQTT_MANAGER_H