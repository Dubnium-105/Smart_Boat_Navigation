#ifndef NETWORK_SERVICE_H
#define NETWORK_SERVICE_H

#include <Arduino.h>

enum class ActiveNetworkType {
  None = 0,
  WiFi = 1,
  Cellular4G = 2,
};

struct NetworkServiceStatus {
  bool wifiConnected;
  bool cellularConnected;
  ActiveNetworkType activeNetwork;
  unsigned long switchedAtMs;
};

void setupNetworkService();
bool connectNetworkService();
void networkServiceLoop();
bool isNetworkServiceReady();
bool isMqttPathReady();
NetworkServiceStatus getNetworkServiceStatus();
const char* activeNetworkName(ActiveNetworkType type);

#endif  // NETWORK_SERVICE_H