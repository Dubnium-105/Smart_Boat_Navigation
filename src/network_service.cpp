#include "network_service.h"

#include "cellular_manager.h"
#if !NETWORK_4G_MODULE_TEST
#include "wifi_manager.h"
#endif

namespace {

NetworkServiceStatus g_status = {false, false, ActiveNetworkType::None, 0};
unsigned long g_last4gStatusLogMs = 0;

void setActiveNetwork(ActiveNetworkType type) {
  if (g_status.activeNetwork == type) {
    return;
  }
  g_status.activeNetwork = type;
  g_status.switchedAtMs = millis();
  Serial.printf("[NetworkService] active network switched to %s\n", activeNetworkName(type));
}

void refreshStatusFromBackends() {
#if NETWORK_4G_MODULE_TEST
  g_status.wifiConnected = false;
  g_status.cellularConnected = isCellularConnected();

  if (g_status.cellularConnected) {
    setActiveNetwork(ActiveNetworkType::Cellular4G);
  } else {
    setActiveNetwork(ActiveNetworkType::None);
  }
#else
  g_status.wifiConnected = (WiFi.status() == WL_CONNECTED);
  g_status.cellularConnected = isCellularConnected();

  if (g_status.wifiConnected) {
    setActiveNetwork(ActiveNetworkType::WiFi);
  } else if (g_status.cellularConnected) {
    setActiveNetwork(ActiveNetworkType::Cellular4G);
  } else {
    setActiveNetwork(ActiveNetworkType::None);
  }
#endif
}

}  // namespace

const char* activeNetworkName(ActiveNetworkType type) {
  switch (type) {
    case ActiveNetworkType::WiFi:
      return "WiFi";
    case ActiveNetworkType::Cellular4G:
      return "4G";
    case ActiveNetworkType::None:
    default:
      return "None";
  }
}

void setupNetworkService() {
  Serial.println("[NetworkService] setup start");
  setupCellular();
  refreshStatusFromBackends();
#if NETWORK_4G_MODULE_TEST
  Serial.println("[NetworkService] 4G module test mode enabled");
#endif
  Serial.println("[NetworkService] setup done");
}

bool connectNetworkService() {
#if NETWORK_4G_MODULE_TEST
  Serial.println("[NetworkService] connecting (4G module test only)...");
  bool cellOk = connectCellular();
  refreshStatusFromBackends();
  return cellOk;
#else
  Serial.println("[NetworkService] connecting (WiFi first, 4G fallback)...");
  bool wifiOk = connectWiFi();
  if (wifiOk) {
    refreshStatusFromBackends();
    return true;
  }

  bool cellOk = connectCellular();
  refreshStatusFromBackends();
  return cellOk;
#endif
}

void networkServiceLoop() {
#if NETWORK_4G_MODULE_TEST
  cellularAutoReconnect();
  refreshStatusFromBackends();

  unsigned long now = millis();
  if (now - g_last4gStatusLogMs >= 3000) {
    g_last4gStatusLogMs = now;
    CellularStatus st = getCellularStatus();
    Serial.printf("[4G-TEST] ready=%d reg=%d attach=%d rssi=%d oper=%s\n",
                  st.moduleReady ? 1 : 0, st.registered ? 1 : 0,
                  st.dataAttached ? 1 : 0, st.rssi, st.operatorName.c_str());
  }
#else
  bool wifiOk = wifiAutoReconnect();
  if (!wifiOk) {
    cellularAutoReconnect();
  }
  refreshStatusFromBackends();
#endif
}

bool isNetworkServiceReady() {
  refreshStatusFromBackends();
  return g_status.wifiConnected || g_status.cellularConnected;
}

bool isMqttPathReady() {
  refreshStatusFromBackends();
#if NETWORK_4G_MODULE_TEST
  return false;
#else
  return g_status.wifiConnected;
#endif
}

NetworkServiceStatus getNetworkServiceStatus() {
  refreshStatusFromBackends();
  return g_status;
}