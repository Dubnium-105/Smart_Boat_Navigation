#include "safety_manager.h"

#include "motor_control.h"
#include "mqtt_manager.h"

#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <math.h>

#ifndef SAFETY_HEARTBEAT_TIMEOUT_MS
#define SAFETY_HEARTBEAT_TIMEOUT_MS 3000UL
#endif

#ifndef SAFETY_COMMAND_MIN_INTERVAL_MS
#define SAFETY_COMMAND_MIN_INTERVAL_MS 80UL
#endif

#ifndef SAFETY_LOW_BATTERY_VOLTAGE
#define SAFETY_LOW_BATTERY_VOLTAGE 6.80f
#endif

#ifndef SAFETY_GEOFENCE_RADIUS_M
#define SAFETY_GEOFENCE_RADIUS_M 120.0f
#endif

#ifndef SAFETY_DEFAULT_LATITUDE
#define SAFETY_DEFAULT_LATITUDE 0.0
#endif

#ifndef SAFETY_DEFAULT_LONGITUDE
#define SAFETY_DEFAULT_LONGITUDE 0.0
#endif

namespace {

struct CommandRateSlot {
  const char* key;
  unsigned long lastAcceptedMs;
};

SafetyStatus g_status;
bool g_geofenceEnabled = false;
bool g_homeConfigured = false;
bool g_rollbackAllowed = true;
double g_homeLat = SAFETY_DEFAULT_LATITUDE;
double g_homeLon = SAFETY_DEFAULT_LONGITUDE;
double g_fenceLat = SAFETY_DEFAULT_LATITUDE;
double g_fenceLon = SAFETY_DEFAULT_LONGITUDE;
float g_fenceRadiusM = SAFETY_GEOFENCE_RADIUS_M;

CommandRateSlot g_rateSlots[] = {
    {"heartbeat", 0},
    {"legacy_motor", 0},
    {"legacy_navigation", 0},
    {"legacy_ota", 0},
    {"legacy_restart", 0},
    {"legacy_shutdown", 0},
    {"motor", 0},
    {"nav_mode", 0},
    {"mission", 0},
    {"ota", 0},
    {"restart", 0},
    {"safety", 0},
};

const char* kCommandWhitelist[] = {
    "heartbeat",
    "legacy_motor",
    "legacy_navigation",
    "legacy_ota",
    "legacy_restart",
    "legacy_shutdown",
    "motor",
    "nav_mode",
    "mission",
    "ota",
    "restart",
    "safety",
};

bool isWhitelisted(const char* commandKey) {
  if (commandKey == nullptr || commandKey[0] == '\0') return false;
  for (size_t i = 0; i < sizeof(kCommandWhitelist) / sizeof(kCommandWhitelist[0]); ++i) {
    if (strcmp(commandKey, kCommandWhitelist[i]) == 0) return true;
  }
  return false;
}

CommandRateSlot* findRateSlot(const char* commandKey) {
  for (size_t i = 0; i < sizeof(g_rateSlots) / sizeof(g_rateSlots[0]); ++i) {
    if (strcmp(commandKey, g_rateSlots[i].key) == 0) return &g_rateSlots[i];
  }
  return nullptr;
}

double degToRad(double deg) { return deg * M_PI / 180.0; }

float distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double earthRadiusM = 6371000.0;
  double dLat = degToRad(lat2 - lat1);
  double dLon = degToRad(lon2 - lon1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(degToRad(lat1)) * cos(degToRad(lat2)) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return static_cast<float>(earthRadiusM * c);
}

bool hasValidPosition() {
  return !(fabs(g_status.latitude) < 0.000001 && fabs(g_status.longitude) < 0.000001);
}

void stopMotors(const char* reason) {
  extern int speedA;
  extern int speedB;
  speedA = 0;
  speedB = 0;
  motor_control(0, 0);
  motor_control(1, 0);
  g_status.failSafeActive = true;
  g_status.lastStopReason = reason != nullptr ? reason : "fail_safe";
  Serial.printf("[Safety] motors stopped: %s\n", g_status.lastStopReason.c_str());
}

void clearFailSafeIfHealthy() {
  if (g_status.heartbeatOk && g_status.geofenceOk && g_status.batteryOk && g_status.commandOk) {
    if (g_status.failSafeActive) Serial.println("[Safety] all guards healthy, fail-safe released");
    g_status.failSafeActive = false;
    g_status.returningHome = false;
    g_status.lastStopReason = "";
  }
}

uint8_t deviceRolloutBucket() {
  String id = get_device_id();
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < id.length(); ++i) {
    hash ^= static_cast<uint8_t>(id[i]);
    hash *= 16777619UL;
  }
  return static_cast<uint8_t>(hash % 100);
}

void evaluateHeartbeat(unsigned long now) {
  if (now - g_status.lastHeartbeatMs > SAFETY_HEARTBEAT_TIMEOUT_MS) {
    if (g_status.heartbeatOk) {
      g_status.heartbeatOk = false;
      stopMotors("heartbeat_timeout");
    }
  } else {
    g_status.heartbeatOk = true;
  }
}

void evaluateBattery() {
  if (g_status.batteryVoltage <= 0.1f) {
    g_status.batteryOk = true;
    return;
  }
  if (g_status.batteryVoltage < SAFETY_LOW_BATTERY_VOLTAGE) {
    g_status.batteryOk = false;
    g_status.returningHome = true;
    stopMotors("low_battery_return_home");
    return;
  }
  g_status.batteryOk = true;
}

void evaluateGeofence() {
  if (!g_geofenceEnabled || !hasValidPosition()) {
    g_status.geofenceOk = true;
    return;
  }
  float distance = distanceMeters(g_status.latitude, g_status.longitude, g_fenceLat, g_fenceLon);
  if (distance > g_fenceRadiusM) {
    g_status.geofenceOk = false;
    g_status.returningHome = true;
    stopMotors("geofence_violation");
    return;
  }
  g_status.geofenceOk = true;
}

}  // namespace

void setupSafetyManager() {
  memset(&g_status, 0, sizeof(g_status));
  g_status.heartbeatOk = true;
  g_status.geofenceOk = true;
  g_status.batteryOk = true;
  g_status.commandOk = true;
  g_status.failSafeActive = false;
  g_status.returningHome = false;
  g_status.batteryVoltage = 0.0f;
  g_status.latitude = 0.0;
  g_status.longitude = 0.0;
  g_status.lastHeartbeatMs = millis();
  g_status.lastCommandAcceptedMs = millis();
  g_status.lastStopReason = "";

  if (fabs(SAFETY_DEFAULT_LATITUDE) > 0.000001 || fabs(SAFETY_DEFAULT_LONGITUDE) > 0.000001) {
    safetySetHome(SAFETY_DEFAULT_LATITUDE, SAFETY_DEFAULT_LONGITUDE);
    safetySetGeoFence(SAFETY_DEFAULT_LATITUDE, SAFETY_DEFAULT_LONGITUDE, SAFETY_GEOFENCE_RADIUS_M);
    safetyEnableGeoFence(true);
  }

  Serial.printf("[Safety] initialized heartbeat_timeout=%lu ms, command_interval=%lu ms, low_battery=%.2f V\n",
                static_cast<unsigned long>(SAFETY_HEARTBEAT_TIMEOUT_MS),
                static_cast<unsigned long>(SAFETY_COMMAND_MIN_INTERVAL_MS),
                SAFETY_LOW_BATTERY_VOLTAGE);
}

void safetyLoop() {
  unsigned long now = millis();
  evaluateHeartbeat(now);
  evaluateBattery();
  evaluateGeofence();
  clearFailSafeIfHealthy();
}

void safetyRecordHeartbeat() {
  g_status.lastHeartbeatMs = millis();
  g_status.heartbeatOk = true;
}

bool safetyAllowsMotion() {
  safetyLoop();
  return !g_status.failSafeActive && g_status.heartbeatOk && g_status.geofenceOk && g_status.batteryOk;
}

bool safetyGuardMotorCommand(int& leftPwm, int& rightPwm, const char* source) {
  if (!safetyAllowsMotion()) {
    leftPwm = 0;
    rightPwm = 0;
    if (g_status.lastStopReason.isEmpty()) g_status.lastStopReason = source != nullptr ? source : "guarded_motor_command";
    return false;
  }
  leftPwm = constrain(leftPwm, -255, 255);
  rightPwm = constrain(rightPwm, -255, 255);
  return true;
}

bool safetyAcceptCommand(const char* commandKey, String* rejectReason) {
  if (!isWhitelisted(commandKey)) {
    g_status.commandOk = false;
    if (rejectReason != nullptr) *rejectReason = "command_not_whitelisted";
    Serial.printf("[Safety] rejected command: %s not in whitelist\n", commandKey != nullptr ? commandKey : "(null)");
    return false;
  }
  CommandRateSlot* slot = findRateSlot(commandKey);
  unsigned long now = millis();
  if (slot != nullptr && now - slot->lastAcceptedMs < SAFETY_COMMAND_MIN_INTERVAL_MS) {
    g_status.commandOk = false;
    if (rejectReason != nullptr) *rejectReason = "command_rate_limited";
    Serial.printf("[Safety] rejected command: %s rate limited\n", commandKey);
    return false;
  }
  if (slot != nullptr) slot->lastAcceptedMs = now;
  g_status.commandOk = true;
  g_status.lastCommandAcceptedMs = now;
  return true;
}

void safetyUpdateTelemetry(double latitude, double longitude, float batteryVoltage) {
  g_status.latitude = latitude;
  g_status.longitude = longitude;
  g_status.batteryVoltage = batteryVoltage;
}

void safetySetHome(double latitude, double longitude) {
  g_homeLat = latitude;
  g_homeLon = longitude;
  g_homeConfigured = true;
  Serial.printf("[Safety] home set lat=%.6f lon=%.6f\n", g_homeLat, g_homeLon);
}

void safetySetGeoFence(double centerLatitude, double centerLongitude, float radiusMeters) {
  g_fenceLat = centerLatitude;
  g_fenceLon = centerLongitude;
  g_fenceRadiusM = radiusMeters > 1.0f ? radiusMeters : SAFETY_GEOFENCE_RADIUS_M;
  Serial.printf("[Safety] geofence set lat=%.6f lon=%.6f radius=%.1f m\n", g_fenceLat, g_fenceLon, g_fenceRadiusM);
}

void safetyEnableGeoFence(bool enabled) {
  g_geofenceEnabled = enabled;
  Serial.printf("[Safety] geofence enabled=%d\n", enabled ? 1 : 0);
}

bool safetyShouldAcceptOta(int rolloutPercent, String* rejectReason) {
  int percent = constrain(rolloutPercent, 0, 100);
  uint8_t bucket = deviceRolloutBucket();
  bool accepted = bucket < percent;
  if (!accepted && rejectReason != nullptr) *rejectReason = String("rollout_bucket_") + bucket + "_not_in_" + percent + "_percent";
  Serial.printf("[Safety] ota rollout percent=%d bucket=%u accepted=%d\n", percent, bucket, accepted ? 1 : 0);
  return accepted;
}

void safetyPrepareOta(bool rollbackAllowed) {
  g_rollbackAllowed = rollbackAllowed;
  stopMotors("ota_prepare");
  Serial.printf("[Safety] OTA prepare rollback_allowed=%d\n", rollbackAllowed ? 1 : 0);
}

void safetyMarkBootSuccessful() {
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) Serial.println("[Safety] OTA image marked valid; rollback cancelled");
  else Serial.printf("[Safety] mark image valid skipped/failed err=%d\n", static_cast<int>(err));
}

void safetyRequestRollback(const char* reason) {
  Serial.printf("[Safety] rollback requested: %s\n", reason != nullptr ? reason : "unknown");
  stopMotors("rollback_requested");
  if (g_rollbackAllowed) esp_ota_mark_app_invalid_rollback_and_reboot();
}

String safetyStatusJson() {
  JsonDocument doc;
  doc["device_id"] = get_device_id();
  doc["heartbeat_ok"] = g_status.heartbeatOk;
  doc["geofence_ok"] = g_status.geofenceOk;
  doc["battery_ok"] = g_status.batteryOk;
  doc["command_ok"] = g_status.commandOk;
  doc["fail_safe_active"] = g_status.failSafeActive;
  doc["returning_home"] = g_status.returningHome;
  doc["battery_voltage"] = g_status.batteryVoltage;
  doc["latitude"] = g_status.latitude;
  doc["longitude"] = g_status.longitude;
  doc["last_heartbeat_ms"] = g_status.lastHeartbeatMs;
  doc["last_command_accepted_ms"] = g_status.lastCommandAcceptedMs;
  doc["last_stop_reason"] = g_status.lastStopReason;
  doc["home_configured"] = g_homeConfigured;
  doc["geofence_enabled"] = g_geofenceEnabled;
  doc["geofence_radius_m"] = g_fenceRadiusM;
  doc["ts_ms"] = millis();
  doc["schema_ver"] = 1;
  String payload;
  serializeJson(doc, payload);
  return payload;
}

SafetyStatus getSafetyStatus() { return g_status; }
