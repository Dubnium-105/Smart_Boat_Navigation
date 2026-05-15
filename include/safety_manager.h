#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>

/**
 * @brief 船端可靠性与安全保护状态。
 */
struct SafetyStatus {
  bool heartbeatOk;
  bool geofenceOk;
  bool batteryOk;
  bool commandOk;
  bool failSafeActive;
  bool returningHome;
  float batteryVoltage;
  double latitude;
  double longitude;
  unsigned long lastHeartbeatMs;
  unsigned long lastCommandAcceptedMs;
  String lastStopReason;
};

/**
 * @brief 初始化安全模块，加载默认地理围栏、低电阈值与指令限速参数。
 */
void setupSafetyManager();

/**
 * @brief 主循环安全巡检。负责心跳超时停车、地理围栏保护、电量低返航触发等。
 */
void safetyLoop();

/**
 * @brief 记录上位机或MQTT控制端心跳。
 */
void safetyRecordHeartbeat();

/**
 * @brief 判断当前是否允许电机输出。
 */
bool safetyAllowsMotion();

/**
 * @brief 对即将下发的左右电机速度做安全钳制。返回false时速度会被置零。
 */
bool safetyGuardMotorCommand(int& leftPwm, int& rightPwm, const char* source);

/**
 * @brief 指令白名单与速率限制。返回false表示应拒绝执行该指令。
 */
bool safetyAcceptCommand(const char* commandKey, String* rejectReason = nullptr);

/**
 * @brief 更新定位与电池遥测。未接入传感器时可由上位机/MQTT透传。
 */
void safetyUpdateTelemetry(double latitude, double longitude, float batteryVoltage);

/**
 * @brief 设置返航/地理围栏中心点。
 */
void safetySetHome(double latitude, double longitude);

/**
 * @brief 设置圆形地理围栏。
 */
void safetySetGeoFence(double centerLatitude, double centerLongitude, float radiusMeters);

/**
 * @brief 启停地理围栏保护。
 */
void safetyEnableGeoFence(bool enabled);

/**
 * @brief OTA灰度判断。rolloutPercent取0~100，基于设备ID散列稳定分桶。
 */
bool safetyShouldAcceptOta(int rolloutPercent, String* rejectReason = nullptr);

/**
 * @brief OTA开始前记录策略，必要时由ESP-IDF回滚机制兜底。
 */
void safetyPrepareOta(bool rollbackAllowed);

/**
 * @brief 新固件启动后确认运行正常，取消回滚挂起状态。
 */
void safetyMarkBootSuccessful();

/**
 * @brief 触发回滚并重启。仅在分区表/bootloader支持回滚时生效。
 */
void safetyRequestRollback(const char* reason);

/**
 * @brief 生成安全状态JSON，便于MQTT发布。
 */
String safetyStatusJson();

/**
 * @brief 读取当前安全状态。
 */
SafetyStatus getSafetyStatus();

#endif  // SAFETY_MANAGER_H
