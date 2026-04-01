#include "mqtt_manager.h"
static_assert(STATE_MANUAL == 0 && STATE_NAVIGATING == 1, "NAV_STATE 枚举未正确包含");
#include "motor_control.h"
#include <ArduinoJson.h>
#include <esp_random.h>
#include <HTTPUpdate.h>

const char* mqttServer = "emqx.link2you.top";
const int mqttPort = 1883;
const int MAX_MQTT_PACKET_SIZE = 10240;

const char* TOPIC_MOTOR = "/motor";
const char* TOPIC_MOTOR_STATUS = "/motor/status";
const char* TOPIC_STREAM_CONFIG = "/stream/config";
const char* TOPIC_RESTART = "/restart";
const char* TOPIC_OTA = "/ota";
const char* TOPIC_CHECK_MQTT = "/check_mqtt";
const char* TOPIC_CHECK_MQTT_REPLY = "/check_mqtt_reply";
const char* TOPIC_MOTOR_CONTROL = "/motor_control";
const char* TOPIC_NAVIGATION = "/navigation";
const char* TOPIC_NAV_MODE_FEEDBACK = "/nav_mode_feedback";
const char* TOPIC_ESP32_INFO = "/ESP32_info";
const char* TOPIC_IR_INFO = "/ir_info";
const char* TOPIC_OTA_INFO = "/OTA_info";

struct MissionRuntime {
  bool active = false;
  String missionId = "";
  String status = "idle";
  unsigned long updatedAt = 0;
};

static String g_clientId = "";
static String g_deviceId = "";
static String g_topicPrefix = "";
static MissionRuntime g_mission;

static String topicFor(const char* channel, const char* name);
static const char* navModeKeyForState(int mode);
static void ensureDeviceIdentity();
static void publishJson(const String& topic, const String& payload, bool retained = false);
static void publishAck(const String& msgId, bool ok, const char* code, const char* detail);
static void publishStatusInfo(const char* extraMsg);
static void publishMotorState(bool includeLegacy);
static void publishMissionState();
static void publishDecisionInfo(const char* reason, const char* detail);
static bool isTopic(const char* topic, const String& expected);
static bool validateNewProtocolTarget(JsonDocument& doc, String* errorDetail);
static bool runOtaUpdate(const String& otaUrl);
static void applyDirectMotorCommand(int requestedSpeedA, int requestedSpeedB, const String& msgId, bool publishAckMessage);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile int irNavState = STATE_MANUAL;
volatile int webRequestedNavState = STATE_MANUAL;
unsigned long irStateStartTime = 0;

String get_device_id() {
  ensureDeviceIdentity();
  return g_deviceId;
}

static const char* navModeKeyForState(int mode) {
  switch (mode) {
    case STATE_NAVIGATING:
      return "navigate";
    case STATE_MISSION:
      return "mission";
    case STATE_MANUAL:
    default:
      return "manual";
  }
}

String get_ir_nav_mode_str() {
  switch (irNavState) {
    case STATE_NAVIGATING:
      return "红外导航";
    case STATE_MISSION:
      return "任务模式";
    case STATE_MANUAL:
    default:
      return "手动控制";
  }
}

static void ensureDeviceIdentity() {
  if (!g_deviceId.isEmpty()) {
    return;
  }

  uint64_t chipId = ESP.getEfuseMac();
  char deviceIdBuf[20];
  snprintf(deviceIdBuf, sizeof(deviceIdBuf), "boat-%06llX",
           static_cast<unsigned long long>(chipId & 0xFFFFFFULL));
  g_deviceId = deviceIdBuf;
  g_topicPrefix = String("/sbn/v1/") + g_deviceId;
}

static String topicFor(const char* channel, const char* name) {
  ensureDeviceIdentity();
  return g_topicPrefix + "/" + channel + "/" + name;
}

static void publishJson(const String& topic, const String& payload, bool retained) {
  if (!mqttClient.connected()) {
    return;
  }
  mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

static void publishAck(const String& msgId, bool ok, const char* code, const char* detail) {
  JsonDocument ackDoc;
  if (!msgId.isEmpty()) {
    ackDoc["msg_id"] = msgId;
  }
  ackDoc["device_id"] = get_device_id();
  ackDoc["ok"] = ok;
  ackDoc["code"] = code;
  if (detail != nullptr && detail[0] != '\0') {
    ackDoc["detail"] = detail;
  }
  ackDoc["ts_ms"] = millis();
  ackDoc["schema_ver"] = 1;

  String payload;
  serializeJson(ackDoc, payload);
  publishJson(topicFor("ack", "cmd"), payload, false);
}

static void publishMotorState(bool includeLegacy) {
  extern int speedA, speedB;

  JsonDocument motorDoc;
  motorDoc["device_id"] = get_device_id();
  motorDoc["speedA_cmd"] = speedA;
  motorDoc["speedB_cmd"] = speedB;
  motorDoc["nav_mode"] = navModeKeyForState(irNavState);
  motorDoc["ts_ms"] = millis();
  motorDoc["schema_ver"] = 1;
  motorDoc["source"] = "esp32";

  String motorStatePayload;
  serializeJson(motorDoc, motorStatePayload);
  publishJson(topicFor("state", "motor"), motorStatePayload, false);

  if (includeLegacy) {
    JsonDocument legacyDoc;
    legacyDoc["speedA"] = speedA;
    legacyDoc["speedB"] = speedB;
    legacyDoc["pwm_direct"] = true;
    legacyDoc["nav_mode"] = get_ir_nav_mode_str();
    legacyDoc["timestamp"] = millis();

    String legacyPayload;
    serializeJson(legacyDoc, legacyPayload);
    publishJson(TOPIC_MOTOR_STATUS, legacyPayload, false);
  }
}

static void publishMissionState() {
  JsonDocument missionDoc;
  missionDoc["device_id"] = get_device_id();
  missionDoc["nav_mode"] = navModeKeyForState(irNavState);
  missionDoc["status"] = g_mission.status;
  missionDoc["ts_ms"] = millis();
  missionDoc["schema_ver"] = 1;

  JsonObject missionObj = missionDoc["mission"].to<JsonObject>();
  if (!g_mission.missionId.isEmpty()) {
    missionObj["mission_id"] = g_mission.missionId;
  }
  missionObj["active"] = g_mission.active;
  missionObj["updated_at_ms"] = g_mission.updatedAt;

  String payload;
  serializeJson(missionDoc, payload);
  publishJson(topicFor("state", "mission"), payload, true);
}

static void publishDecisionInfo(const char* reason, const char* detail) {
  JsonDocument decisionDoc;
  decisionDoc["device_id"] = get_device_id();
  decisionDoc["nav_mode"] = navModeKeyForState(irNavState);
  decisionDoc["reason"] = reason;
  if (detail != nullptr && detail[0] != '\0') {
    decisionDoc["detail"] = detail;
  }
  decisionDoc["ts_ms"] = millis();
  decisionDoc["schema_ver"] = 1;
  decisionDoc["source"] = "esp32";

  String payload;
  serializeJson(decisionDoc, payload);
  publishJson(topicFor("telemetry", "decision"), payload, false);
}

static void publishStatusInfo(const char* extraMsg) {
  JsonDocument legacyInfoDoc;
  legacyInfoDoc["msg"] = extraMsg != nullptr ? extraMsg : "ESP32-CAM状态信息";
  legacyInfoDoc["ip"] = WiFi.localIP().toString();
  legacyInfoDoc["wifi_ssid"] = WiFi.SSID();
  legacyInfoDoc["wifi_rssi"] = WiFi.RSSI();
  legacyInfoDoc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  legacyInfoDoc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
  legacyInfoDoc["mode"] = get_ir_nav_mode_str();
  legacyInfoDoc["clientId"] = g_clientId;
  legacyInfoDoc["device_id"] = get_device_id();
  legacyInfoDoc["nav_state"] = irNavState;
  legacyInfoDoc["web_nav_state"] = webRequestedNavState;
  legacyInfoDoc["timestamp"] = millis();

  String legacyPayload;
  serializeJson(legacyInfoDoc, legacyPayload);
  publishJson(TOPIC_ESP32_INFO, legacyPayload, false);

  extern bool cameraAvailable;

  JsonDocument stateDoc;
  stateDoc["device_id"] = get_device_id();
  stateDoc["fw_version"] = __DATE__ " " __TIME__;
  stateDoc["uptime_ms"] = millis();
  stateDoc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
  stateDoc["nav_mode"] = navModeKeyForState(irNavState);
  stateDoc["ts_ms"] = millis();
  stateDoc["schema_ver"] = 1;
  stateDoc["source"] = "esp32";
  if (extraMsg != nullptr && extraMsg[0] != '\0') {
    stateDoc["msg"] = extraMsg;
  }

  JsonObject wifiDoc = stateDoc["wifi"].to<JsonObject>();
  wifiDoc["connected"] = (WiFi.status() == WL_CONNECTED);
  wifiDoc["ssid"] = WiFi.SSID();
  wifiDoc["rssi"] = WiFi.RSSI();

  JsonObject mqttDoc = stateDoc["mqtt"].to<JsonObject>();
  mqttDoc["connected"] = mqttClient.connected();
  mqttDoc["client_id"] = g_clientId;

  JsonObject healthDoc = stateDoc["health"].to<JsonObject>();
  healthDoc["camera_ok"] = cameraAvailable;
  healthDoc["ir_ok"] = true;
  healthDoc["imu_ok"] = false;

  String statePayload;
  serializeJson(stateDoc, statePayload);
  publishJson(topicFor("state", "device"), statePayload, true);
}

void publishIRInfo(int sensorId, const char* direction, int angle) {
  if (!mqttClient.connected()) {
    return;
  }

  JsonDocument legacyDoc;
  legacyDoc["direction"] = direction;
  legacyDoc["sensor_id"] = sensorId;
  legacyDoc["angle"] = angle;
  legacyDoc["timestamp"] = millis();

  String legacyPayload;
  serializeJson(legacyDoc, legacyPayload);
  publishJson(TOPIC_IR_INFO, legacyPayload, false);

  JsonDocument perceptionDoc;
  perceptionDoc["device_id"] = get_device_id();
  perceptionDoc["nav_mode"] = navModeKeyForState(irNavState);
  perceptionDoc["ts_ms"] = millis();
  perceptionDoc["schema_ver"] = 1;
  perceptionDoc["source"] = "esp32";

  JsonObject irDoc = perceptionDoc["ir"].to<JsonObject>();
  irDoc["sensor_id"] = sensorId;
  irDoc["direction"] = direction;
  irDoc["angle_deg"] = angle;

  String perceptionPayload;
  serializeJson(perceptionDoc, perceptionPayload);
  publishJson(topicFor("telemetry", "perception"), perceptionPayload, false);
}

static bool isTopic(const char* topic, const String& expected) {
  return strcmp(topic, expected.c_str()) == 0;
}

static bool validateNewProtocolTarget(JsonDocument& doc, String* errorDetail) {
  if (!doc["device_id"].is<const char*>()) {
    if (errorDetail != nullptr) {
      *errorDetail = "missing device_id";
    }
    return false;
  }

  String targetId = doc["device_id"].as<const char*>();
  if (targetId != get_device_id()) {
    if (errorDetail != nullptr) {
      *errorDetail = "device_id mismatch";
    }
    return false;
  }

  return true;
}

static void applyDirectMotorCommand(int requestedSpeedA, int requestedSpeedB, const String& msgId, bool publishAckMessage) {
  extern int speedA, speedB;

  speedA = constrain(requestedSpeedA, -255, 255);
  speedB = constrain(requestedSpeedB, -255, 255);
  webRequestedNavState = STATE_MANUAL;
  setNavMode(STATE_MANUAL);
  motor_control(0, speedA);
  motor_control(1, speedB);

  publishMotorState(true);
  publishDecisionInfo("direct_pwm", "manual motor command applied");

  if (publishAckMessage) {
    publishAck(msgId, true, "OK", "motor command applied");
  }
}

static bool runOtaUpdate(const String& otaUrl) {
  if (otaUrl.isEmpty()) {
    publishJson(TOPIC_OTA_INFO, "OTA URL为空", false);
    return false;
  }

  publishJson(TOPIC_OTA_INFO, (String("开始OTA升级，URL:") + otaUrl), false);
  WiFiClient otaClient;
  t_httpUpdate_return ret = httpUpdate.update(otaClient, otaUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      publishJson(TOPIC_OTA_INFO, (String("OTA失败: ") + httpUpdate.getLastErrorString()), false);
      Serial.printf("OTA失败: %s\n", httpUpdate.getLastErrorString().c_str());
      return false;
    case HTTP_UPDATE_NO_UPDATES:
      publishJson(TOPIC_OTA_INFO, "OTA无可用更新", false);
      Serial.println("OTA无可用更新");
      return false;
    case HTTP_UPDATE_OK:
      publishJson(TOPIC_OTA_INFO, "OTA升级成功，正在重启...", false);
      Serial.println("OTA升级成功，正在重启...");
      delay(1000);
      ESP.restart();
      return true;
  }

  return false;
}

void setNavMode(int mode) {
  if (mode < STATE_MANUAL || mode > STATE_MISSION) {
    return;
  }

  if (mode != irNavState) {
    irNavState = mode;
    irStateStartTime = millis();
    Serial.printf("导航状态切换为: %s\n", get_ir_nav_mode_str().c_str());

    extern int speedA, speedB;
    speedA = 0;
    speedB = 0;
    motor_control(0, 0);
    motor_control(1, 0);
  }

  JsonDocument feedbackDoc;
  feedbackDoc["nav_mode"] = get_ir_nav_mode_str();
  feedbackDoc["nav_mode_key"] = navModeKeyForState(irNavState);
  feedbackDoc["device_id"] = get_device_id();
  feedbackDoc["timestamp"] = millis();

  String feedbackPayload;
  serializeJson(feedbackDoc, feedbackPayload);
  publishJson(TOPIC_NAV_MODE_FEEDBACK, feedbackPayload, false);

  publishStatusInfo("导航模式切换");
  publishMotorState(true);
  publishMissionState();
}

void setupMQTT() {
  ensureDeviceIdentity();
  randomSeed(esp_random());
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);
  mqttClient.setBufferSize(MAX_MQTT_PACKET_SIZE);
  Serial.printf("MQTT客户端已配置，device_id=%s\n", g_deviceId.c_str());
}

bool mqtt_reconnect() {
  ensureDeviceIdentity();

  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");

    g_clientId = "ESP32-CAM-" + g_deviceId;
    if (mqttClient.connect(g_clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");

      mqttClient.subscribe(TOPIC_MOTOR);
      mqttClient.subscribe(TOPIC_STREAM_CONFIG);
      mqttClient.subscribe(TOPIC_RESTART);
      mqttClient.subscribe(TOPIC_OTA);
      mqttClient.subscribe(TOPIC_CHECK_MQTT);
      mqttClient.subscribe(TOPIC_MOTOR_CONTROL);
      mqttClient.subscribe(TOPIC_NAVIGATION);

      mqttClient.subscribe(topicFor("cmd", "motor").c_str());
      mqttClient.subscribe(topicFor("cmd", "nav_mode").c_str());
      mqttClient.subscribe(topicFor("cmd", "mission").c_str());
      mqttClient.subscribe(topicFor("cmd", "ota").c_str());
      mqttClient.subscribe(topicFor("cmd", "restart").c_str());

      publishStatusInfo("ESP32-CAM已上线");
      publishMotorState(true);
      publishMissionState();
      publishDecisionInfo("mqtt", "broker connected");
      return true;
    }

    Serial.print("MQTT连接失败, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" 1秒后重试");
    delay(1000);
  }

  if (!mqttClient.connected()) {
    Serial.println("MQTT连接失败次数过多。");
  }
  return mqttClient.connected();
}

void loopMQTT() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  ensureDeviceIdentity();
  Serial.printf("收到MQTT消息: 主题=%s, 长度=%d\n", topic, length);

  char message[256];
  if (length < sizeof(message) - 1) {
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("消息内容: %s\n", message);
  } else {
    Serial.println("消息过长，无法完整显示。");
  }

  if (strcmp(topic, TOPIC_MOTOR) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return;
    }

    if (doc["clientId"].is<const char*>()) {
      String targetId = doc["clientId"].as<const char*>();
      if (targetId != g_clientId) {
        Serial.printf("忽略非本机指令，目标clientId=%s，本机clientId=%s\n",
                      targetId.c_str(), g_clientId.c_str());
        return;
      }
    } else {
      Serial.println("未指定clientId，忽略指令");
      return;
    }

    if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
      applyDirectMotorCommand(doc["speedA"].as<int>(), doc["speedB"].as<int>(), "", false);
    } else {
      Serial.println("JSON中缺少speedA或speedB字段或类型错误，应为整数。");
    }
    return;
  }

  if (strcmp(topic, TOPIC_NAVIGATION) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error || !doc["command"].is<const char*>()) {
      return;
    }

    String cmd = doc["command"].as<const char*>();
    if (cmd == "manual") {
      webRequestedNavState = STATE_MANUAL;
      setNavMode(STATE_MANUAL);
      publishDecisionInfo("legacy_nav_mode", "manual");
    } else if (cmd == "navigate") {
      webRequestedNavState = STATE_NAVIGATING;
      setNavMode(STATE_NAVIGATING);
      publishDecisionInfo("legacy_nav_mode", "navigate");
    } else {
      Serial.printf("收到未知导航命令: %s，未做处理\n", cmd.c_str());
    }
    return;
  }

  if (strcmp(topic, TOPIC_CHECK_MQTT) == 0) {
    publishJson(TOPIC_CHECK_MQTT_REPLY, "{\"msg\":\"pong\"}", false);
    Serial.println("收到/check_mqtt，已回复/pong");
    publishStatusInfo("MQTT连接检查响应");
    return;
  }

  if (strcmp(topic, TOPIC_OTA) == 0) {
    String otaUrl;
    otaUrl.reserve(length);
    for (unsigned int i = 0; i < length; ++i) {
      otaUrl += static_cast<char>(payload[i]);
    }
    otaUrl.trim();
    Serial.printf("收到OTA升级命令，URL: %s\n", otaUrl.c_str());
    runOtaUpdate(otaUrl);
    return;
  }

  if (strcmp(topic, TOPIC_RESTART) == 0) {
    Serial.println("收到重启命令，准备重启...");
    publishJson(TOPIC_OTA_INFO, "即将重启ESP32", false);
    delay(1000);
    ESP.restart();
    return;
  }

  if (strcmp(topic, TOPIC_MOTOR_CONTROL) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error && doc["key"].is<const char*>()) {
      String key = doc["key"].as<const char*>();
      if (key == "shutdown") {
        applyDirectMotorCommand(0, 0, "", false);
        Serial.println("收到电机关闭命令，已自动归零速度");
      }
    }
    return;
  }

  if (isTopic(topic, topicFor("cmd", "motor"))) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      publishAck("", false, "BAD_JSON", "motor payload is not valid json");
      return;
    }

    String msgId = doc["msg_id"].is<const char*>() ? doc["msg_id"].as<const char*>() : "";
    String detail;
    if (!validateNewProtocolTarget(doc, &detail)) {
      publishAck(msgId, false, "BAD_DEVICE_ID", detail.c_str());
      return;
    }

    if (!doc["mode"].is<const char*>()) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "missing mode");
      return;
    }

    String mode = doc["mode"].as<const char*>();
    if (mode != "direct_pwm") {
      publishAck(msgId, false, "UNSUPPORTED_MODE", "only direct_pwm is implemented");
      return;
    }

    if (!doc["speedA"].is<int>() || !doc["speedB"].is<int>()) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "speedA or speedB missing");
      return;
    }

    int requestedSpeedA = doc["speedA"].as<int>();
    int requestedSpeedB = doc["speedB"].as<int>();
    if (requestedSpeedA < -255 || requestedSpeedA > 255 ||
        requestedSpeedB < -255 || requestedSpeedB > 255) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "speedA or speedB out of range");
      return;
    }

    applyDirectMotorCommand(requestedSpeedA, requestedSpeedB, msgId, true);
    return;
  }

  if (isTopic(topic, topicFor("cmd", "nav_mode"))) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      publishAck("", false, "BAD_JSON", "nav_mode payload is not valid json");
      return;
    }

    String msgId = doc["msg_id"].is<const char*>() ? doc["msg_id"].as<const char*>() : "";
    String detail;
    if (!validateNewProtocolTarget(doc, &detail)) {
      publishAck(msgId, false, "BAD_DEVICE_ID", detail.c_str());
      return;
    }

    if (!doc["mode"].is<const char*>()) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "missing mode");
      return;
    }

    String mode = doc["mode"].as<const char*>();
    if (mode == "manual") {
      webRequestedNavState = STATE_MANUAL;
      setNavMode(STATE_MANUAL);
    } else if (mode == "navigate") {
      webRequestedNavState = STATE_NAVIGATING;
      setNavMode(STATE_NAVIGATING);
    } else if (mode == "mission") {
      webRequestedNavState = STATE_MISSION;
      setNavMode(STATE_MISSION);
    } else {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "unknown nav mode");
      return;
    }

    publishDecisionInfo("nav_mode", mode.c_str());
    publishAck(msgId, true, "OK", "nav mode updated");
    return;
  }

  if (isTopic(topic, topicFor("cmd", "mission"))) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      publishAck("", false, "BAD_JSON", "mission payload is not valid json");
      return;
    }

    String msgId = doc["msg_id"].is<const char*>() ? doc["msg_id"].as<const char*>() : "";
    String detail;
    if (!validateNewProtocolTarget(doc, &detail)) {
      publishAck(msgId, false, "BAD_DEVICE_ID", detail.c_str());
      return;
    }

    if (!doc["action"].is<const char*>()) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "missing action");
      return;
    }

    String action = doc["action"].as<const char*>();
    if (action == "start") {
      if (!doc["mission"].is<JsonObject>()) {
        publishAck(msgId, false, "BAD_PARAM_RANGE", "mission object is required");
        return;
      }

      JsonObject missionObj = doc["mission"].as<JsonObject>();
      if (!missionObj["mission_id"].is<const char*>()) {
        publishAck(msgId, false, "BAD_PARAM_RANGE", "mission.mission_id is required");
        return;
      }

      g_mission.active = true;
      g_mission.missionId = missionObj["mission_id"].as<const char*>();
      g_mission.status = "accepted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MISSION;
      setNavMode(STATE_MISSION);
      publishMissionState();
      publishDecisionInfo("mission", "mission accepted; executor pending");
      publishAck(msgId, true, "OK", "mission accepted; executor not implemented yet");
      return;
    }

    if (!g_mission.active && action != "abort") {
      publishAck(msgId, false, "MODE_CONFLICT", "no active mission");
      return;
    }

    if (action == "pause") {
      g_mission.status = "paused";
      g_mission.updatedAt = millis();
      publishMissionState();
      publishDecisionInfo("mission", "paused");
      publishAck(msgId, true, "OK", "mission paused");
      return;
    }

    if (action == "resume") {
      g_mission.status = "accepted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MISSION;
      setNavMode(STATE_MISSION);
      publishMissionState();
      publishDecisionInfo("mission", "resumed");
      publishAck(msgId, true, "OK", "mission resumed");
      return;
    }

    if (action == "abort") {
      g_mission.active = false;
      g_mission.status = "aborted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MANUAL;
      setNavMode(STATE_MANUAL);
      publishMissionState();
      publishDecisionInfo("mission", "aborted");
      publishAck(msgId, true, "OK", "mission aborted");
      return;
    }

    publishAck(msgId, false, "BAD_PARAM_RANGE", "unknown mission action");
    return;
  }

  if (isTopic(topic, topicFor("cmd", "ota"))) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      publishAck("", false, "BAD_JSON", "ota payload is not valid json");
      return;
    }

    String msgId = doc["msg_id"].is<const char*>() ? doc["msg_id"].as<const char*>() : "";
    String detail;
    if (!validateNewProtocolTarget(doc, &detail)) {
      publishAck(msgId, false, "BAD_DEVICE_ID", detail.c_str());
      return;
    }

    if (!doc["url"].is<const char*>()) {
      publishAck(msgId, false, "BAD_PARAM_RANGE", "missing url");
      return;
    }

    publishAck(msgId, true, "OK", "ota command accepted");
    runOtaUpdate(doc["url"].as<const char*>());
    return;
  }

  if (isTopic(topic, topicFor("cmd", "restart"))) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      publishAck("", false, "BAD_JSON", "restart payload is not valid json");
      return;
    }

    String msgId = doc["msg_id"].is<const char*>() ? doc["msg_id"].as<const char*>() : "";
    String detail;
    if (!validateNewProtocolTarget(doc, &detail)) {
      publishAck(msgId, false, "BAD_DEVICE_ID", detail.c_str());
      return;
    }

    int delayMs = doc["delay_ms"].is<int>() ? doc["delay_ms"].as<int>() : 1000;
    publishAck(msgId, true, "OK", "restart scheduled");
    publishJson(TOPIC_OTA_INFO, "即将重启ESP32", false);
    delay(delayMs);
    ESP.restart();
    return;
  }
}
