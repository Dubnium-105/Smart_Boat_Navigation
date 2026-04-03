#include "mqtt_manager.h"
static_assert(STATE_MANUAL == 0 && STATE_NAVIGATING == 1, "NAV_STATE enum mismatch");

#include "motor_control.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <esp_random.h>
#include <time.h>

const char* mqttServer = "broker.emqx.io";
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

const char* ROLE_ADMIN_TOKEN = "sbn-admin-token";
const char* ROLE_OPERATOR_TOKEN = "sbn-operator-token";
const char* ROLE_VIEWER_TOKEN = "sbn-viewer-token";

constexpr uint32_t CMD_DEDUP_TTL_MS = 120000;
constexpr uint8_t CMD_DEDUP_CACHE_SIZE = 20;
constexpr uint8_t TELEMETRY_KIND_IR = 1;
constexpr uint8_t TELEMETRY_KIND_DECISION = 2;
constexpr uint8_t TELEMETRY_KIND_MOTOR = 3;
constexpr uint8_t TELEMETRY_KIND_HEALTH = 4;
constexpr uint8_t TELEMETRY_RING_CAP = 96;
constexpr uint8_t TELEMETRY_MAX_SAMPLES_PER_BATCH = 24;
constexpr uint16_t TELEMETRY_SAMPLE_BYTES = 12;
constexpr uint16_t TELEMETRY_HEADER_BYTES = 24;
constexpr uint16_t TELEMETRY_MAX_RAW_BYTES = TELEMETRY_RING_CAP * TELEMETRY_SAMPLE_BYTES;
constexpr uint16_t TELEMETRY_MAX_COMPRESSED_BYTES = TELEMETRY_MAX_RAW_BYTES * 2;
constexpr uint16_t TELEMETRY_MAX_PACKET_BYTES = TELEMETRY_HEADER_BYTES + TELEMETRY_MAX_COMPRESSED_BYTES;

struct MissionRuntime {
  bool active = false;
  String missionId = "";
  String status = "idle";
  unsigned long updatedAt = 0;
};

enum AuthRole {
  AUTH_ROLE_NONE = 0,
  AUTH_ROLE_VIEWER = 1,
  AUTH_ROLE_OPERATOR = 2,
  AUTH_ROLE_ADMIN = 3
};

struct CommandAckCacheEntry {
  String msgId;
  bool ok = false;
  String code;
  String detail;
  uint8_t protocolVer = PROTOCOL_ACTIVE_VERSION;
  String clientId;
  bool retryable = false;
  unsigned long tsMs = 0;
};

struct TelemetrySample {
  uint8_t kind = 0;
  uint8_t navMode = 0;
  int8_t value1 = 0;
  int8_t value2 = 0;
  uint32_t tsMs = 0;
  int32_t extra = 0;
};

struct CommandResult {
  bool ok = false;
  bool retryable = false;
  String code = "INTERNAL_ERROR";
  String detail = "internal error";
  String otaUrl = "";
  int restartDelayMs = 0;
};

static String g_clientId = "";
static String g_deviceId = "";
static String g_topicPrefixV1 = "";
static String g_topicPrefixV2 = "";
static MissionRuntime g_mission;

static CommandAckCacheEntry g_cmdCache[CMD_DEDUP_CACHE_SIZE];
static unsigned long g_lastCmdCacheGcMs = 0;

static TelemetrySample g_telemetryRing[TELEMETRY_RING_CAP];
static uint8_t g_telemetryHead = 0;
static uint8_t g_telemetryTail = 0;
static uint8_t g_telemetryCount = 0;
static uint8_t g_telemetryBatchMaxSamples = TELEMETRY_MAX_SAMPLES_PER_BATCH;
static uint16_t g_telemetryBatchIntervalMs = 1500;
static unsigned long g_lastTelemetryFlushMs = 0;
static uint32_t g_telemetryBatchSeq = 0;

static bool g_httpTelemetryEnabled = false;
static String g_httpTelemetryUrl = "";
static uint8_t g_pendingHttpPacket[TELEMETRY_MAX_PACKET_BYTES];
static size_t g_pendingHttpPacketLen = 0;
static uint8_t g_pendingHttpRetryCount = 0;
static unsigned long g_nextHttpRetryMs = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile int irNavState = STATE_MANUAL;
volatile int webRequestedNavState = STATE_MANUAL;
unsigned long irStateStartTime = 0;

static const char* navModeKeyForState(int mode);
static String topicForVersion(uint8_t protocolVer, const char* channel, const char* name);
static String topicFor(const char* channel, const char* name);
static void ensureDeviceIdentity();
static void publishJson(const String& topic, const String& payload, bool retained = false);
static void publishBinary(const String& topic, const uint8_t* payload, size_t payloadLen, bool retained = false);
static void setBaseMessageFields(JsonDocument& doc, const char* msgType, const char* msgName, uint8_t protocolVer);
static bool isTopic(const char* topic, const String& expected);

static uint8_t parseProtocolVersion(const JsonDocument& doc);
static JsonVariantConst commandField(const JsonDocument& doc, const char* key, const char* legacyKey = nullptr);
static String commandString(const JsonDocument& doc, const char* key, const char* legacyKey = nullptr);
static bool commandInt(const JsonDocument& doc, const char* key, int* outValue, const char* legacyKey = nullptr);
static bool commandUInt32(const JsonDocument& doc, const char* key, uint32_t* outValue, const char* legacyKey = nullptr);
static bool validateNewProtocolTarget(const JsonDocument& doc, uint8_t protocolVer, String* errorDetail);
static bool validateTtlWindow(const JsonDocument& doc, String* errorDetail);
static AuthRole roleForToken(const String& token);
static bool checkCommandAuthorization(const JsonDocument& doc, const String& cmdName, uint8_t protocolVer,
                                      String* clientId, AuthRole* role, String* errorCode, String* errorDetail);
static bool roleHasPermission(AuthRole role, const String& cmdName);

static int findCachedAck(const String& msgId);
static void rememberAck(const String& msgId, bool ok, const String& code, const String& detail,
                        uint8_t protocolVer, const String& clientId, bool retryable);
static void cleanupAckCache();

static void publishAck(const String& msgId, bool ok, const String& code, const String& detail,
                       uint8_t protocolVer, const String& clientId, bool duplicate = false, bool retryable = false);
static void publishStatusInfo(const char* extraMsg);
static void publishMotorState(bool includeLegacy);
static void publishMissionState();
static void publishDecisionInfo(const char* reason, const char* detail);
static bool runOtaUpdate(const String& otaUrl);
static void applyDirectMotorCommand(int requestedSpeedA, int requestedSpeedB);
static bool handleLegacyTopic(const char* topic, const uint8_t* payload, unsigned int length);
static bool dispatchUnifiedCommand(const String& cmdName, uint8_t protocolVer, const uint8_t* payload, unsigned int length);
static bool resolveUnifiedCommandTopic(const char* topic, uint8_t* protocolVer, String* cmdName);

static void pushTelemetrySample(uint8_t kind, uint8_t navMode, int8_t value1, int8_t value2, int32_t extra);
static size_t encodeTelemetryRaw(TelemetrySample* samples, size_t sampleCount, uint8_t* outRaw, size_t outCap);
static size_t rleCompress(const uint8_t* input, size_t inputLen, uint8_t* output, size_t outputCap);
static uint32_t crc32(const uint8_t* data, size_t len);
static void putU16LE(uint8_t* buf, size_t offset, uint16_t value);
static void putU32LE(uint8_t* buf, size_t offset, uint32_t value);
static bool buildTelemetryPacket(TelemetrySample* samples, size_t sampleCount, uint8_t* outPacket, size_t* outLen,
                                 float* compressionRatio);
static bool publishTelemetryBatch(bool force);
static bool uploadTelemetryPacketHttp(const uint8_t* packet, size_t packetLen);
static void scheduleHttpRetry(const uint8_t* packet, size_t packetLen);
static void processHttpRetry();
static int32_t fnv1aHash32(const String& text);

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
  g_topicPrefixV1 = String("/sbn/v1/") + g_deviceId;
  g_topicPrefixV2 = String("/sbn/v2/") + g_deviceId;
}

static String topicForVersion(uint8_t protocolVer, const char* channel, const char* name) {
  ensureDeviceIdentity();
  String prefix = protocolVer >= 2 ? g_topicPrefixV2 : g_topicPrefixV1;
  return prefix + "/" + channel + "/" + name;
}

static String topicFor(const char* channel, const char* name) {
  return topicForVersion(PROTOCOL_ACTIVE_VERSION, channel, name);
}

static void publishJson(const String& topic, const String& payload, bool retained) {
  if (!mqttClient.connected()) {
    return;
  }
  mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

static void publishBinary(const String& topic, const uint8_t* payload, size_t payloadLen, bool retained) {
  if (!mqttClient.connected() || payload == nullptr || payloadLen == 0) {
    return;
  }
  mqttClient.publish(topic.c_str(), payload, static_cast<unsigned int>(payloadLen), retained);
}

static void setBaseMessageFields(JsonDocument& doc, const char* msgType, const char* msgName, uint8_t protocolVer) {
  doc["protocol_ver"] = protocolVer;
  doc["schema_ver"] = PROTOCOL_SCHEMA_VERSION;
  doc["compat_min_ver"] = PROTOCOL_COMPAT_MIN_VERSION;
  doc["msg_type"] = msgType;
  doc["msg_name"] = msgName;
  doc["device_id"] = get_device_id();
  doc["ts_ms"] = millis();
  doc["source"] = "esp32";
}

static bool isTopic(const char* topic, const String& expected) {
  return strcmp(topic, expected.c_str()) == 0;
}

static JsonVariantConst commandField(const JsonDocument& doc, const char* key, const char* legacyKey) {
  if (doc["data"].is<JsonObjectConst>()) {
    JsonObjectConst dataObj = doc["data"].as<JsonObjectConst>();
    JsonVariantConst dataValue = dataObj[key];
    if (!dataValue.isNull()) {
      return dataValue;
    }
    if (legacyKey != nullptr) {
      JsonVariantConst legacyDataValue = dataObj[legacyKey];
      if (!legacyDataValue.isNull()) {
        return legacyDataValue;
      }
    }
  }

  JsonVariantConst value = doc[key];
  if (!value.isNull()) {
    return value;
  }

  if (legacyKey != nullptr) {
    return doc[legacyKey];
  }

  return value;
}

static String commandString(const JsonDocument& doc, const char* key, const char* legacyKey) {
  JsonVariantConst value = commandField(doc, key, legacyKey);
  if (value.is<const char*>()) {
    return value.as<const char*>();
  }
  return "";
}

static bool commandInt(const JsonDocument& doc, const char* key, int* outValue, const char* legacyKey) {
  if (outValue == nullptr) {
    return false;
  }

  JsonVariantConst value = commandField(doc, key, legacyKey);
  if (value.is<int>()) {
    *outValue = value.as<int>();
    return true;
  }
  if (value.is<long>()) {
    *outValue = static_cast<int>(value.as<long>());
    return true;
  }
  if (value.is<float>()) {
    *outValue = static_cast<int>(value.as<float>());
    return true;
  }
  return false;
}

static bool commandUInt32(const JsonDocument& doc, const char* key, uint32_t* outValue, const char* legacyKey) {
  if (outValue == nullptr) {
    return false;
  }

  JsonVariantConst value = commandField(doc, key, legacyKey);
  if (value.is<uint32_t>()) {
    *outValue = value.as<uint32_t>();
    return true;
  }
  if (value.is<unsigned long>()) {
    *outValue = static_cast<uint32_t>(value.as<unsigned long>());
    return true;
  }
  if (value.is<int>()) {
    int v = value.as<int>();
    if (v >= 0) {
      *outValue = static_cast<uint32_t>(v);
      return true;
    }
  }
  return false;
}

static uint8_t parseProtocolVersion(const JsonDocument& doc) {
  JsonVariantConst direct = doc["protocol_ver"];
  if (!direct.isNull()) {
    if (direct.is<int>()) {
      int v = direct.as<int>();
      if (v > 0 && v < 255) {
        return static_cast<uint8_t>(v);
      }
    }
  }

  if (doc["meta"].is<JsonObjectConst>()) {
    JsonObjectConst meta = doc["meta"].as<JsonObjectConst>();
    JsonVariantConst nested = meta["protocol_ver"];
    if (nested.is<int>()) {
      int v = nested.as<int>();
      if (v > 0 && v < 255) {
        return static_cast<uint8_t>(v);
      }
    }
  }

  return PROTOCOL_COMPAT_MIN_VERSION;
}

static bool validateNewProtocolTarget(const JsonDocument& doc, uint8_t protocolVer, String* errorDetail) {
  String targetId = commandString(doc, "device_id");
  if (targetId.isEmpty()) {
    if (protocolVer >= PROTOCOL_ACTIVE_VERSION) {
      if (errorDetail != nullptr) {
        *errorDetail = "missing device_id";
      }
      return false;
    }
    return true;
  }

  if (targetId != get_device_id()) {
    if (errorDetail != nullptr) {
      *errorDetail = "device_id mismatch";
    }
    return false;
  }

  return true;
}

static bool validateTtlWindow(const JsonDocument& doc, String* errorDetail) {
  uint32_t ttlMs = 0;
  if (!commandUInt32(doc, "ttl_ms", &ttlMs)) {
    return true;
  }

  if (ttlMs == 0 || ttlMs > 60000) {
    if (errorDetail != nullptr) {
      *errorDetail = "ttl_ms out of range";
    }
    return false;
  }

  JsonVariantConst tsValue = commandField(doc, "ts_unix_ms");
  if (tsValue.isNull()) {
    return true;
  }

  long long sendUnixMs = 0;
  if (tsValue.is<long long>()) {
    sendUnixMs = tsValue.as<long long>();
  } else if (tsValue.is<long>()) {
    sendUnixMs = tsValue.as<long>();
  } else if (tsValue.is<int>()) {
    sendUnixMs = tsValue.as<int>();
  } else {
    return true;
  }

  time_t nowSec = time(nullptr);
  if (nowSec < 1700000000) {
    return true;
  }

  long long nowMs = static_cast<long long>(nowSec) * 1000;
  if (sendUnixMs + static_cast<long long>(ttlMs) < nowMs) {
    if (errorDetail != nullptr) {
      *errorDetail = "command expired";
    }
    return false;
  }

  return true;
}

static AuthRole roleForToken(const String& token) {
  if (token.isEmpty()) {
    return AUTH_ROLE_NONE;
  }

  if (token == ROLE_ADMIN_TOKEN || token == (String("adm-") + get_device_id() + "-2026")) {
    return AUTH_ROLE_ADMIN;
  }
  if (token == ROLE_OPERATOR_TOKEN || token == (String("op-") + get_device_id() + "-2026")) {
    return AUTH_ROLE_OPERATOR;
  }
  if (token == ROLE_VIEWER_TOKEN || token == (String("view-") + get_device_id() + "-2026")) {
    return AUTH_ROLE_VIEWER;
  }

  return AUTH_ROLE_NONE;
}

static bool roleHasPermission(AuthRole role, const String& cmdName) {
  if (cmdName == "motor" || cmdName == "nav_mode") {
    return role >= AUTH_ROLE_OPERATOR;
  }

  if (cmdName == "mission" || cmdName == "ota" || cmdName == "restart" || cmdName == "protocol_cfg") {
    return role >= AUTH_ROLE_ADMIN;
  }

  return false;
}

static bool checkCommandAuthorization(const JsonDocument& doc, const String& cmdName, uint8_t protocolVer,
                                      String* clientId, AuthRole* role, String* errorCode, String* errorDetail) {
  (void)protocolVer;
  String cid = commandString(doc, "client_id", "clientId");
  String token = commandString(doc, "token");

  if (clientId != nullptr) {
    *clientId = cid;
  }

  if (cid.isEmpty()) {
    if (errorCode != nullptr) {
      *errorCode = "AUTH_REQUIRED";
    }
    if (errorDetail != nullptr) {
      *errorDetail = "missing client_id";
    }
    return false;
  }

  if (token.isEmpty()) {
    if (errorCode != nullptr) {
      *errorCode = "AUTH_REQUIRED";
    }
    if (errorDetail != nullptr) {
      *errorDetail = "missing token";
    }
    return false;
  }

  AuthRole resolvedRole = roleForToken(token);
  if (role != nullptr) {
    *role = resolvedRole;
  }

  if (resolvedRole == AUTH_ROLE_NONE) {
    if (errorCode != nullptr) {
      *errorCode = "AUTH_FAILED";
    }
    if (errorDetail != nullptr) {
      *errorDetail = "token not accepted";
    }
    return false;
  }

  if (!roleHasPermission(resolvedRole, cmdName)) {
    if (errorCode != nullptr) {
      *errorCode = "PERMISSION_DENIED";
    }
    if (errorDetail != nullptr) {
      *errorDetail = "role does not have permission for command";
    }
    return false;
  }

  return true;
}

static int findCachedAck(const String& msgId) {
  if (msgId.isEmpty()) {
    return -1;
  }

  for (uint8_t i = 0; i < CMD_DEDUP_CACHE_SIZE; ++i) {
    if (g_cmdCache[i].msgId == msgId) {
      return static_cast<int>(i);
    }
  }

  return -1;
}

static void rememberAck(const String& msgId, bool ok, const String& code, const String& detail,
                        uint8_t protocolVer, const String& clientId, bool retryable) {
  if (msgId.isEmpty()) {
    return;
  }

  int existing = findCachedAck(msgId);
  int slot = existing;

  if (slot < 0) {
    unsigned long oldest = ULONG_MAX;
    slot = 0;
    for (uint8_t i = 0; i < CMD_DEDUP_CACHE_SIZE; ++i) {
      if (g_cmdCache[i].msgId.isEmpty()) {
        slot = i;
        oldest = 0;
        break;
      }
      if (g_cmdCache[i].tsMs < oldest) {
        oldest = g_cmdCache[i].tsMs;
        slot = i;
      }
    }
  }

  g_cmdCache[slot].msgId = msgId;
  g_cmdCache[slot].ok = ok;
  g_cmdCache[slot].code = code;
  g_cmdCache[slot].detail = detail;
  g_cmdCache[slot].protocolVer = protocolVer;
  g_cmdCache[slot].clientId = clientId;
  g_cmdCache[slot].retryable = retryable;
  g_cmdCache[slot].tsMs = millis();
}

static void cleanupAckCache() {
  unsigned long nowMs = millis();
  for (uint8_t i = 0; i < CMD_DEDUP_CACHE_SIZE; ++i) {
    if (g_cmdCache[i].msgId.isEmpty()) {
      continue;
    }
    if (nowMs - g_cmdCache[i].tsMs > CMD_DEDUP_TTL_MS) {
      g_cmdCache[i] = CommandAckCacheEntry();
    }
  }
}

static void publishAck(const String& msgId, bool ok, const String& code, const String& detail,
                       uint8_t protocolVer, const String& clientId, bool duplicate, bool retryable) {
  JsonDocument ackDoc;
  setBaseMessageFields(ackDoc, "event", "ack", protocolVer);
  if (!msgId.isEmpty()) {
    ackDoc["msg_id"] = msgId;
  }
  if (!clientId.isEmpty()) {
    ackDoc["client_id"] = clientId;
  }
  ackDoc["ok"] = ok;
  ackDoc["code"] = code;
  ackDoc["detail"] = detail;
  ackDoc["duplicate"] = duplicate;
  ackDoc["retryable"] = retryable;

  JsonObject dataObj = ackDoc["data"].to<JsonObject>();
  dataObj["ok"] = ok;
  dataObj["code"] = code;
  dataObj["detail"] = detail;
  dataObj["duplicate"] = duplicate;
  dataObj["retryable"] = retryable;

  String payload;
  serializeJson(ackDoc, payload);

  publishJson(topicForVersion(protocolVer, "ack", "cmd"), payload, false);
  if (protocolVer >= PROTOCOL_ACTIVE_VERSION) {
    publishJson(topicForVersion(PROTOCOL_COMPAT_MIN_VERSION, "ack", "cmd"), payload, false);
  }
}

static int32_t fnv1aHash32(const String& text) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < text.length(); ++i) {
    hash ^= static_cast<uint8_t>(text[i]);
    hash *= 16777619UL;
  }
  return static_cast<int32_t>(hash);
}

static void pushTelemetrySample(uint8_t kind, uint8_t navMode, int8_t value1, int8_t value2, int32_t extra) {
  TelemetrySample sample;
  sample.kind = kind;
  sample.navMode = navMode;
  sample.value1 = value1;
  sample.value2 = value2;
  sample.tsMs = millis();
  sample.extra = extra;

  if (g_telemetryCount >= TELEMETRY_RING_CAP) {
    g_telemetryTail = (g_telemetryTail + 1) % TELEMETRY_RING_CAP;
    g_telemetryCount--;
  }

  g_telemetryRing[g_telemetryHead] = sample;
  g_telemetryHead = (g_telemetryHead + 1) % TELEMETRY_RING_CAP;
  g_telemetryCount++;
}

static size_t encodeTelemetryRaw(TelemetrySample* samples, size_t sampleCount, uint8_t* outRaw, size_t outCap) {
  if (samples == nullptr || outRaw == nullptr) {
    return 0;
  }

  const size_t needLen = sampleCount * TELEMETRY_SAMPLE_BYTES;
  if (needLen > outCap) {
    return 0;
  }

  size_t offset = 0;
  for (size_t i = 0; i < sampleCount; ++i) {
    outRaw[offset++] = samples[i].kind;
    outRaw[offset++] = samples[i].navMode;
    outRaw[offset++] = static_cast<uint8_t>(samples[i].value1);
    outRaw[offset++] = static_cast<uint8_t>(samples[i].value2);

    uint32_t ts = samples[i].tsMs;
    outRaw[offset++] = static_cast<uint8_t>(ts & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((ts >> 8) & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((ts >> 16) & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((ts >> 24) & 0xFF);

    int32_t extra = samples[i].extra;
    outRaw[offset++] = static_cast<uint8_t>(extra & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((extra >> 8) & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((extra >> 16) & 0xFF);
    outRaw[offset++] = static_cast<uint8_t>((extra >> 24) & 0xFF);
  }

  return offset;
}

static size_t rleCompress(const uint8_t* input, size_t inputLen, uint8_t* output, size_t outputCap) {
  if (input == nullptr || output == nullptr || inputLen == 0) {
    return 0;
  }

  size_t inPos = 0;
  size_t outPos = 0;

  while (inPos < inputLen) {
    uint8_t current = input[inPos];
    uint8_t runLen = 1;

    while (inPos + runLen < inputLen && input[inPos + runLen] == current && runLen < 255) {
      runLen++;
    }

    if (outPos + 2 > outputCap) {
      return 0;
    }

    output[outPos++] = runLen;
    output[outPos++] = current;
    inPos += runLen;
  }

  return outPos;
}

static void putU16LE(uint8_t* buf, size_t offset, uint16_t value) {
  buf[offset] = static_cast<uint8_t>(value & 0xFF);
  buf[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void putU32LE(uint8_t* buf, size_t offset, uint32_t value) {
  buf[offset] = static_cast<uint8_t>(value & 0xFF);
  buf[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buf[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buf[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static bool buildTelemetryPacket(TelemetrySample* samples, size_t sampleCount, uint8_t* outPacket, size_t* outLen,
                                 float* compressionRatio) {
  if (samples == nullptr || outPacket == nullptr || outLen == nullptr || sampleCount == 0) {
    return false;
  }

  static uint8_t rawBuf[TELEMETRY_MAX_RAW_BYTES];
  static uint8_t compressedBuf[TELEMETRY_MAX_COMPRESSED_BYTES];

  size_t rawLen = encodeTelemetryRaw(samples, sampleCount, rawBuf, sizeof(rawBuf));
  if (rawLen == 0) {
    return false;
  }

  size_t compressedLen = rleCompress(rawBuf, rawLen, compressedBuf, sizeof(compressedBuf));
  const bool useCompressed = compressedLen > 0 && compressedLen < rawLen;
  const uint8_t codec = useCompressed ? 1 : 0;
  const uint8_t* payload = useCompressed ? compressedBuf : rawBuf;
  const size_t payloadLen = useCompressed ? compressedLen : rawLen;

  if (payloadLen + TELEMETRY_HEADER_BYTES > TELEMETRY_MAX_PACKET_BYTES) {
    return false;
  }

  outPacket[0] = 'S';
  outPacket[1] = 'B';
  outPacket[2] = 'N';
  outPacket[3] = '2';
  outPacket[4] = PROTOCOL_ACTIVE_VERSION;
  outPacket[5] = codec;
  putU16LE(outPacket, 6, static_cast<uint16_t>(sampleCount));
  putU16LE(outPacket, 8, static_cast<uint16_t>(rawLen));
  putU16LE(outPacket, 10, static_cast<uint16_t>(payloadLen));
  putU32LE(outPacket, 12, g_telemetryBatchSeq++);
  putU32LE(outPacket, 16, crc32(rawBuf, rawLen));
  putU32LE(outPacket, 20, millis());

  memcpy(outPacket + TELEMETRY_HEADER_BYTES, payload, payloadLen);
  *outLen = TELEMETRY_HEADER_BYTES + payloadLen;

  if (compressionRatio != nullptr) {
    *compressionRatio = static_cast<float>(payloadLen) / static_cast<float>(rawLen);
  }

  return true;
}

static bool uploadTelemetryPacketHttp(const uint8_t* packet, size_t packetLen) {
  if (!g_httpTelemetryEnabled || g_httpTelemetryUrl.isEmpty()) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  HTTPClient http;
  if (!http.begin(g_httpTelemetryUrl)) {
    return false;
  }

  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("X-Device-Id", get_device_id());
  http.addHeader("X-Protocol-Ver", String(PROTOCOL_ACTIVE_VERSION));
  http.addHeader("X-Schema-Ver", String(PROTOCOL_SCHEMA_VERSION));

  if (packetLen > TELEMETRY_MAX_PACKET_BYTES) {
    http.end();
    return false;
  }

  // HTTPClient::POST requires a mutable buffer, so copy from immutable packet data.
  uint8_t postBuffer[TELEMETRY_MAX_PACKET_BYTES];
  memcpy(postBuffer, packet, packetLen);
  int code = http.POST(postBuffer, packetLen);
  http.end();

  return code >= 200 && code < 300;
}

static void scheduleHttpRetry(const uint8_t* packet, size_t packetLen) {
  if (packet == nullptr || packetLen == 0 || packetLen > sizeof(g_pendingHttpPacket)) {
    return;
  }

  memcpy(g_pendingHttpPacket, packet, packetLen);
  g_pendingHttpPacketLen = packetLen;
  g_pendingHttpRetryCount = 0;
  g_nextHttpRetryMs = millis() + 1000;
}

static void processHttpRetry() {
  if (g_pendingHttpPacketLen == 0) {
    return;
  }

  if (millis() < g_nextHttpRetryMs) {
    return;
  }

  if (uploadTelemetryPacketHttp(g_pendingHttpPacket, g_pendingHttpPacketLen)) {
    g_pendingHttpPacketLen = 0;
    g_pendingHttpRetryCount = 0;
    g_nextHttpRetryMs = 0;
    publishDecisionInfo("telemetry_http", "pending packet upload success");
    return;
  }

  g_pendingHttpRetryCount++;
  if (g_pendingHttpRetryCount > 5) {
    g_pendingHttpPacketLen = 0;
    g_pendingHttpRetryCount = 0;
    g_nextHttpRetryMs = 0;
    publishDecisionInfo("telemetry_http", "pending packet dropped after retries");
    return;
  }

  uint32_t backoffMs = 1000UL << g_pendingHttpRetryCount;
  if (backoffMs > 60000UL) {
    backoffMs = 60000UL;
  }
  g_nextHttpRetryMs = millis() + backoffMs;
}

static bool publishTelemetryBatch(bool force) {
  if (!mqttClient.connected()) {
    return false;
  }

  if (g_telemetryCount == 0) {
    return false;
  }

  unsigned long nowMs = millis();
  if (!force) {
    if (g_telemetryCount < g_telemetryBatchMaxSamples &&
        nowMs - g_lastTelemetryFlushMs < g_telemetryBatchIntervalMs) {
      return false;
    }
  }

  TelemetrySample samples[TELEMETRY_MAX_SAMPLES_PER_BATCH];
  size_t sampleCount = 0;
  while (sampleCount < g_telemetryBatchMaxSamples && g_telemetryCount > 0) {
    samples[sampleCount++] = g_telemetryRing[g_telemetryTail];
    g_telemetryTail = (g_telemetryTail + 1) % TELEMETRY_RING_CAP;
    g_telemetryCount--;
  }

  uint8_t packet[TELEMETRY_MAX_PACKET_BYTES];
  size_t packetLen = 0;
  float ratio = 1.0f;
  if (!buildTelemetryPacket(samples, sampleCount, packet, &packetLen, &ratio)) {
    return false;
  }

  publishBinary(topicFor("telemetry", "binary"), packet, packetLen, false);

  JsonDocument batchDoc;
  setBaseMessageFields(batchDoc, "state", "telemetry_batch", PROTOCOL_ACTIVE_VERSION);
  batchDoc["sample_count"] = static_cast<int>(sampleCount);
  batchDoc["packet_bytes"] = static_cast<int>(packetLen);
  batchDoc["compression_ratio"] = ratio;
  batchDoc["batch_seq"] = g_telemetryBatchSeq - 1;

  JsonObject dataObj = batchDoc["data"].to<JsonObject>();
  dataObj["sample_count"] = static_cast<int>(sampleCount);
  dataObj["packet_bytes"] = static_cast<int>(packetLen);
  dataObj["compression_ratio"] = ratio;
  dataObj["batch_seq"] = g_telemetryBatchSeq - 1;

  String batchPayload;
  serializeJson(batchDoc, batchPayload);
  publishJson(topicFor("state", "telemetry_batch"), batchPayload, false);

  if (!uploadTelemetryPacketHttp(packet, packetLen)) {
    scheduleHttpRetry(packet, packetLen);
  }

  g_lastTelemetryFlushMs = nowMs;
  return true;
}

void pumpProtocolRuntime() {
  if (millis() - g_lastCmdCacheGcMs > 10000) {
    cleanupAckCache();
    g_lastCmdCacheGcMs = millis();
  }

  processHttpRetry();
  publishTelemetryBatch(false);
}

static void publishMotorState(bool includeLegacy) {
  extern int speedA, speedB;

  JsonDocument motorDoc;
  setBaseMessageFields(motorDoc, "state", "motor", PROTOCOL_ACTIVE_VERSION);
  motorDoc["nav_mode"] = navModeKeyForState(irNavState);
  motorDoc["speedA_cmd"] = speedA;
  motorDoc["speedB_cmd"] = speedB;
  motorDoc["speedA_applied"] = speedA;
  motorDoc["speedB_applied"] = speedB;
  motorDoc["pwm_direct"] = true;

  JsonObject dataObj = motorDoc["data"].to<JsonObject>();
  dataObj["nav_mode"] = navModeKeyForState(irNavState);
  dataObj["speedA_cmd"] = speedA;
  dataObj["speedB_cmd"] = speedB;
  dataObj["speedA_applied"] = speedA;
  dataObj["speedB_applied"] = speedB;
  dataObj["pwm_direct"] = true;

  String motorPayload;
  serializeJson(motorDoc, motorPayload);
  publishJson(topicFor("state", "motor"), motorPayload, false);

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

  pushTelemetrySample(TELEMETRY_KIND_MOTOR, static_cast<uint8_t>(irNavState),
                      static_cast<int8_t>(constrain(speedA, -127, 127)),
                      static_cast<int8_t>(constrain(speedB, -127, 127)),
                      static_cast<int32_t>((speedA << 16) | (speedB & 0xFFFF)));
}

static void publishMissionState() {
  JsonDocument missionDoc;
  setBaseMessageFields(missionDoc, "state", "mission", PROTOCOL_ACTIVE_VERSION);
  missionDoc["nav_mode"] = navModeKeyForState(irNavState);
  missionDoc["status"] = g_mission.status;

  JsonObject missionObj = missionDoc["mission"].to<JsonObject>();
  missionObj["active"] = g_mission.active;
  missionObj["updated_at_ms"] = g_mission.updatedAt;
  if (!g_mission.missionId.isEmpty()) {
    missionObj["mission_id"] = g_mission.missionId;
  }

  JsonObject dataObj = missionDoc["data"].to<JsonObject>();
  dataObj["nav_mode"] = navModeKeyForState(irNavState);
  dataObj["status"] = g_mission.status;
  JsonObject dataMission = dataObj["mission"].to<JsonObject>();
  dataMission["active"] = g_mission.active;
  dataMission["updated_at_ms"] = g_mission.updatedAt;
  if (!g_mission.missionId.isEmpty()) {
    dataMission["mission_id"] = g_mission.missionId;
  }

  String payload;
  serializeJson(missionDoc, payload);
  publishJson(topicFor("state", "mission"), payload, true);
}

static void publishDecisionInfo(const char* reason, const char* detail) {
  JsonDocument decisionDoc;
  setBaseMessageFields(decisionDoc, "state", "decision", PROTOCOL_ACTIVE_VERSION);
  decisionDoc["nav_mode"] = navModeKeyForState(irNavState);
  decisionDoc["reason"] = reason;
  if (detail != nullptr && detail[0] != '\0') {
    decisionDoc["detail"] = detail;
  }

  JsonObject dataObj = decisionDoc["data"].to<JsonObject>();
  dataObj["nav_mode"] = navModeKeyForState(irNavState);
  dataObj["reason"] = reason;
  if (detail != nullptr && detail[0] != '\0') {
    dataObj["detail"] = detail;
  }

  String payload;
  serializeJson(decisionDoc, payload);
  publishJson(topicFor("telemetry", "decision"), payload, false);

  int32_t reasonHash = fnv1aHash32(String(reason));
  int32_t detailHash = detail != nullptr ? fnv1aHash32(String(detail)) : 0;
  pushTelemetrySample(TELEMETRY_KIND_DECISION, static_cast<uint8_t>(irNavState),
                      static_cast<int8_t>(reasonHash & 0x7F),
                      static_cast<int8_t>(detailHash & 0x7F),
                      reasonHash ^ detailHash);
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
  setBaseMessageFields(stateDoc, "state", "device", PROTOCOL_ACTIVE_VERSION);
  stateDoc["fw_version"] = __DATE__ " " __TIME__;
  stateDoc["uptime_ms"] = millis();
  stateDoc["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
  stateDoc["nav_mode"] = navModeKeyForState(irNavState);
  stateDoc["client_id"] = g_clientId;
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
  mqttDoc["protocol_ver"] = PROTOCOL_ACTIVE_VERSION;

  JsonObject healthDoc = stateDoc["health"].to<JsonObject>();
  healthDoc["camera_ok"] = cameraAvailable;
  healthDoc["ir_ok"] = true;
  healthDoc["imu_ok"] = false;

  JsonObject protocolDoc = stateDoc["protocol"].to<JsonObject>();
  protocolDoc["active_ver"] = PROTOCOL_ACTIVE_VERSION;
  protocolDoc["compat_min_ver"] = PROTOCOL_COMPAT_MIN_VERSION;
  protocolDoc["schema_ver"] = PROTOCOL_SCHEMA_VERSION;
  protocolDoc["telemetry_batch_ms"] = g_telemetryBatchIntervalMs;
  protocolDoc["telemetry_max_samples"] = g_telemetryBatchMaxSamples;
  protocolDoc["http_telemetry_enabled"] = g_httpTelemetryEnabled;

  JsonObject dataObj = stateDoc["data"].to<JsonObject>();
  dataObj["fw_version"] = __DATE__ " " __TIME__;
  dataObj["uptime_ms"] = millis();
  dataObj["stream_url"] = "http://" + WiFi.localIP().toString() + "/stream";
  dataObj["nav_mode"] = navModeKeyForState(irNavState);
  dataObj["client_id"] = g_clientId;
  if (extraMsg != nullptr && extraMsg[0] != '\0') {
    dataObj["msg"] = extraMsg;
  }

  String statePayload;
  serializeJson(stateDoc, statePayload);
  publishJson(topicFor("state", "device"), statePayload, true);

  pushTelemetrySample(TELEMETRY_KIND_HEALTH, static_cast<uint8_t>(irNavState),
                      static_cast<int8_t>(cameraAvailable ? 1 : 0),
                      static_cast<int8_t>(mqttClient.connected() ? 1 : 0),
                      WiFi.RSSI());
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
  setBaseMessageFields(perceptionDoc, "state", "perception", PROTOCOL_ACTIVE_VERSION);
  perceptionDoc["nav_mode"] = navModeKeyForState(irNavState);

  JsonObject irDoc = perceptionDoc["ir"].to<JsonObject>();
  irDoc["sensor_id"] = sensorId;
  irDoc["direction"] = direction;
  irDoc["angle_deg"] = angle;

  JsonObject dataObj = perceptionDoc["data"].to<JsonObject>();
  dataObj["nav_mode"] = navModeKeyForState(irNavState);
  JsonObject dataIr = dataObj["ir"].to<JsonObject>();
  dataIr["sensor_id"] = sensorId;
  dataIr["direction"] = direction;
  dataIr["angle_deg"] = angle;

  String perceptionPayload;
  serializeJson(perceptionDoc, perceptionPayload);
  publishJson(topicFor("telemetry", "perception"), perceptionPayload, false);

  pushTelemetrySample(TELEMETRY_KIND_IR, static_cast<uint8_t>(irNavState),
                      static_cast<int8_t>(constrain(sensorId, -127, 127)),
                      static_cast<int8_t>(constrain(angle, -127, 127)),
                      fnv1aHash32(String(direction)));
}

static void applyDirectMotorCommand(int requestedSpeedA, int requestedSpeedB) {
  extern int speedA, speedB;

  speedA = constrain(requestedSpeedA, -255, 255);
  speedB = constrain(requestedSpeedB, -255, 255);
  webRequestedNavState = STATE_MANUAL;
  setNavMode(STATE_MANUAL);
  motor_control(0, speedA);
  motor_control(1, speedB);

  publishMotorState(true);
  publishDecisionInfo("direct_pwm", "manual motor command applied");
}

static bool runOtaUpdate(const String& otaUrl) {
  if (otaUrl.isEmpty()) {
    publishJson(TOPIC_OTA_INFO, "OTA URL为空", false);
    return false;
  }

  publishJson(TOPIC_OTA_INFO, String("开始OTA升级，URL:") + otaUrl, false);
  WiFiClient otaClient;
  t_httpUpdate_return ret = httpUpdate.update(otaClient, otaUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      publishJson(TOPIC_OTA_INFO, String("OTA失败: ") + httpUpdate.getLastErrorString(), false);
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
  feedbackDoc["protocol_ver"] = PROTOCOL_ACTIVE_VERSION;

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

  Serial.printf("MQTT configured, device_id=%s\n", g_deviceId.c_str());
  Serial.printf("MQTT broker host=%s, port=%d\n", mqttServer, mqttPort);
  Serial.printf("Protocol active v%d, compat >= v%d\n", PROTOCOL_ACTIVE_VERSION, PROTOCOL_COMPAT_MIN_VERSION);
}

bool mqtt_reconnect() {
  ensureDeviceIdentity();

  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    attempts++;
    Serial.print("尝试MQTT连接...");

    g_clientId = String("ESP32-CAM-") + g_deviceId;
    if (mqttClient.connect(g_clientId.c_str())) {
      Serial.println("已连接到MQTT服务器");

      mqttClient.subscribe(TOPIC_MOTOR);
      mqttClient.subscribe(TOPIC_STREAM_CONFIG);
      mqttClient.subscribe(TOPIC_RESTART);
      mqttClient.subscribe(TOPIC_OTA);
      mqttClient.subscribe(TOPIC_CHECK_MQTT);
      mqttClient.subscribe(TOPIC_MOTOR_CONTROL);
      mqttClient.subscribe(TOPIC_NAVIGATION);

      mqttClient.subscribe(topicForVersion(1, "cmd", "motor").c_str());
      mqttClient.subscribe(topicForVersion(1, "cmd", "nav_mode").c_str());
      mqttClient.subscribe(topicForVersion(1, "cmd", "mission").c_str());
      mqttClient.subscribe(topicForVersion(1, "cmd", "ota").c_str());
      mqttClient.subscribe(topicForVersion(1, "cmd", "restart").c_str());
      mqttClient.subscribe(topicForVersion(1, "cmd", "protocol_cfg").c_str());

      mqttClient.subscribe(topicForVersion(2, "cmd", "motor").c_str());
      mqttClient.subscribe(topicForVersion(2, "cmd", "nav_mode").c_str());
      mqttClient.subscribe(topicForVersion(2, "cmd", "mission").c_str());
      mqttClient.subscribe(topicForVersion(2, "cmd", "ota").c_str());
      mqttClient.subscribe(topicForVersion(2, "cmd", "restart").c_str());
      mqttClient.subscribe(topicForVersion(2, "cmd", "protocol_cfg").c_str());

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
  pumpProtocolRuntime();
}

static bool resolveUnifiedCommandTopic(const char* topic, uint8_t* protocolVer, String* cmdName) {
  struct TopicEntry {
    uint8_t ver;
    const char* name;
  };

  static const TopicEntry entries[] = {
    {1, "motor"}, {1, "nav_mode"}, {1, "mission"}, {1, "ota"}, {1, "restart"}, {1, "protocol_cfg"},
    {2, "motor"}, {2, "nav_mode"}, {2, "mission"}, {2, "ota"}, {2, "restart"}, {2, "protocol_cfg"}
  };

  for (const auto& entry : entries) {
    if (isTopic(topic, topicForVersion(entry.ver, "cmd", entry.name))) {
      if (protocolVer != nullptr) {
        *protocolVer = entry.ver;
      }
      if (cmdName != nullptr) {
        *cmdName = entry.name;
      }
      return true;
    }
  }

  return false;
}

static bool handleLegacyTopic(const char* topic, const uint8_t* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_MOTOR) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("JSON解析失败: ");
      Serial.println(error.c_str());
      return true;
    }

    if (doc["clientId"].is<const char*>()) {
      String targetId = doc["clientId"].as<const char*>();
      if (targetId != g_clientId) {
        Serial.printf("忽略非本机指令，目标clientId=%s，本机clientId=%s\n",
                      targetId.c_str(), g_clientId.c_str());
        return true;
      }
    } else {
      Serial.println("未指定clientId，忽略指令");
      return true;
    }

    if (doc["speedA"].is<int>() && doc["speedB"].is<int>()) {
      applyDirectMotorCommand(doc["speedA"].as<int>(), doc["speedB"].as<int>());
    } else {
      Serial.println("JSON缺少speedA/speedB或类型错误");
    }
    return true;
  }

  if (strcmp(topic, TOPIC_NAVIGATION) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error || !doc["command"].is<const char*>()) {
      return true;
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
      Serial.printf("收到未知导航命令: %s\n", cmd.c_str());
    }
    return true;
  }

  if (strcmp(topic, TOPIC_CHECK_MQTT) == 0) {
    publishJson(TOPIC_CHECK_MQTT_REPLY, "{\"msg\":\"pong\"}", false);
    publishStatusInfo("MQTT连接检查响应");
    return true;
  }

  if (strcmp(topic, TOPIC_OTA) == 0) {
    String otaUrl;
    otaUrl.reserve(length);
    for (unsigned int i = 0; i < length; ++i) {
      otaUrl += static_cast<char>(payload[i]);
    }
    otaUrl.trim();
    runOtaUpdate(otaUrl);
    return true;
  }

  if (strcmp(topic, TOPIC_RESTART) == 0) {
    publishJson(TOPIC_OTA_INFO, "即将重启ESP32", false);
    delay(1000);
    ESP.restart();
    return true;
  }

  if (strcmp(topic, TOPIC_MOTOR_CONTROL) == 0) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    if (!error && doc["key"].is<const char*>()) {
      String key = doc["key"].as<const char*>();
      if (key == "shutdown") {
        applyDirectMotorCommand(0, 0);
      }
    }
    return true;
  }

  return false;
}

static CommandResult executeCommand(const String& cmdName, const JsonDocument& doc) {
  CommandResult result;

  if (cmdName == "motor") {
    String mode = commandString(doc, "mode");
    if (mode.isEmpty()) {
      mode = "direct_pwm";
    }
    if (mode != "direct_pwm") {
      result.code = "UNSUPPORTED_MODE";
      result.detail = "only direct_pwm is implemented";
      return result;
    }

    int speedA = 0;
    int speedB = 0;
    if (!commandInt(doc, "speedA", &speedA) || !commandInt(doc, "speedB", &speedB)) {
      result.code = "BAD_PARAM_RANGE";
      result.detail = "missing speedA or speedB";
      return result;
    }

    if (speedA < -255 || speedA > 255 || speedB < -255 || speedB > 255) {
      result.code = "BAD_PARAM_RANGE";
      result.detail = "speedA/speedB out of range";
      return result;
    }

    applyDirectMotorCommand(speedA, speedB);
    result.ok = true;
    result.code = "OK";
    result.detail = "motor command applied";
    return result;
  }

  if (cmdName == "nav_mode") {
    String mode = commandString(doc, "mode");
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
      result.code = "BAD_PARAM_RANGE";
      result.detail = "unknown nav mode";
      return result;
    }

    publishDecisionInfo("nav_mode", mode.c_str());
    result.ok = true;
    result.code = "OK";
    result.detail = "nav mode updated";
    return result;
  }

  if (cmdName == "mission") {
    String action = commandString(doc, "action");
    if (action.isEmpty()) {
      result.code = "BAD_PARAM_RANGE";
      result.detail = "missing action";
      return result;
    }

    JsonObjectConst missionObj;
    if (doc["data"].is<JsonObjectConst>()) {
      missionObj = doc["data"].as<JsonObjectConst>()["mission"].as<JsonObjectConst>();
    }
    if (missionObj.isNull()) {
      missionObj = doc["mission"].as<JsonObjectConst>();
    }

    if (action == "start") {
      if (missionObj.isNull() || !missionObj["mission_id"].is<const char*>()) {
        result.code = "BAD_PARAM_RANGE";
        result.detail = "mission.mission_id is required";
        return result;
      }

      g_mission.active = true;
      g_mission.missionId = missionObj["mission_id"].as<const char*>();
      g_mission.status = "accepted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MISSION;
      setNavMode(STATE_MISSION);
      publishMissionState();
      publishDecisionInfo("mission", "mission accepted");
      result.ok = true;
      result.code = "OK";
      result.detail = "mission accepted";
      return result;
    }

    if (!g_mission.active && action != "abort") {
      result.code = "MODE_CONFLICT";
      result.detail = "no active mission";
      return result;
    }

    if (action == "pause") {
      g_mission.status = "paused";
      g_mission.updatedAt = millis();
      publishMissionState();
      publishDecisionInfo("mission", "paused");
      result.ok = true;
      result.code = "OK";
      result.detail = "mission paused";
      return result;
    }

    if (action == "resume") {
      g_mission.status = "accepted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MISSION;
      setNavMode(STATE_MISSION);
      publishMissionState();
      publishDecisionInfo("mission", "resumed");
      result.ok = true;
      result.code = "OK";
      result.detail = "mission resumed";
      return result;
    }

    if (action == "abort") {
      g_mission.active = false;
      g_mission.status = "aborted";
      g_mission.updatedAt = millis();
      webRequestedNavState = STATE_MANUAL;
      setNavMode(STATE_MANUAL);
      publishMissionState();
      publishDecisionInfo("mission", "aborted");
      result.ok = true;
      result.code = "OK";
      result.detail = "mission aborted";
      return result;
    }

    result.code = "BAD_PARAM_RANGE";
    result.detail = "unknown mission action";
    return result;
  }

  if (cmdName == "ota") {
    String url = commandString(doc, "url");
    if (url.isEmpty()) {
      result.code = "BAD_PARAM_RANGE";
      result.detail = "missing url";
      return result;
    }

    result.ok = true;
    result.code = "OK";
    result.detail = "ota command accepted";
    result.otaUrl = url;
    return result;
  }

  if (cmdName == "restart") {
    int delayMs = 1000;
    commandInt(doc, "delay_ms", &delayMs);
    delayMs = constrain(delayMs, 0, 10000);

    result.ok = true;
    result.code = "OK";
    result.detail = "restart scheduled";
    result.restartDelayMs = delayMs;
    return result;
  }

  if (cmdName == "protocol_cfg") {
    JsonObjectConst cfgObj;
    if (doc["data"].is<JsonObjectConst>()) {
      cfgObj = doc["data"].as<JsonObjectConst>()["telemetry"].as<JsonObjectConst>();
    }
    if (cfgObj.isNull()) {
      cfgObj = doc["telemetry"].as<JsonObjectConst>();
    }

    if (!cfgObj.isNull()) {
      if (cfgObj["batch_ms"].is<int>()) {
        g_telemetryBatchIntervalMs = static_cast<uint16_t>(constrain(cfgObj["batch_ms"].as<int>(), 300, 10000));
      }
      if (cfgObj["max_samples"].is<int>()) {
        g_telemetryBatchMaxSamples = static_cast<uint8_t>(constrain(cfgObj["max_samples"].as<int>(), 4, TELEMETRY_MAX_SAMPLES_PER_BATCH));
      }
      if (cfgObj["http_enable"].is<bool>()) {
        g_httpTelemetryEnabled = cfgObj["http_enable"].as<bool>();
      }
      if (cfgObj["http_url"].is<const char*>()) {
        g_httpTelemetryUrl = cfgObj["http_url"].as<const char*>();
      }
    }

    result.ok = true;
    result.code = "OK";
    result.detail = "protocol config updated";
    publishStatusInfo("protocol config updated");
    return result;
  }

  result.code = "BAD_PARAM_RANGE";
  result.detail = "unknown command name";
  return result;
}

static bool dispatchUnifiedCommand(const String& cmdName, uint8_t topicProtocolVer, const uint8_t* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    publishAck("", false, "BAD_JSON", "payload is not valid json", topicProtocolVer, "", false, false);
    return true;
  }

  uint8_t payloadProtocolVer = parseProtocolVersion(doc);
  uint8_t effectiveProtocolVer = payloadProtocolVer >= topicProtocolVer ? payloadProtocolVer : topicProtocolVer;

  String msgId = commandString(doc, "msg_id");
  if (msgId.isEmpty()) {
    msgId = String("auto-") + millis();
  }

  int cachedIdx = findCachedAck(msgId);
  if (cachedIdx >= 0) {
    publishAck(msgId,
               g_cmdCache[cachedIdx].ok,
               g_cmdCache[cachedIdx].code,
               g_cmdCache[cachedIdx].detail,
               g_cmdCache[cachedIdx].protocolVer,
               g_cmdCache[cachedIdx].clientId,
               true,
               g_cmdCache[cachedIdx].retryable);
    return true;
  }

  String detail;
  if (!validateNewProtocolTarget(doc, effectiveProtocolVer, &detail)) {
    publishAck(msgId, false, "BAD_DEVICE_ID", detail, effectiveProtocolVer, "", false, false);
    rememberAck(msgId, false, "BAD_DEVICE_ID", detail, effectiveProtocolVer, "", false);
    return true;
  }

  if (!validateTtlWindow(doc, &detail)) {
    publishAck(msgId, false, "CMD_EXPIRED", detail, effectiveProtocolVer, "", false, false);
    rememberAck(msgId, false, "CMD_EXPIRED", detail, effectiveProtocolVer, "", false);
    return true;
  }

  String clientId;
  AuthRole role = AUTH_ROLE_NONE;
  String authCode;
  if (!checkCommandAuthorization(doc, cmdName, effectiveProtocolVer, &clientId, &role, &authCode, &detail)) {
    publishAck(msgId, false, authCode, detail, effectiveProtocolVer, clientId, false, false);
    rememberAck(msgId, false, authCode, detail, effectiveProtocolVer, clientId, false);
    return true;
  }

  CommandResult result = executeCommand(cmdName, doc);
  publishAck(msgId, result.ok, result.code, result.detail, effectiveProtocolVer, clientId, false, result.retryable);
  rememberAck(msgId, result.ok, result.code, result.detail, effectiveProtocolVer, clientId, result.retryable);

  if (result.ok && !result.otaUrl.isEmpty()) {
    runOtaUpdate(result.otaUrl);
  }

  if (result.ok && result.restartDelayMs >= 0 && cmdName == "restart") {
    publishJson(TOPIC_OTA_INFO, "即将重启ESP32", false);
    delay(result.restartDelayMs);
    ESP.restart();
  }

  return true;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  ensureDeviceIdentity();
  Serial.printf("收到MQTT消息: topic=%s len=%u\n", topic, length);

  if (length < 256) {
    char message[256];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("消息内容: %s\n", message);
  } else {
    Serial.println("消息较长，省略打印");
  }

  if (handleLegacyTopic(topic, payload, length)) {
    return;
  }

  uint8_t protocolVer = PROTOCOL_COMPAT_MIN_VERSION;
  String cmdName;
  if (resolveUnifiedCommandTopic(topic, &protocolVer, &cmdName)) {
    dispatchUnifiedCommand(cmdName, protocolVer, payload, length);
    return;
  }
}
