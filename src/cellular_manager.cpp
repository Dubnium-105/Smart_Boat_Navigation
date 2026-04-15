#include "cellular_manager.h"

#include "cellular_parser.h"
#include "esp_camera_pins.h"

#include <cstdarg>
#include <cstdio>
#include <cctype>

// Air780E/YED-M780-C default UART mapping.
// Use free GPIOs on this board profile that do not overlap with camera/IR/motor pins.
#ifndef CELLULAR_UART_BAUD
#define CELLULAR_UART_BAUD 115200
#endif

#ifndef CELLULAR_UART_RX_PIN
#define CELLULAR_UART_RX_PIN 1
#endif

#ifndef CELLULAR_UART_TX_PIN
#define CELLULAR_UART_TX_PIN 2
#endif

#ifndef CELLULAR_PWRKEY_PIN
#define CELLULAR_PWRKEY_PIN -1
#endif

#ifndef CELLULAR_PWRKEY_ACTIVE_LEVEL
#define CELLULAR_PWRKEY_ACTIVE_LEVEL 0
#endif

#ifndef CELLULAR_PWRKEY_PULSE_MS
#define CELLULAR_PWRKEY_PULSE_MS 1200
#endif

#ifndef CELLULAR_RESET_PIN
#define CELLULAR_RESET_PIN -1
#endif

#ifndef CELLULAR_RESET_ACTIVE_LEVEL
#define CELLULAR_RESET_ACTIVE_LEVEL 0
#endif

#ifndef CELLULAR_RESET_PULSE_MS
#define CELLULAR_RESET_PULSE_MS 180
#endif

#ifndef CELLULAR_STATUS_PIN
#define CELLULAR_STATUS_PIN -1
#endif

#ifndef CELLULAR_STATUS_ACTIVE_LEVEL
#define CELLULAR_STATUS_ACTIVE_LEVEL 1
#endif

namespace {

HardwareSerial SerialAT(1);
CellularStatus g_status = {false, false, false, -1, ""};
unsigned long g_lastReconnectAttempt = 0;
bool g_cellularLogEnabled = true;
bool g_uartStarted = false;
bool g_firstPowerOnWaitDone = false;
uint32_t g_currentBaud = CELLULAR_UART_BAUD;
bool g_powerKeySequenceDone = false;
bool g_controlPinsInitialized = false;

const uint32_t kProbeBauds[] = {115200};

bool pinMatchesCameraBus(int pin) {
  return pin == Y7_GPIO_NUM || pin == Y8_GPIO_NUM || pin == Y9_GPIO_NUM ||
         pin == Y6_GPIO_NUM || pin == Y5_GPIO_NUM || pin == Y4_GPIO_NUM ||
         pin == Y3_GPIO_NUM || pin == Y2_GPIO_NUM || pin == PCLK_GPIO_NUM ||
         pin == VSYNC_GPIO_NUM || pin == HREF_GPIO_NUM || pin == XCLK_GPIO_NUM ||
         pin == SIOD_GPIO_NUM || pin == SIOC_GPIO_NUM;
}

bool uartPinsConflictWithCamera() {
  return pinMatchesCameraBus(CELLULAR_UART_RX_PIN) || pinMatchesCameraBus(CELLULAR_UART_TX_PIN);
}

int idleLevelForActiveLevel(int activeLevel) {
  return activeLevel ? LOW : HIGH;
}

void logCellular(const char* fmt, ...) {
  if (!g_cellularLogEnabled) {
    return;
  }

  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.printf("[Air780E] %s\n", buf);
}

String sanitizeForLog(const String& input) {
  String out;
  out.reserve(input.length() * 4);
  for (size_t i = 0; i < input.length(); ++i) {
    unsigned char c = static_cast<unsigned char>(input[i]);
    if (c == '\r' || c == '\n' || c == '\t' || (c >= 32 && c <= 126)) {
      out += static_cast<char>(c);
    } else {
      char hexBuf[6];
      snprintf(hexBuf, sizeof(hexBuf), "\\x%02X", c);
      out += hexBuf;
    }
  }
  return out;
}

float printableRatio(const String& input) {
  if (input.isEmpty()) {
    return 1.0f;
  }

  size_t printable = 0;
  for (size_t i = 0; i < input.length(); ++i) {
    unsigned char c = static_cast<unsigned char>(input[i]);
    if (c == '\r' || c == '\n' || c == '\t' || (c >= 32 && c <= 126)) {
      printable++;
    }
  }
  return static_cast<float>(printable) / static_cast<float>(input.length());
}

String readUntilTimeout(uint32_t timeoutMs) {
  String response;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = static_cast<char>(SerialAT.read());
      response += c;
    }
    delay(5);
  }
  return response;
}

String trimAtResponse(const String& input) {
  String out = input;
  out.trim();
  return out;
}

bool responseLooksLikeEchoOnly(const String& response, const char* cmd) {
  String trimmed = trimAtResponse(response);
  if (trimmed.isEmpty()) {
    return false;
  }

  String sent = String(cmd);
  sent.trim();
  return trimmed.equalsIgnoreCase(sent);
}

void logResponseHints(const String& response, const char* cmd) {
  if (response.isEmpty()) {
    logCellular("RX empty (no reply before timeout)");
    return;
  }

  if (responseLooksLikeEchoOnly(response, cmd)) {
    logCellular(
        "RX is echo-only for '%s' (likely TX/RX loopback, wrong UART port, or module echo without final OK)",
        cmd);
    return;
  }

  if (printableRatio(response) < 0.25f) {
    logCellular("RX looks binary (likely wrong UART pins/level or not AT port)");
  }
}

void initializeControlPinsIfNeeded() {
  if (g_controlPinsInitialized) {
    return;
  }
  g_controlPinsInitialized = true;

#if CELLULAR_PWRKEY_PIN >= 0
  pinMode(CELLULAR_PWRKEY_PIN, OUTPUT);
  digitalWrite(CELLULAR_PWRKEY_PIN, idleLevelForActiveLevel(CELLULAR_PWRKEY_ACTIVE_LEVEL));
#endif

#if CELLULAR_RESET_PIN >= 0
  pinMode(CELLULAR_RESET_PIN, OUTPUT);
  digitalWrite(CELLULAR_RESET_PIN, idleLevelForActiveLevel(CELLULAR_RESET_ACTIVE_LEVEL));
#endif

#if CELLULAR_STATUS_PIN >= 0
  pinMode(CELLULAR_STATUS_PIN, INPUT);
#endif
}

void pulseControlPin(const char* label, int pin, int activeLevel, uint32_t pulseMs) {
  if (pin < 0) {
    return;
  }

  digitalWrite(pin, activeLevel ? HIGH : LOW);
  delay(pulseMs);
  digitalWrite(pin, idleLevelForActiveLevel(activeLevel));
  logCellular("%s pulse done pin=%d pulse_ms=%lu", label, pin,
              static_cast<unsigned long>(pulseMs));
}

bool hasStatusPin() {
  return CELLULAR_STATUS_PIN >= 0;
}

bool hasPowerKeyPin() {
  return CELLULAR_PWRKEY_PIN >= 0;
}

bool hasResetPin() {
  return CELLULAR_RESET_PIN >= 0;
}

bool isStatusPinActive() {
#if CELLULAR_STATUS_PIN >= 0
  return digitalRead(CELLULAR_STATUS_PIN) ==
         (CELLULAR_STATUS_ACTIVE_LEVEL ? HIGH : LOW);
#else
  return false;
#endif
}

void logStatusPinSnapshot(const char* stage) {
  if (!hasStatusPin()) {
    return;
  }

  logCellular("status pin snapshot stage=%s pin=%d raw=%d active=%d", stage,
              CELLULAR_STATUS_PIN, digitalRead(CELLULAR_STATUS_PIN),
              isStatusPinActive() ? 1 : 0);
}

bool sendAT(const char* cmd, const char* expected, uint32_t timeoutMs = 800) {
  while (SerialAT.available()) {
    SerialAT.read();
  }

  logCellular("TX: %s", cmd);
  SerialAT.println(cmd);
  String response = readUntilTimeout(timeoutMs);
  String logLine = sanitizeForLog(response);
  logCellular("RX: %s", logLine.c_str());
  logResponseHints(response, cmd);
  return response.indexOf(expected) >= 0;
}

String queryAT(const char* cmd, uint32_t timeoutMs = 800) {
  while (SerialAT.available()) {
    SerialAT.read();
  }
  logCellular("TX: %s", cmd);
  SerialAT.println(cmd);
  String response = readUntilTimeout(timeoutMs);
  String logLine = sanitizeForLog(response);
  logCellular("RX: %s", logLine.c_str());
  logResponseHints(response, cmd);
  return response;
}

void runPowerKeySequenceIfNeeded() {
  if (g_powerKeySequenceDone) {
    return;
  }
  g_powerKeySequenceDone = true;

  if (hasStatusPin() && isStatusPinActive()) {
    logCellular("status pin already indicates module powered on, skip initial PWRKEY pulse");
    return;
  }

#if CELLULAR_PWRKEY_PIN >= 0
  pulseControlPin("pwrkey", CELLULAR_PWRKEY_PIN, CELLULAR_PWRKEY_ACTIVE_LEVEL,
                  CELLULAR_PWRKEY_PULSE_MS);
#else
  logCellular("PWRKEY not configured; relying on external auto power-on");
#endif
}

void runResetSequence() {
#if CELLULAR_RESET_PIN >= 0
  pulseControlPin("reset", CELLULAR_RESET_PIN, CELLULAR_RESET_ACTIVE_LEVEL,
                  CELLULAR_RESET_PULSE_MS);
#else
  logCellular("RESET not configured; cannot trigger hardware reset");
#endif
}

void triggerRecoverySequence() {
  logStatusPinSnapshot("before_recovery");

  if (hasResetPin() && (hasPowerKeyPin() || !hasStatusPin() || isStatusPinActive())) {
    runResetSequence();
    delay(300);
    g_powerKeySequenceDone = false;
  } else if (hasResetPin()) {
    logCellular(
        "skip reset recovery: status pin says module is off and no PWRKEY is available to power it back on");
  }

  if (hasPowerKeyPin()) {
    runPowerKeySequenceIfNeeded();
    delay(2500);
  }

  logStatusPinSnapshot("after_recovery");
}

bool probeAtCurrentBaud() {
  for (int i = 0; i < 3; ++i) {
    if (sendAT("AT", "OK", 1200)) {
      return true;
    }
    delay(120);
  }
  return false;
}

bool beginAndProbeBaud(uint32_t baud) {
  g_currentBaud = baud;
  SerialAT.begin(baud, SERIAL_8N1, CELLULAR_UART_RX_PIN, CELLULAR_UART_TX_PIN);
  g_uartStarted = true;
  delay(120);
  logCellular("serial started baud=%lu rx=%d tx=%d", static_cast<unsigned long>(baud),
              CELLULAR_UART_RX_PIN, CELLULAR_UART_TX_PIN);
  return probeAtCurrentBaud();
}

void refreshCellularStatus() {
  String cereg = queryAT("AT+CEREG?", 900);
  cellular::CeregStat regStat = cellular::parseCeregStat(std::string(cereg.c_str()));
  g_status.registered = cellular::isRegistered(regStat);

  String cgatt = queryAT("AT+CGATT?", 800);
  g_status.dataAttached =
      (cellular::parseCgattStat(std::string(cgatt.c_str())) == cellular::CgattStat::Attached);

  String csq = queryAT("AT+CSQ", 600);
  g_status.rssi = cellular::parseSignalQuality(std::string(csq.c_str()));

  String cops = queryAT("AT+COPS?", 900);
  std::string oper = cellular::parseOperatorName(std::string(cops.c_str()));
  g_status.operatorName = oper.c_str();

  logCellular("status: moduleReady=%d registered=%d attached=%d rssi=%d operator=%s",
              g_status.moduleReady ? 1 : 0, g_status.registered ? 1 : 0,
              g_status.dataAttached ? 1 : 0, g_status.rssi, g_status.operatorName.c_str());
}

}  // namespace

void setupCellular() {
  initializeControlPinsIfNeeded();
  runPowerKeySequenceIfNeeded();

  if (uartPinsConflictWithCamera()) {
    logCellular(
        "UART pin conflict: modem rx=%d tx=%d overlaps camera bus "
        "(current camera pins include Y7=%d Y8=%d Y9=%d)",
        CELLULAR_UART_RX_PIN, CELLULAR_UART_TX_PIN, Y7_GPIO_NUM, Y8_GPIO_NUM, Y9_GPIO_NUM);
  }

  if (!g_firstPowerOnWaitDone) {
    g_firstPowerOnWaitDone = true;
    // Air780E power-up may take a while before AT is responsive.
    delay(3000);
  }

  logStatusPinSnapshot("before_probe");

  bool ready = false;
  if (g_uartStarted) {
    ready = probeAtCurrentBaud();
  }
  if (!ready) {
    for (size_t i = 0; i < sizeof(kProbeBauds) / sizeof(kProbeBauds[0]); ++i) {
      if (beginAndProbeBaud(kProbeBauds[i])) {
        ready = true;
        break;
      }
    }
  }

  if (!ready && (hasPowerKeyPin() || hasResetPin())) {
    logCellular("no AT response, running control-line recovery");
    triggerRecoverySequence();

    if (g_uartStarted) {
      ready = probeAtCurrentBaud();
    }
    if (!ready) {
      for (size_t i = 0; i < sizeof(kProbeBauds) / sizeof(kProbeBauds[0]); ++i) {
        if (beginAndProbeBaud(kProbeBauds[i])) {
          ready = true;
          break;
        }
      }
    }
  }

  g_status.moduleReady = ready;
  if (!g_status.moduleReady) {
    logStatusPinSnapshot("probe_failed");
    logCellular(
        "module not ready (check VBAT, GND, TX/RX cross, 1.8V/3.3V level compatibility, PWRKEY)");
    return;
  }

  // Full-function mode, keep command compatible with most LTE CAT1 modules.
  sendAT("AT+CFUN=1", "OK", 1200);
  refreshCellularStatus();
}

bool connectCellular() {
  if (!g_status.moduleReady) {
    setupCellular();
  }
  if (!g_status.moduleReady) {
    logCellular("connect failed: module not ready");
    return false;
  }

  refreshCellularStatus();
  bool ready = g_status.registered && g_status.dataAttached;
  logCellular("connect result=%d", ready ? 1 : 0);
  return ready;
}

bool cellularAutoReconnect() {
  if (isCellularConnected()) {
    return true;
  }

  unsigned long now = millis();
  if (now - g_lastReconnectAttempt < 5000) {
    return false;
  }
  g_lastReconnectAttempt = now;
  logCellular("auto reconnect triggered");

  return connectCellular();
}

bool isCellularConnected() {
  return g_status.moduleReady && g_status.registered && g_status.dataAttached;
}

CellularStatus getCellularStatus() {
  return g_status;
}

void setCellularLogEnabled(bool enabled) {
  g_cellularLogEnabled = enabled;
  logCellular("log enabled=%d", enabled ? 1 : 0);
}
