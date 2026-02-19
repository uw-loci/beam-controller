#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

/*
  BCON_Mega_Logic.cpp
  ------------------------------------------------------------
  Beam Controller firmware for Arduino Mega driving up to 3x PVX-4140 pulsers.

  Key features:
  - RS485 command interface (half-duplex transceiver such as MAX485)
  - Three independent pulser gate outputs
  - Two output modes per channel: DC and timed PULSE
  - Hardware interlock enforcement (KB_Interlock)
  - Communication watchdog fail-safe (forces outputs LOW on timeout)
  - Status input monitoring per pulser:
      * Enable Status
      * Power Status
      * Over Current Status
      * Gated Status

  Notes from PVX-4140 docs integrated in this design:
  - Gate input expects logic-level +5V into 50 ohm.
  - Gate HIGH selects +HV to output, Gate LOW selects -HV.
  - Remote status outputs are active-low open-drain; use pull-ups when interfacing.

  ------------------------------------------------------------
  ADAPTATION CHECKLIST (edit in Config section below):
  1) Pin map (outputs, interlock, status inputs, RS485 DE/RE pin, power LED)
  2) Interlock polarity
  3) Watchdog timeout, telemetry interval, serial baud rate
  4) Pulse duration limits
  ------------------------------------------------------------
*/

namespace Config {

constexpr uint8_t CHANNEL_COUNT = 3;

// ---------------------------
// RS485 / Serial configuration
// ---------------------------
HardwareSerial &RS485_SERIAL = Serial1;   // Mega: Serial1 uses RX1=19, TX1=18
constexpr uint32_t RS485_BAUD = 115200;
constexpr uint8_t RS485_DE_RE_PIN = 2;    // Tie DE and /RE together on transceiver

// ---------------------------
// Core safety / behavior config
// ---------------------------
constexpr uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 1000;  // comm timeout -> outputs forced LOW
constexpr uint32_t DEFAULT_TELEMETRY_MS = 1000;         // periodic status reports (0 disables)
constexpr uint32_t HEARTBEAT_MIN_MS = 50;
constexpr uint32_t HEARTBEAT_MAX_MS = 60000;

// Interlock behavior.
// Plan requirement: HIGH = allowed to enable outputs, LOW = force outputs LOW.
constexpr uint8_t KB_INTERLOCK_PIN = 22;
constexpr bool INTERLOCK_ACTIVE_HIGH = true;
constexpr bool INTERLOCK_USE_PULLUP = false;  // set true only if your wiring requires it

// External power LED output (fed when Arduino powered)
constexpr uint8_t POWER_LED_PIN = 13;

// ---------------------------
// Pulser gate outputs (to MOSFET gate-drive interface)
// ---------------------------
constexpr uint8_t PULSER_GATE_OUTPUT_PINS[CHANNEL_COUNT] = {5, 6, 7};

// ---------------------------
// Pulser status inputs (active-low open-drain from PVX-4140 remote interface)
// Requires external pull-up network if not using INPUT_PULLUP.
// ---------------------------
constexpr bool STATUS_USE_INTERNAL_PULLUPS = true;

constexpr uint8_t ENABLE_STATUS_PINS[CHANNEL_COUNT] = {23, 27, 31};
constexpr uint8_t POWER_STATUS_PINS[CHANNEL_COUNT] = {24, 28, 32};
constexpr uint8_t OVERCURRENT_STATUS_PINS[CHANNEL_COUNT] = {25, 29, 33};
constexpr uint8_t GATED_STATUS_PINS[CHANNEL_COUNT] = {26, 30, 34};

// ---------------------------
// Pulse duration limits
// ---------------------------
constexpr uint32_t PULSE_DURATION_MIN_MS = 1;
constexpr uint32_t PULSE_DURATION_MAX_MS = 60000;
constexpr uint32_t PULSE_COUNT_MIN = 1;
constexpr uint32_t PULSE_COUNT_MAX = 10000;

}  // namespace Config

namespace Runtime {

enum class OutputMode : uint8_t {
  Off = 0,
  DC,
  Pulse,
  PulseTrain
};

enum class TopLevelState : uint8_t {
  Ready = 0,
  SafeInterlock,
  SafeWatchdog,
  FaultLatched
};

struct ChannelControl {
  OutputMode mode = OutputMode::Off;
  uint32_t pulseDurationMs = 100;
  uint32_t pulseCount = 1;
  uint32_t pulsesRemaining = 0;
  uint32_t pulseStartMs = 0;
  bool pulseTrainInLowGap = false;
  bool lastLevel = false;
};

ChannelControl channels[Config::CHANNEL_COUNT];

uint32_t lastCommandMillis = 0;
uint32_t watchdogTimeoutMs = Config::DEFAULT_WATCHDOG_TIMEOUT_MS;
uint32_t telemetryPeriodMs = Config::DEFAULT_TELEMETRY_MS;
uint32_t lastTelemetryMillis = 0;
bool faultLatched = false;

constexpr size_t RX_BUFFER_SIZE = 120;
char rxBuffer[RX_BUFFER_SIZE] = {0};
size_t rxIndex = 0;

}  // namespace Runtime

// ---------------------------
// Helpers
// ---------------------------

static bool activeLowRead(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static bool isInterlockSatisfied() {
  const bool raw = (digitalRead(Config::KB_INTERLOCK_PIN) == HIGH);
  return Config::INTERLOCK_ACTIVE_HIGH ? raw : !raw;
}

static bool isWatchdogHealthy() {
  return (millis() - Runtime::lastCommandMillis) <= Runtime::watchdogTimeoutMs;
}

static bool isAnyOverCurrentAsserted() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    if (activeLowRead(Config::OVERCURRENT_STATUS_PINS[channel])) {
      return true;
    }
  }
  return false;
}

static Runtime::TopLevelState evaluateTopLevelState() {
  if (!isInterlockSatisfied()) {
    return Runtime::TopLevelState::SafeInterlock;
  }

  if (!isWatchdogHealthy()) {
    return Runtime::TopLevelState::SafeWatchdog;
  }

  if (Runtime::faultLatched) {
    return Runtime::TopLevelState::FaultLatched;
  }

  return Runtime::TopLevelState::Ready;
}

static const char *topLevelStateToString(Runtime::TopLevelState state) {
  switch (state) {
    case Runtime::TopLevelState::Ready: return "READY";
    case Runtime::TopLevelState::SafeInterlock: return "SAFE_INTERLOCK";
    case Runtime::TopLevelState::SafeWatchdog: return "SAFE_WATCHDOG";
    case Runtime::TopLevelState::FaultLatched: return "FAULT_LATCHED";
    default: return "UNKNOWN";
  }
}

static const char *topLevelStateReason(Runtime::TopLevelState state) {
  switch (state) {
    case Runtime::TopLevelState::SafeInterlock: return "INTERLOCK_LOW";
    case Runtime::TopLevelState::SafeWatchdog: return "WATCHDOG_EXPIRED";
    case Runtime::TopLevelState::FaultLatched: return "FAULT_LATCHED";
    case Runtime::TopLevelState::Ready:
    default:
      return "NONE";
  }
}

static const char *modeToString(Runtime::OutputMode mode) {
  switch (mode) {
    case Runtime::OutputMode::Off: return "OFF";
    case Runtime::OutputMode::DC:  return "DC";
    case Runtime::OutputMode::Pulse: return "PULSE";
    case Runtime::OutputMode::PulseTrain: return "PULSE_TRAIN";
    default: return "UNKNOWN";
  }
}

static bool parseUInt32(const char *text, uint32_t &valueOut) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPtr = nullptr;
  unsigned long parsed = strtoul(text, &endPtr, 10);
  if (*endPtr != '\0') {
    return false;
  }

  valueOut = static_cast<uint32_t>(parsed);
  return true;
}

static void rs485SetTransmit(bool enabled) {
  digitalWrite(Config::RS485_DE_RE_PIN, enabled ? HIGH : LOW);
}

static void rs485SendLine(const char *line) {
  rs485SetTransmit(true);
  Config::RS485_SERIAL.println(line);
  Config::RS485_SERIAL.flush();
  rs485SetTransmit(false);
}

static void sendOk(const char *detail = "OK") {
  char out[96];
  snprintf(out, sizeof(out), "OK %s", detail);
  rs485SendLine(out);
}

static void sendErr(const char *detail) {
  char out[96];
  snprintf(out, sizeof(out), "ERR %s", detail);
  rs485SendLine(out);
}

static void forceAllOutputsLow() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
    Runtime::channels[channel].lastLevel = false;
  }
}

static void applyOutputs() {
  const Runtime::TopLevelState state = evaluateTopLevelState();

  if (state != Runtime::TopLevelState::Ready) {
    forceAllOutputsLow();
    return;
  }

  /* 
    Iterate over channels and apply output levels based on mode and timing.
     This is called frequently from loop(), so we check mode and timing to
     determine if the output should be HIGH or LOW, and only update the
     pin if the level has changed since last time. 
  */
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    Runtime::ChannelControl &cfg = Runtime::channels[channel];
    bool level = LOW;

    switch (cfg.mode) {
      case Runtime::OutputMode::Off:
        level = LOW;
        break;

      case Runtime::OutputMode::DC:
        level = HIGH;
        break;

      case Runtime::OutputMode::Pulse: {
        const uint32_t nowMs = millis();
        const uint32_t elapsedMs = nowMs - cfg.pulseStartMs;
        if (elapsedMs < cfg.pulseDurationMs) {
          level = HIGH;
        } else {
          level = LOW;
          cfg.mode = Runtime::OutputMode::Off;
          cfg.pulsesRemaining = 0;
        }
        break;
      }

      case Runtime::OutputMode::PulseTrain: {
        if (cfg.pulsesRemaining == 0) {
          level = LOW;
          cfg.mode = Runtime::OutputMode::Off;
          cfg.pulseTrainInLowGap = false;
          break;
        }

        const uint32_t nowMs = millis();
        const uint32_t elapsedMs = nowMs - cfg.pulseStartMs;

        if (!cfg.pulseTrainInLowGap) {
          if (elapsedMs < cfg.pulseDurationMs) {
            level = HIGH;
          } else {
            cfg.pulsesRemaining--;
            if (cfg.pulsesRemaining == 0) {
              level = LOW;
              cfg.mode = Runtime::OutputMode::Off;
              cfg.pulseTrainInLowGap = false;
            } else {
              level = LOW;
              cfg.pulseTrainInLowGap = true;
              cfg.pulseStartMs = nowMs;
            }
          }
        } else {
          level = LOW;
          if (elapsedMs >= cfg.pulseDurationMs) {
            cfg.pulseTrainInLowGap = false;
            cfg.pulseStartMs = nowMs;
          }
        }
        break;
      }

      default:
        level = LOW;
        break;
    }

    if (level != cfg.lastLevel) {
      digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], level);
      cfg.lastLevel = level;
    }
  }
}

static void setChannelOff(uint8_t indexZeroBased) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::Off;
  cfg.pulsesRemaining = 0;
  cfg.pulseTrainInLowGap = false;
}

static void setChannelDC(uint8_t indexZeroBased) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::DC;
  cfg.pulsesRemaining = 0;
  cfg.pulseTrainInLowGap = false;
}

static void setChannelPulse(uint8_t indexZeroBased, uint32_t durationMs) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::Pulse;
  cfg.pulseDurationMs = durationMs;
  cfg.pulseCount = 1;
  cfg.pulsesRemaining = 1;
  cfg.pulseStartMs = millis();
  cfg.pulseTrainInLowGap = false;
}

static void setChannelPulseTrain(uint8_t indexZeroBased, uint32_t durationMs, uint32_t count) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::PulseTrain;
  cfg.pulseDurationMs = durationMs;
  cfg.pulseCount = count;
  cfg.pulsesRemaining = count;
  cfg.pulseStartMs = millis();
  cfg.pulseTrainInLowGap = false;
}

static void sendStatusReport() {
  char out[160];
  const Runtime::TopLevelState state = evaluateTopLevelState();

  snprintf(
    out,
    sizeof(out),
    "SYS state=%s reason=%s fault_latched=%u telemetry_ms=%lu",
    topLevelStateToString(state),
    topLevelStateReason(state),
    Runtime::faultLatched ? 1 : 0,
    static_cast<unsigned long>(Runtime::telemetryPeriodMs)
  );
  rs485SendLine(out);

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    const uint8_t c = channel + 1;
    const bool stEnable = activeLowRead(Config::ENABLE_STATUS_PINS[channel]);
    const bool stPower = activeLowRead(Config::POWER_STATUS_PINS[channel]);
    const bool stOverCurrent = activeLowRead(Config::OVERCURRENT_STATUS_PINS[channel]);
    const bool stGated = activeLowRead(Config::GATED_STATUS_PINS[channel]);

    snprintf(
      out,
      sizeof(out),
      "CH%u mode=%s pulse_ms=%lu en_st=%u pwr_st=%u oc_st=%u gated_st=%u",
      c,
      modeToString(Runtime::channels[channel].mode),
      static_cast<unsigned long>(Runtime::channels[channel].pulseDurationMs),
      stEnable ? 1 : 0,
      stPower ? 1 : 0,
      stOverCurrent ? 1 : 0,
      stGated ? 1 : 0
    );
    rs485SendLine(out);
  }
}

static bool equalsIgnoreCase(const char *a, const char *b) {
  return strcasecmp(a, b) == 0;
}

static uint8_t toChannelIndex(const char *token) {
  uint32_t n = 0;
  if (!parseUInt32(token, n)) {
    return 255;
  }
  if (n < 1 || n > Config::CHANNEL_COUNT) {
    return 255;
  }
  return static_cast<uint8_t>(n - 1);
}

static void processCommand(char *line) {
  char *tokens[8] = {nullptr};
  uint8_t tokenCount = 0;

  char *context = nullptr;
  char *tok = strtok_r(line, " \t", &context);
  while (tok != nullptr && tokenCount < 8) {
    tokens[tokenCount++] = tok;
    tok = strtok_r(nullptr, " \t", &context);
  }

  if (tokenCount == 0) {
    return;
  }

  // Treat any complete line as link activity (keeps watchdog alive).
  Runtime::lastCommandMillis = millis();

  if (equalsIgnoreCase(tokens[0], "PING")) {
    rs485SendLine("PONG");
    return;
  }

  if (equalsIgnoreCase(tokens[0], "HELP")) {
    rs485SendLine("CMD PING");
    rs485SendLine("CMD STATUS");
    rs485SendLine("CMD STOP ALL");
    rs485SendLine("CMD SET CH <1..3> OFF");
    rs485SendLine("CMD SET CH <1..3> DC");
    rs485SendLine("CMD SET CH <1..3> PULSE <duration_ms> [count]");
    rs485SendLine("CMD SET WATCHDOG <ms>");
    rs485SendLine("CMD SET TELEMETRY <ms|0=off>");
    rs485SendLine("CMD CLEAR FAULT");
    rs485SendLine("CMD ARM");
    return;
  }

  if (equalsIgnoreCase(tokens[0], "STATUS") ||
      (tokenCount >= 2 && equalsIgnoreCase(tokens[0], "GET") && equalsIgnoreCase(tokens[1], "STATUS"))) {
    sendStatusReport();
    return;
  }

  if (tokenCount >= 2 && equalsIgnoreCase(tokens[0], "STOP") && equalsIgnoreCase(tokens[1], "ALL")) {
    for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
      setChannelOff(channel);
    }
    sendOk("ALL_OFF");
    return;
  }

  if ((tokenCount >= 2 && equalsIgnoreCase(tokens[0], "CLEAR") && equalsIgnoreCase(tokens[1], "FAULT")) ||
      equalsIgnoreCase(tokens[0], "ARM")) {
    if (isAnyOverCurrentAsserted()) {
      sendErr("FAULT_STILL_ACTIVE");
      return;
    }

    if (!isInterlockSatisfied()) {
      sendErr("INTERLOCK_NOT_READY");
      return;
    }

    Runtime::faultLatched = false;
    sendOk("FAULT_CLEARED");
    return;
  }

  if (tokenCount >= 2 && equalsIgnoreCase(tokens[0], "SET")) {
    if (equalsIgnoreCase(tokens[1], "WATCHDOG")) {
      if (tokenCount < 3) {
        sendErr("SET WATCHDOG <ms>");
        return;
      }

      uint32_t ms = 0;
      if (!parseUInt32(tokens[2], ms)) {
        sendErr("INVALID_WATCHDOG");
        return;
      }

      if (ms < Config::HEARTBEAT_MIN_MS || ms > Config::HEARTBEAT_MAX_MS) {
        sendErr("WATCHDOG_RANGE");
        return;
      }

      Runtime::watchdogTimeoutMs = ms;
      sendOk("WATCHDOG_UPDATED");
      return;
    }

    if (equalsIgnoreCase(tokens[1], "TELEMETRY")) {
      if (tokenCount < 3) {
        sendErr("SET TELEMETRY <ms|0>");
        return;
      }

      uint32_t ms = 0;
      if (!parseUInt32(tokens[2], ms)) {
        sendErr("INVALID_TELEMETRY");
        return;
      }

      Runtime::telemetryPeriodMs = ms;
      sendOk("TELEMETRY_UPDATED");
      return;
    }

    if (equalsIgnoreCase(tokens[1], "CH")) {
      if (tokenCount < 4) {
        sendErr("SET CH <n> ...");
        return;
      }

      if (evaluateTopLevelState() != Runtime::TopLevelState::Ready) {
        sendErr("NOT_READY");
        return;
      }

      const uint8_t channel = toChannelIndex(tokens[2]);
      if (channel == 255) {
        sendErr("INVALID_CHANNEL");
        return;
      }

      if (equalsIgnoreCase(tokens[3], "OFF")) {
        setChannelOff(channel);
        sendOk("CH_OFF");
        return;
      }

      if (equalsIgnoreCase(tokens[3], "DC")) {
        setChannelDC(channel);
        sendOk("CH_DC");
        return;
      }

      if (equalsIgnoreCase(tokens[3], "PULSE")) {
        if (tokenCount < 5) {
          sendErr("SET CH <n> PULSE <duration_ms>");
          return;
        }

        if (tokenCount > 6) {
          sendErr("SET CH <n> PULSE <duration_ms> [count]");
          return;
        }

        uint32_t durationMs = 0;

        if (!parseUInt32(tokens[4], durationMs)) {
          sendErr("INVALID_DURATION");
          return;
        }

        if (durationMs < Config::PULSE_DURATION_MIN_MS || durationMs > Config::PULSE_DURATION_MAX_MS) {
          sendErr("DURATION_RANGE");
          return;
        }

        uint32_t count = 1;
        if (tokenCount == 6) {
          if (!parseUInt32(tokens[5], count)) {
            sendErr("INVALID_COUNT");
            return;
          }

          if (count < Config::PULSE_COUNT_MIN || count > Config::PULSE_COUNT_MAX) {
            sendErr("COUNT_RANGE");
            return;
          }
        }

        if (count == 1) {
          setChannelPulse(channel, durationMs);
          sendOk("CH_PULSE");
        } else {
          setChannelPulseTrain(channel, durationMs, count);
          sendOk("CH_PULSE_TRAIN");
        }
        return;
      }

      sendErr("UNKNOWN_CH_MODE");
      return;
    }

    sendErr("UNKNOWN_SET_TARGET");
    return;
  }

  sendErr("UNKNOWN_COMMAND");
}

static void pollRS485() {
  while (Config::RS485_SERIAL.available() > 0) {
    const char c = static_cast<char>(Config::RS485_SERIAL.read());

    // Ignore carriage return, parse on newline.
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      Runtime::rxBuffer[Runtime::rxIndex] = '\0';
      if (Runtime::rxIndex > 0) {
        processCommand(Runtime::rxBuffer);
      }
      Runtime::rxIndex = 0;
      Runtime::rxBuffer[0] = '\0';
      continue;
    }

    if (Runtime::rxIndex < (Runtime::RX_BUFFER_SIZE - 1)) {
      Runtime::rxBuffer[Runtime::rxIndex++] = c;
    } else {
      // Overflow protection: reset parser state and notify host.
      Runtime::rxIndex = 0;
      Runtime::rxBuffer[0] = '\0';
      sendErr("RX_OVERFLOW");
    }
  }
}

// ---------------------------
// Arduino lifecycle
// ---------------------------

void setup() {
  // Initialize RS485 transceiver direction control.
  pinMode(Config::RS485_DE_RE_PIN, OUTPUT);
  rs485SetTransmit(false);  // receive mode by default

  Config::RS485_SERIAL.begin(Config::RS485_BAUD);

  // Power LED indicates controller is powered.
  pinMode(Config::POWER_LED_PIN, OUTPUT);
  digitalWrite(Config::POWER_LED_PIN, HIGH);

  // Interlock input.
  pinMode(
    Config::KB_INTERLOCK_PIN,
    Config::INTERLOCK_USE_PULLUP ? INPUT_PULLUP : INPUT
  );

  // Pulser gate outputs: start LOW (safe).
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    pinMode(Config::PULSER_GATE_OUTPUT_PINS[channel], OUTPUT);
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);

    Runtime::channels[channel].mode = Runtime::OutputMode::Off;
    Runtime::channels[channel].pulseDurationMs = 100;
    Runtime::channels[channel].pulseCount = 1;
    Runtime::channels[channel].pulsesRemaining = 0;
    Runtime::channels[channel].pulseStartMs = millis();
    Runtime::channels[channel].pulseTrainInLowGap = false;
    Runtime::channels[channel].lastLevel = false;
  }

  // Pulser status lines (active-low open-drain).
  const uint8_t statusPinMode = Config::STATUS_USE_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT;
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    pinMode(Config::ENABLE_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::POWER_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::OVERCURRENT_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::GATED_STATUS_PINS[channel], statusPinMode);
  }

  Runtime::lastCommandMillis = millis();
  Runtime::lastTelemetryMillis = millis();

  rs485SendLine("BCON READY");
  rs485SendLine("Use HELP for command list");
}

void loop() {
  pollRS485();

  if (isAnyOverCurrentAsserted()) {
    Runtime::faultLatched = true;
  }

  applyOutputs();

  /*
    Periodic telemetry: send status report at configured interval if enabled.
     This includes current mode and status of each channel, as well as system-level info.
  */
  if (Runtime::telemetryPeriodMs > 0) {
    const uint32_t now = millis();
    if ((now - Runtime::lastTelemetryMillis) >= Runtime::telemetryPeriodMs) {
      Runtime::lastTelemetryMillis = now;
      sendStatusReport();
    }
  }
}
