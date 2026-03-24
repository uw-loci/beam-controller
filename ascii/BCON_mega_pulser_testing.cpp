#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

/*
  BCON_Mega_pulser_testing.cpp
  ------------------------------------------------------------
  Pulser test firmware for Arduino Mega.

  Purpose:
  - Mirror pulser output behavior from BCON_Mega_Logic (OFF / DC / timed PULSE)
  - Read and record pulser status input signals
  - Use USB serial CLI (Serial) instead of RS485
  - No watchdog and no interlock gating

  Upload target: Arduino Mega 2560
  CLI: open Serial Monitor at configured baud, newline-terminated commands
  ------------------------------------------------------------
*/

namespace Config {

constexpr uint8_t CHANNEL_COUNT = 3;

// ---------------------------
// CLI serial configuration
// ---------------------------
HardwareSerial &CLI_SERIAL = Serial;
constexpr uint32_t CLI_BAUD = 115200;

// ---------------------------
// Optional periodic telemetry
// ---------------------------
constexpr uint32_t DEFAULT_TELEMETRY_MS = 1000;  // 0 disables periodic status output

// External power LED output (fed when Arduino powered)
constexpr uint8_t POWER_LED_PIN = 13;

// ---------------------------
// Pulser gate outputs (to MOSFET gate-drive interface)
// ---------------------------
constexpr uint8_t PULSER_GATE_OUTPUT_PINS[CHANNEL_COUNT] = {5, 6, 7};

// ---------------------------
// Pulser status inputs (active-low open-drain from PVX-4140 remote interface)
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

struct ChannelControl {
  OutputMode mode = OutputMode::Off;
  uint32_t pulseDurationMs = 100;
  uint32_t pulseCount = 1;
  uint32_t pulsesRemaining = 0;
  uint32_t pulseStartMs = 0;
  bool pulseTrainInLowGap = false;
  bool lastLevel = false;
};

struct ChannelStatusRecord {
  bool en = false;
  bool pwr = false;
  bool oc = false;
  bool gated = false;

  uint32_t enTransitions = 0;
  uint32_t pwrTransitions = 0;
  uint32_t ocTransitions = 0;
  uint32_t gatedTransitions = 0;

  bool initialized = false;
};

ChannelControl channels[Config::CHANNEL_COUNT];
ChannelStatusRecord statusRecords[Config::CHANNEL_COUNT];

uint32_t telemetryPeriodMs = Config::DEFAULT_TELEMETRY_MS;
uint32_t lastTelemetryMillis = 0;

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

static const char *modeToString(Runtime::OutputMode mode) {
  switch (mode) {
    case Runtime::OutputMode::Off: return "OFF";
    case Runtime::OutputMode::DC: return "DC";
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

static void sendLine(const char *line) {
  Config::CLI_SERIAL.println(line);
}

static void sendOk(const char *detail = "OK") {
  char out[96];
  snprintf(out, sizeof(out), "OK %s", detail);
  sendLine(out);
}

static void sendErr(const char *detail) {
  char out[96];
  snprintf(out, sizeof(out), "ERR %s", detail);
  sendLine(out);
}

static void forceAllOutputsLow() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
    Runtime::channels[channel].lastLevel = false;
  }
}

static void applyOutputs() {
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

static void sampleStatusSignals() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    Runtime::ChannelStatusRecord &record = Runtime::statusRecords[channel];

    const bool en = activeLowRead(Config::ENABLE_STATUS_PINS[channel]);
    const bool pwr = activeLowRead(Config::POWER_STATUS_PINS[channel]);
    const bool oc = activeLowRead(Config::OVERCURRENT_STATUS_PINS[channel]);
    const bool gated = activeLowRead(Config::GATED_STATUS_PINS[channel]);

    if (!record.initialized) {
      record.en = en;
      record.pwr = pwr;
      record.oc = oc;
      record.gated = gated;
      record.initialized = true;
      continue;
    }

    if (record.en != en) {
      record.en = en;
      ++record.enTransitions;
    }
    if (record.pwr != pwr) {
      record.pwr = pwr;
      ++record.pwrTransitions;
    }
    if (record.oc != oc) {
      record.oc = oc;
      ++record.ocTransitions;
    }
    if (record.gated != gated) {
      record.gated = gated;
      ++record.gatedTransitions;
    }
  }
}

static void resetStatusTransitionCounters() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    Runtime::ChannelStatusRecord &record = Runtime::statusRecords[channel];
    record.enTransitions = 0;
    record.pwrTransitions = 0;
    record.ocTransitions = 0;
    record.gatedTransitions = 0;
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
  char out[192];

  snprintf(
    out,
    sizeof(out),
    "SYS telemetry_ms=%lu uptime_ms=%lu",
    static_cast<unsigned long>(Runtime::telemetryPeriodMs),
    static_cast<unsigned long>(millis())
  );
  sendLine(out);

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    const uint8_t c = channel + 1;
    const Runtime::ChannelStatusRecord &st = Runtime::statusRecords[channel];

    snprintf(
      out,
      sizeof(out),
      "CH%u mode=%s pulse_ms=%lu count=%lu remaining=%lu en_st=%u pwr_st=%u oc_st=%u gated_st=%u en_t=%lu pwr_t=%lu oc_t=%lu gated_t=%lu",
      c,
      modeToString(Runtime::channels[channel].mode),
      static_cast<unsigned long>(Runtime::channels[channel].pulseDurationMs),
      static_cast<unsigned long>(Runtime::channels[channel].pulseCount),
      static_cast<unsigned long>(Runtime::channels[channel].pulsesRemaining),
      st.en ? 1 : 0,
      st.pwr ? 1 : 0,
      st.oc ? 1 : 0,
      st.gated ? 1 : 0,
      static_cast<unsigned long>(st.enTransitions),
      static_cast<unsigned long>(st.pwrTransitions),
      static_cast<unsigned long>(st.ocTransitions),
      static_cast<unsigned long>(st.gatedTransitions)
    );
    sendLine(out);
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

  if (equalsIgnoreCase(tokens[0], "PING")) {
    sendLine("PONG");
    return;
  }

  if (equalsIgnoreCase(tokens[0], "HELP")) {
    sendLine("CMD PING");
    sendLine("CMD STATUS");
    sendLine("CMD STOP ALL");
    sendLine("CMD SET CH <1..3> OFF");
    sendLine("CMD SET CH <1..3> DC");
    sendLine("CMD SET CH <1..3> PULSE <duration_ms> [count]");
    sendLine("CMD SET TELEMETRY <ms|0=off>");
    sendLine("CMD RESET COUNTS");
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

  if (tokenCount >= 2 && equalsIgnoreCase(tokens[0], "RESET") && equalsIgnoreCase(tokens[1], "COUNTS")) {
    resetStatusTransitionCounters();
    sendOk("COUNTS_RESET");
    return;
  }

  if (tokenCount >= 2 && equalsIgnoreCase(tokens[0], "SET")) {
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
        if (tokenCount >= 6) {
          if (!parseUInt32(tokens[5], count)) {
            sendErr("INVALID_COUNT");
            return;
          }
          if (count < Config::PULSE_COUNT_MIN || count > Config::PULSE_COUNT_MAX) {
            sendErr("COUNT_RANGE");
            return;
          }
        }

        if (count <= 1) {
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

static void pollCli() {
  while (Config::CLI_SERIAL.available() > 0) {
    const char c = static_cast<char>(Config::CLI_SERIAL.read());

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
  Config::CLI_SERIAL.begin(Config::CLI_BAUD);

  // Power LED indicates controller is powered.
  pinMode(Config::POWER_LED_PIN, OUTPUT);
  digitalWrite(Config::POWER_LED_PIN, HIGH);

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

  Runtime::lastTelemetryMillis = millis();

  // Prime recorded status values.
  sampleStatusSignals();

  sendLine("BCON PULSER TEST READY");
  sendLine("CLI over USB Serial. Use HELP for command list");
}

void loop() {
  pollCli();
  sampleStatusSignals();
  applyOutputs();

  if (Runtime::telemetryPeriodMs > 0) {
    const uint32_t now = millis();
    if ((now - Runtime::lastTelemetryMillis) >= Runtime::telemetryPeriodMs) {
      Runtime::lastTelemetryMillis = now;
      sendStatusReport();
    }
  }
}
