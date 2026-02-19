#include <Arduino.h>
#include <string.h>

#ifndef BCON_MODBUS_USE_USB_SERIAL
#define BCON_MODBUS_USE_USB_SERIAL 1
#endif

#if BCON_MODBUS_USE_USB_SERIAL
#define BCON_MODBUS_PORT Serial
#else
#define BCON_MODBUS_PORT Serial1
#endif

#define BCON_ENABLE_I2C_LCD 1

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

namespace Config {

constexpr uint8_t CHANNEL_COUNT = 3;

HardwareSerial &RS485_SERIAL = BCON_MODBUS_PORT;
constexpr uint32_t RS485_BAUD = 115200;
constexpr uint8_t RS485_DE_RE_PIN = 2;
constexpr uint8_t MODBUS_SLAVE_ID = 1;
constexpr bool USE_USB_SERIAL = (BCON_MODBUS_USE_USB_SERIAL != 0);

constexpr uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 1000;
constexpr uint32_t DEFAULT_TELEMETRY_MS = 1000;
constexpr uint32_t HEARTBEAT_MIN_MS = 50;
constexpr uint32_t HEARTBEAT_MAX_MS = 60000;

constexpr uint8_t KB_INTERLOCK_PIN = 22;
constexpr bool INTERLOCK_ACTIVE_HIGH = true;
constexpr bool INTERLOCK_USE_PULLUP = false;

constexpr uint8_t PULSER_GATE_OUTPUT_PINS[CHANNEL_COUNT] = {5, 6, 7};
constexpr uint8_t PULSER_ENABLE_TOGGLE_OUTPUT_PINS[CHANNEL_COUNT] = {8, 9, 10};
constexpr bool ENABLE_TOGGLE_ACTIVE_HIGH = true;
constexpr uint16_t ENABLE_TOGGLE_PULSE_MS = 100;

constexpr bool STATUS_USE_INTERNAL_PULLUPS = true;

constexpr uint8_t ENABLE_STATUS_PINS[CHANNEL_COUNT] = {23, 27, 31};
constexpr uint8_t POWER_STATUS_PINS[CHANNEL_COUNT] = {24, 28, 32};
constexpr uint8_t OVERCURRENT_STATUS_PINS[CHANNEL_COUNT] = {25, 29, 33};
constexpr uint8_t GATED_STATUS_PINS[CHANNEL_COUNT] = {26, 30, 34};

constexpr uint32_t PULSE_DURATION_MIN_MS = 1;
constexpr uint32_t PULSE_DURATION_MAX_MS = 60000;
constexpr uint32_t PULSE_COUNT_MIN = 1;
constexpr uint32_t PULSE_COUNT_MAX = 10000;

constexpr size_t MODBUS_RX_BUFFER_SIZE = 256;
constexpr uint32_t MODBUS_FRAME_GAP_MS = 4;

constexpr bool ENABLE_DEBUG_LCD = (BCON_ENABLE_I2C_LCD != 0);
constexpr uint8_t LCD_I2C_ADDRESS = 0x27;
constexpr uint8_t LCD_COLS = 20;
constexpr uint8_t LCD_ROWS = 4;
constexpr uint32_t LCD_REFRESH_MS = 250;

constexpr const char *LCD_LABEL_WATCHDOG = "WDG";
constexpr const char *LCD_LABEL_INTERLOCK = "INT";
constexpr const char *LCD_LABEL_FAULT = "FLT";
constexpr const char *LCD_LABEL_CHANNELS[CHANNEL_COUNT] = {"CH1", "CH2", "CH3"};

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

enum class ModbusError : uint16_t {
  None = 0,
  IllegalFunction = 1,
  IllegalAddress = 2,
  IllegalValue = 3,
  DeviceFailure = 4,
  NotReady = 10,
  FaultStillActive = 11,
  InterlockNotReady = 12,
  BufferOverflow = 13
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

volatile uint32_t chCountdownMs[Config::CHANNEL_COUNT] = {0};
volatile bool chEventFlag[Config::CHANNEL_COUNT] = {false};
volatile uint16_t enableToggleCountdownMs[Config::CHANNEL_COUNT] = {0};
volatile bool enableToggleEventFlag[Config::CHANNEL_COUNT] = {false};
volatile uint32_t lcdCountdownMs = 0;
volatile bool lcdEventFlag = false;

uint32_t lastCommandMillis = 0;
uint32_t watchdogTimeoutMs = Config::DEFAULT_WATCHDOG_TIMEOUT_MS;
uint32_t telemetryPeriodMs = Config::DEFAULT_TELEMETRY_MS;
bool faultLatched = false;
ModbusError lastModbusError = ModbusError::None;

uint8_t modbusRxBuffer[Config::MODBUS_RX_BUFFER_SIZE] = {0};
size_t modbusRxIndex = 0;
uint32_t lastModbusByteMillis = 0;

}  // namespace Runtime

namespace ModbusMap {

constexpr uint16_t REG_WATCHDOG_MS = 0;
constexpr uint16_t REG_TELEMETRY_MS = 1;
constexpr uint16_t REG_COMMAND = 2;

constexpr uint16_t REG_CH1_MODE = 10;
constexpr uint16_t REG_CH1_PULSE_MS = 11;
constexpr uint16_t REG_CH1_COUNT = 12;
constexpr uint16_t REG_CH1_ENABLE_TOGGLE = 13;

constexpr uint16_t REG_CH2_MODE = 20;
constexpr uint16_t REG_CH2_PULSE_MS = 21;
constexpr uint16_t REG_CH2_COUNT = 22;
constexpr uint16_t REG_CH2_ENABLE_TOGGLE = 23;

constexpr uint16_t REG_CH3_MODE = 30;
constexpr uint16_t REG_CH3_PULSE_MS = 31;
constexpr uint16_t REG_CH3_COUNT = 32;
constexpr uint16_t REG_CH3_ENABLE_TOGGLE = 33;

constexpr uint16_t REG_SYS_STATE = 100;
constexpr uint16_t REG_SYS_REASON = 101;
constexpr uint16_t REG_FAULT_LATCHED = 102;
constexpr uint16_t REG_INTERLOCK_OK = 103;
constexpr uint16_t REG_WATCHDOG_OK = 104;
constexpr uint16_t REG_LAST_ERROR = 105;

constexpr uint16_t REG_CH_STATUS_BASE = 110;
constexpr uint16_t REG_CH_STATUS_STRIDE = 10;

}  // namespace ModbusMap

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

static const char *modeToShortString(Runtime::OutputMode mode) {
  switch (mode) {
    case Runtime::OutputMode::Off: return "OFF";
    case Runtime::OutputMode::DC: return "DC ";
    case Runtime::OutputMode::Pulse: return "PUL";
    case Runtime::OutputMode::PulseTrain: return "TRN";
    default: return "UNK";
  }
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

static uint16_t topLevelStateToCode(Runtime::TopLevelState state) {
  return static_cast<uint16_t>(state);
}

static uint16_t topLevelReasonCode(Runtime::TopLevelState state) {
  switch (state) {
    case Runtime::TopLevelState::Ready: return 0;
    case Runtime::TopLevelState::SafeInterlock: return 1;
    case Runtime::TopLevelState::SafeWatchdog: return 2;
    case Runtime::TopLevelState::FaultLatched: return 3;
    default: return 0;
  }
}

static uint16_t modeToCode(Runtime::OutputMode mode) {
  switch (mode) {
    case Runtime::OutputMode::Off: return 0;
    case Runtime::OutputMode::DC: return 1;
    case Runtime::OutputMode::Pulse: return 2;
    case Runtime::OutputMode::PulseTrain: return 3;
    default: return 0;
  }
}

static void setModbusError(Runtime::ModbusError error) {
  Runtime::lastModbusError = error;
}

static void pulseEnableToggle(uint8_t channelIndex) {
  const uint8_t pin = Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[channelIndex];
  const uint8_t activeLevel = Config::ENABLE_TOGGLE_ACTIVE_HIGH ? HIGH : LOW;

  digitalWrite(pin, activeLevel);
  noInterrupts();
  Runtime::enableToggleCountdownMs[channelIndex] = Config::ENABLE_TOGGLE_PULSE_MS;
  Runtime::enableToggleEventFlag[channelIndex] = false;
  interrupts();
}

static void rs485SetTransmit(bool enabled) {
  if (Config::USE_USB_SERIAL) {
    (void)enabled;
    return;
  }
  digitalWrite(Config::RS485_DE_RE_PIN, enabled ? HIGH : LOW);
}

static uint16_t modbusCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void modbusSendFrame(const uint8_t *payload, size_t payloadLen) {
  uint8_t frame[260];
  if (payloadLen + 2 > sizeof(frame)) {
    return;
  }

  for (size_t i = 0; i < payloadLen; ++i) {
    frame[i] = payload[i];
  }

  const uint16_t crc = modbusCrc16(payload, payloadLen);
  frame[payloadLen] = static_cast<uint8_t>(crc & 0xFF);
  frame[payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  rs485SetTransmit(true);
  Config::RS485_SERIAL.write(frame, payloadLen + 2);
  Config::RS485_SERIAL.flush();
  rs485SetTransmit(false);
}

static void modbusSendException(uint8_t slaveId, uint8_t functionCode, uint8_t exceptionCode) {
  const uint8_t payload[3] = {
    slaveId,
    static_cast<uint8_t>(functionCode | 0x80),
    exceptionCode
  };
  modbusSendFrame(payload, sizeof(payload));
}

static void forceAllOutputsLow() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
    Runtime::channels[channel].lastLevel = false;
  }
}

static void armChannelTimer(uint8_t channel, uint32_t delayMs) {
  noInterrupts();
  Runtime::chCountdownMs[channel] = delayMs;
  Runtime::chEventFlag[channel] = false;
  interrupts();
}

static void clearChannelTimer(uint8_t channel) {
  noInterrupts();
  Runtime::chCountdownMs[channel] = 0;
  Runtime::chEventFlag[channel] = false;
  interrupts();
}

static void channelTimerTick(uint8_t channel) {
  if (Runtime::chCountdownMs[channel] > 0) {
    Runtime::chCountdownMs[channel]--;
    if (Runtime::chCountdownMs[channel] == 0) {
      Runtime::chEventFlag[channel] = true;
    }
  }
}

static void lcdTimerTick() {
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    if (Runtime::enableToggleCountdownMs[channel] > 0) {
      Runtime::enableToggleCountdownMs[channel]--;
      if (Runtime::enableToggleCountdownMs[channel] == 0) {
        Runtime::enableToggleEventFlag[channel] = true;
      }
    }
  }

  if (Runtime::lcdCountdownMs > 0) {
    Runtime::lcdCountdownMs--;
    if (Runtime::lcdCountdownMs == 0) {
      Runtime::lcdEventFlag = true;
    }
  }
}

static void serviceEnableToggleOutputs() {
  const uint8_t inactiveLevel = Config::ENABLE_TOGGLE_ACTIVE_HIGH ? LOW : HIGH;

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    bool event = false;
    noInterrupts();
    if (Runtime::enableToggleEventFlag[channel]) {
      Runtime::enableToggleEventFlag[channel] = false;
      event = true;
    }
    interrupts();

    if (event) {
      digitalWrite(Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[channel], inactiveLevel);
    }
  }
}

ISR(TIMER1_COMPA_vect) { channelTimerTick(0); }
ISR(TIMER3_COMPA_vect) { channelTimerTick(1); }
ISR(TIMER4_COMPA_vect) { channelTimerTick(2); }
ISR(TIMER5_COMPA_vect) { lcdTimerTick(); }

static void setupChannelTimers() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 249;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31) | (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 249;
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS41) | (1 << CS40);
  TIMSK4 |= (1 << OCIE4A);

  interrupts();
}

static void armLcdTimer(uint32_t delayMs) {
  noInterrupts();
  Runtime::lcdCountdownMs = delayMs;
  Runtime::lcdEventFlag = false;
  interrupts();
}

static bool consumeLcdTimerEvent() {
  bool event = false;
  noInterrupts();
  if (Runtime::lcdEventFlag) {
    Runtime::lcdEventFlag = false;
    event = true;
  }
  interrupts();
  return event;
}

static void setupLcdTimer() {
  noInterrupts();

  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  OCR5A = 249;
  TCCR5B |= (1 << WGM52);
  TCCR5B |= (1 << CS51) | (1 << CS50);
  TIMSK5 |= (1 << OCIE5A);

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    Runtime::enableToggleCountdownMs[channel] = 0;
    Runtime::enableToggleEventFlag[channel] = false;
  }

  Runtime::lcdCountdownMs = 0;
  Runtime::lcdEventFlag = false;

  interrupts();
}

static void setChannelOff(uint8_t indexZeroBased) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::Off;
  cfg.pulsesRemaining = 0;
  cfg.pulseTrainInLowGap = false;
  cfg.lastLevel = false;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[indexZeroBased], LOW);
  clearChannelTimer(indexZeroBased);
}

static void setChannelDC(uint8_t indexZeroBased) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::DC;
  cfg.pulsesRemaining = 0;
  cfg.pulseTrainInLowGap = false;
  cfg.lastLevel = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[indexZeroBased], HIGH);
  clearChannelTimer(indexZeroBased);
}

static void setChannelPulse(uint8_t indexZeroBased, uint32_t durationMs) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::Pulse;
  cfg.pulseDurationMs = durationMs;
  cfg.pulseCount = 1;
  cfg.pulsesRemaining = 1;
  cfg.pulseStartMs = millis();
  cfg.pulseTrainInLowGap = false;
  cfg.lastLevel = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[indexZeroBased], HIGH);
  armChannelTimer(indexZeroBased, cfg.pulseDurationMs);
}

static void setChannelPulseTrain(uint8_t indexZeroBased, uint32_t durationMs, uint32_t count) {
  Runtime::ChannelControl &cfg = Runtime::channels[indexZeroBased];
  cfg.mode = Runtime::OutputMode::PulseTrain;
  cfg.pulseDurationMs = durationMs;
  cfg.pulseCount = count;
  cfg.pulsesRemaining = count;
  cfg.pulseStartMs = millis();
  cfg.pulseTrainInLowGap = false;
  cfg.lastLevel = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[indexZeroBased], HIGH);
  armChannelTimer(indexZeroBased, cfg.pulseDurationMs);
}

static void applyOutputs() {
  const Runtime::TopLevelState state = evaluateTopLevelState();

  if (state != Runtime::TopLevelState::Ready) {
    forceAllOutputsLow();
    return;
  }

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    Runtime::ChannelControl &cfg = Runtime::channels[channel];
    switch (cfg.mode) {
      case Runtime::OutputMode::Off:
        if (cfg.lastLevel) {
          cfg.lastLevel = false;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
        }
        clearChannelTimer(channel);
        break;

      case Runtime::OutputMode::DC:
        if (!cfg.lastLevel) {
          cfg.lastLevel = true;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], HIGH);
        }
        clearChannelTimer(channel);
        break;

      case Runtime::OutputMode::Pulse: {
        bool event = false;
        noInterrupts();
        if (Runtime::chEventFlag[channel]) {
          Runtime::chEventFlag[channel] = false;
          event = true;
        }
        interrupts();

        if (!event) {
          break;
        }

        if (cfg.lastLevel) {
          cfg.lastLevel = false;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
          cfg.mode = Runtime::OutputMode::Off;
          cfg.pulsesRemaining = 0;
          clearChannelTimer(channel);
        }
        break;
      }

      case Runtime::OutputMode::PulseTrain: {
        bool event = false;
        noInterrupts();
        if (Runtime::chEventFlag[channel]) {
          Runtime::chEventFlag[channel] = false;
          event = true;
        }
        interrupts();

        if (!event) {
          break;
        }

        if (cfg.lastLevel) {
          cfg.lastLevel = false;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
          if (cfg.pulsesRemaining > 0) {
            cfg.pulsesRemaining--;
          }

          if (cfg.pulsesRemaining == 0) {
            clearChannelTimer(channel);
          cfg.mode = Runtime::OutputMode::Off;
          cfg.pulseTrainInLowGap = false;
          } else {
            cfg.pulseTrainInLowGap = true;
            cfg.pulseStartMs = millis();
            armChannelTimer(channel, cfg.pulseDurationMs);
          }
        } else {
          cfg.pulseTrainInLowGap = false;
          cfg.pulseStartMs = millis();
          cfg.lastLevel = true;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], HIGH);
          armChannelTimer(channel, cfg.pulseDurationMs);

          if (cfg.pulsesRemaining == 0) {
            cfg.mode = Runtime::OutputMode::Off;
            cfg.lastLevel = false;
            digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
            clearChannelTimer(channel);
          }
        }
        break;
      }

      default:
        cfg.lastLevel = false;
        digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);
        clearChannelTimer(channel);
        break;
    }
  }
}

static bool decodeChannelControlRegister(uint16_t reg, uint8_t &channelIndex, uint8_t &field) {
  if (reg < 10 || reg > 33) {
    return false;
  }

  const uint8_t decade = static_cast<uint8_t>(reg / 10);
  const uint8_t units = static_cast<uint8_t>(reg % 10);

  if (decade < 1 || decade > 3 || units > 3) {
    return false;
  }

  channelIndex = decade - 1;
  field = units;
  return true;
}

static bool decodeChannelStatusRegister(uint16_t reg, uint8_t &channelIndex, uint8_t &field) {
  if (reg < ModbusMap::REG_CH_STATUS_BASE) {
    return false;
  }

  const uint16_t delta = reg - ModbusMap::REG_CH_STATUS_BASE;
  const uint8_t idx = static_cast<uint8_t>(delta / ModbusMap::REG_CH_STATUS_STRIDE);
  const uint8_t off = static_cast<uint8_t>(delta % ModbusMap::REG_CH_STATUS_STRIDE);

  if (idx >= Config::CHANNEL_COUNT || off > 8) {
    return false;
  }

  channelIndex = idx;
  field = off;
  return true;
}

static bool readHoldingRegister(uint16_t reg, uint16_t &valueOut) {
  switch (reg) {
    case ModbusMap::REG_WATCHDOG_MS:
      valueOut = static_cast<uint16_t>(Runtime::watchdogTimeoutMs);
      return true;
    case ModbusMap::REG_TELEMETRY_MS:
      valueOut = static_cast<uint16_t>(Runtime::telemetryPeriodMs);
      return true;
    case ModbusMap::REG_COMMAND:
      valueOut = 0;
      return true;
    case ModbusMap::REG_SYS_STATE:
      valueOut = topLevelStateToCode(evaluateTopLevelState());
      return true;
    case ModbusMap::REG_SYS_REASON:
      valueOut = topLevelReasonCode(evaluateTopLevelState());
      return true;
    case ModbusMap::REG_FAULT_LATCHED:
      valueOut = Runtime::faultLatched ? 1 : 0;
      return true;
    case ModbusMap::REG_INTERLOCK_OK:
      valueOut = isInterlockSatisfied() ? 1 : 0;
      return true;
    case ModbusMap::REG_WATCHDOG_OK:
      valueOut = isWatchdogHealthy() ? 1 : 0;
      return true;
    case ModbusMap::REG_LAST_ERROR:
      valueOut = static_cast<uint16_t>(Runtime::lastModbusError);
      return true;
    default:
      break;
  }

  uint8_t channel = 0;
  uint8_t field = 0;

  if (decodeChannelControlRegister(reg, channel, field)) {
    const Runtime::ChannelControl &cfg = Runtime::channels[channel];
    if (field == 0) {
      valueOut = modeToCode(cfg.mode);
      return true;
    }
    if (field == 1) {
      valueOut = static_cast<uint16_t>(cfg.pulseDurationMs);
      return true;
    }
    if (field == 2) {
      valueOut = static_cast<uint16_t>(cfg.pulseCount);
      return true;
    }
    if (field == 3) {
      valueOut = 0;
      return true;
    }
  }

  if (decodeChannelStatusRegister(reg, channel, field)) {
    const Runtime::ChannelControl &cfg = Runtime::channels[channel];
    switch (field) {
      case 0: valueOut = modeToCode(cfg.mode); return true;
      case 1: valueOut = static_cast<uint16_t>(cfg.pulseDurationMs); return true;
      case 2: valueOut = static_cast<uint16_t>(cfg.pulseCount); return true;
      case 3: valueOut = static_cast<uint16_t>(cfg.pulsesRemaining); return true;
      case 4: valueOut = activeLowRead(Config::ENABLE_STATUS_PINS[channel]) ? 1 : 0; return true;
      case 5: valueOut = activeLowRead(Config::POWER_STATUS_PINS[channel]) ? 1 : 0; return true;
      case 6: valueOut = activeLowRead(Config::OVERCURRENT_STATUS_PINS[channel]) ? 1 : 0; return true;
      case 7: valueOut = activeLowRead(Config::GATED_STATUS_PINS[channel]) ? 1 : 0; return true;
      case 8: valueOut = cfg.lastLevel ? 1 : 0; return true;
      default: return false;
    }
  }

  return false;
}

static bool writeHoldingRegister(uint16_t reg, uint16_t value, uint8_t &exceptionCodeOut) {
  exceptionCodeOut = 0x03;

  if (reg == ModbusMap::REG_WATCHDOG_MS) {
    if (value < Config::HEARTBEAT_MIN_MS || value > Config::HEARTBEAT_MAX_MS) {
      setModbusError(Runtime::ModbusError::IllegalValue);
      return false;
    }
    Runtime::watchdogTimeoutMs = value;
    setModbusError(Runtime::ModbusError::None);
    return true;
  }

  if (reg == ModbusMap::REG_TELEMETRY_MS) {
    Runtime::telemetryPeriodMs = value;
    setModbusError(Runtime::ModbusError::None);
    return true;
  }

  if (reg == ModbusMap::REG_COMMAND) {
    if (value == 0) {
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    if (value == 1) {
      for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
        setChannelOff(channel);
      }
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    if (value == 2 || value == 3) {
      if (isAnyOverCurrentAsserted()) {
        setModbusError(Runtime::ModbusError::FaultStillActive);
        exceptionCodeOut = 0x04;
        return false;
      }
      if (!isInterlockSatisfied()) {
        setModbusError(Runtime::ModbusError::InterlockNotReady);
        exceptionCodeOut = 0x04;
        return false;
      }
      Runtime::faultLatched = false;
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    setModbusError(Runtime::ModbusError::IllegalValue);
    return false;
  }

  uint8_t channel = 0;
  uint8_t field = 0;
  if (!decodeChannelControlRegister(reg, channel, field)) {
    setModbusError(Runtime::ModbusError::IllegalAddress);
    exceptionCodeOut = 0x02;
    return false;
  }

  Runtime::ChannelControl &cfg = Runtime::channels[channel];

  if (field == 1) {
    if (value < Config::PULSE_DURATION_MIN_MS || value > Config::PULSE_DURATION_MAX_MS) {
      setModbusError(Runtime::ModbusError::IllegalValue);
      return false;
    }
    cfg.pulseDurationMs = value;
    setModbusError(Runtime::ModbusError::None);
    return true;
  }

  if (field == 2) {
    if (value < Config::PULSE_COUNT_MIN || value > Config::PULSE_COUNT_MAX) {
      setModbusError(Runtime::ModbusError::IllegalValue);
      return false;
    }
    cfg.pulseCount = value;
    setModbusError(Runtime::ModbusError::None);
    return true;
  }

  if (field == 3) {
    if (value != 1) {
      setModbusError(Runtime::ModbusError::IllegalValue);
      return false;
    }

    pulseEnableToggle(channel);
    setModbusError(Runtime::ModbusError::None);
    return true;
  }

  if (field == 0) {
    const Runtime::TopLevelState state = evaluateTopLevelState();

    if (value == 0) {
      setChannelOff(channel);
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    if (state != Runtime::TopLevelState::Ready) {
      setModbusError(Runtime::ModbusError::NotReady);
      exceptionCodeOut = 0x04;
      return false;
    }

    if (value == 1) {
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[channel])) {
        pulseEnableToggle(channel);
      }
      setChannelDC(channel);
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    if (value == 2) {
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[channel])) {
        pulseEnableToggle(channel);
      }
      if (cfg.pulseCount <= 1) {
        setChannelPulse(channel, cfg.pulseDurationMs);
      } else {
        setChannelPulseTrain(channel, cfg.pulseDurationMs, cfg.pulseCount);
      }
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    if (value == 3) {
      if (cfg.pulseCount < 2) {
        setModbusError(Runtime::ModbusError::IllegalValue);
        return false;
      }
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[channel])) {
        pulseEnableToggle(channel);
      }
      setChannelPulseTrain(channel, cfg.pulseDurationMs, cfg.pulseCount);
      setModbusError(Runtime::ModbusError::None);
      return true;
    }

    setModbusError(Runtime::ModbusError::IllegalValue);
    return false;
  }

  setModbusError(Runtime::ModbusError::IllegalAddress);
  exceptionCodeOut = 0x02;
  return false;
}

static void processModbusFrame(const uint8_t *frame, size_t len) {
  if (len < 4) {
    return;
  }

  const uint8_t slaveId = frame[0];
  const bool isBroadcast = (slaveId == 0);
  if (!isBroadcast && slaveId != Config::MODBUS_SLAVE_ID) {
    return;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(frame[len - 2]) |
                               static_cast<uint16_t>(frame[len - 1] << 8);
  const uint16_t calculatedCrc = modbusCrc16(frame, len - 2);
  if (receivedCrc != calculatedCrc) {
    return;
  }

  Runtime::lastCommandMillis = millis();

  const uint8_t functionCode = frame[1];

  if (functionCode == 0x03) {
    if (len != 8) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    const uint16_t startReg = static_cast<uint16_t>(frame[2] << 8) | frame[3];
    const uint16_t quantity = static_cast<uint16_t>(frame[4] << 8) | frame[5];

    if (quantity == 0 || quantity > 125) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    uint8_t payload[260];
    payload[0] = slaveId;
    payload[1] = functionCode;
    payload[2] = static_cast<uint8_t>(quantity * 2);

    for (uint16_t i = 0; i < quantity; ++i) {
      uint16_t value = 0;
      if (!readHoldingRegister(static_cast<uint16_t>(startReg + i), value)) {
        if (!isBroadcast) {
          modbusSendException(slaveId, functionCode, 0x02);
        }
        setModbusError(Runtime::ModbusError::IllegalAddress);
        return;
      }
      payload[3 + (i * 2)] = static_cast<uint8_t>((value >> 8) & 0xFF);
      payload[4 + (i * 2)] = static_cast<uint8_t>(value & 0xFF);
    }

    if (!isBroadcast) {
      modbusSendFrame(payload, static_cast<size_t>(3 + quantity * 2));
    }

    setModbusError(Runtime::ModbusError::None);
    return;
  }

  if (functionCode == 0x06) {
    if (len != 8) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    const uint16_t reg = static_cast<uint16_t>(frame[2] << 8) | frame[3];
    const uint16_t value = static_cast<uint16_t>(frame[4] << 8) | frame[5];

    uint8_t exceptionCode = 0x03;
    if (!writeHoldingRegister(reg, value, exceptionCode)) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, exceptionCode);
      }
      return;
    }

    if (!isBroadcast) {
      modbusSendFrame(frame, 6);
    }
    return;
  }

  if (functionCode == 0x10) {
    if (len < 11) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    const uint16_t startReg = static_cast<uint16_t>(frame[2] << 8) | frame[3];
    const uint16_t quantity = static_cast<uint16_t>(frame[4] << 8) | frame[5];
    const uint8_t byteCount = frame[6];

    if (quantity == 0 || quantity > 123 || byteCount != quantity * 2) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    if (len != static_cast<size_t>(9 + byteCount)) {
      if (!isBroadcast) {
        modbusSendException(slaveId, functionCode, 0x03);
      }
      setModbusError(Runtime::ModbusError::IllegalValue);
      return;
    }

    for (uint16_t i = 0; i < quantity; ++i) {
      const size_t offset = static_cast<size_t>(7 + i * 2);
      const uint16_t value = static_cast<uint16_t>(frame[offset] << 8) | frame[offset + 1];
      uint8_t exceptionCode = 0x03;
      if (!writeHoldingRegister(static_cast<uint16_t>(startReg + i), value, exceptionCode)) {
        if (!isBroadcast) {
          modbusSendException(slaveId, functionCode, exceptionCode);
        }
        return;
      }
    }

    if (!isBroadcast) {
      uint8_t payload[6];
      payload[0] = slaveId;
      payload[1] = functionCode;
      payload[2] = static_cast<uint8_t>((startReg >> 8) & 0xFF);
      payload[3] = static_cast<uint8_t>(startReg & 0xFF);
      payload[4] = static_cast<uint8_t>((quantity >> 8) & 0xFF);
      payload[5] = static_cast<uint8_t>(quantity & 0xFF);
      modbusSendFrame(payload, sizeof(payload));
    }
    return;
  }

  if (!isBroadcast) {
    modbusSendException(slaveId, functionCode, 0x01);
  }
  setModbusError(Runtime::ModbusError::IllegalFunction);
}

static void pollModbus() {
  while (Config::RS485_SERIAL.available() > 0) {
    const int raw = Config::RS485_SERIAL.read();
    if (raw < 0) {
      break;
    }

    if (Runtime::modbusRxIndex < Config::MODBUS_RX_BUFFER_SIZE) {
      Runtime::modbusRxBuffer[Runtime::modbusRxIndex++] = static_cast<uint8_t>(raw);
      Runtime::lastModbusByteMillis = millis();
    } else {
      Runtime::modbusRxIndex = 0;
      setModbusError(Runtime::ModbusError::BufferOverflow);
    }
  }

  if (Runtime::modbusRxIndex > 0) {
    const uint32_t now = millis();
    if ((now - Runtime::lastModbusByteMillis) >= Config::MODBUS_FRAME_GAP_MS) {
      processModbusFrame(Runtime::modbusRxBuffer, Runtime::modbusRxIndex);
      Runtime::modbusRxIndex = 0;
    }
  }
}

namespace LcdDisplay {

#if BCON_ENABLE_I2C_LCD
LiquidCrystal_I2C lcdPrimary(Config::LCD_I2C_ADDRESS, Config::LCD_COLS, Config::LCD_ROWS);
LiquidCrystal_I2C *lcd = &lcdPrimary;
bool lcdInitialized = false;
bool lcdFirstUpdateLogged = false;
char lineCache[Config::LCD_ROWS][Config::LCD_COLS + 1] = {{0}};

static void lcdInitDebugPrint(const char *msg) {
  (void)msg;
}

static void lcdInitDebugPrintAddress(const char *prefix, uint8_t address) {
  (void)prefix;
  (void)address;
}

static void lcdInitDebugPrintRow(uint8_t row, const char *text) {
  (void)row;
  (void)text;
}

static bool i2cAddressPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static void writeRow(uint8_t row, const char *text) {
  if (!lcdInitialized || row >= Config::LCD_ROWS || text == nullptr) {
    return;
  }

  char padded[Config::LCD_COLS + 1] = {0};
  for (uint8_t i = 0; i < Config::LCD_COLS; ++i) {
    padded[i] = ' ';
  }
  padded[Config::LCD_COLS] = '\0';

  const size_t textLen = strlen(text);
  const size_t copyLen = (textLen < Config::LCD_COLS) ? textLen : Config::LCD_COLS;
  for (size_t i = 0; i < copyLen; ++i) {
    padded[i] = text[i];
  }

  if (strncmp(lineCache[row], padded, Config::LCD_COLS) == 0) {
    return;
  }

  strncpy(lineCache[row], padded, Config::LCD_COLS);
  lineCache[row][Config::LCD_COLS] = '\0';
  lcd->setCursor(0, row);
  lcd->print(padded);
  lcdInitDebugPrintRow(row, padded);
}
#endif

void begin() {
#if BCON_ENABLE_I2C_LCD
  lcdInitDebugPrint("[LCD] begin init");
  Wire.begin();
  lcdInitDebugPrint("[LCD] Wire.begin done");
  lcdInitDebugPrintAddress("[LCD] probing address ", Config::LCD_I2C_ADDRESS);

  if (i2cAddressPresent(Config::LCD_I2C_ADDRESS)) {
    lcd = &lcdPrimary;
    lcdInitDebugPrint("[LCD] address responded");
  } else {
    lcdInitialized = false;
    lcdInitDebugPrint("[LCD] address not found");
    return;
  }

  lcdInitDebugPrint("[LCD] calling lcd->init()");
  lcd->init();
  lcdInitDebugPrint("[LCD] enabling backlight");
  lcd->backlight();
  lcdInitDebugPrint("[LCD] clearing display");
  lcd->clear();
  lcdInitialized = true;
  lcdFirstUpdateLogged = false;
  lcdInitDebugPrint("[LCD] init complete");

  writeRow(0, "BCON Pulser Debug");
  writeRow(1, "Booting...");
  writeRow(2, "");
  writeRow(3, "");
  armLcdTimer(Config::LCD_REFRESH_MS);
#endif
}

void update() {
#if BCON_ENABLE_I2C_LCD
  if (!Config::ENABLE_DEBUG_LCD) {
    return;
  }

  if (!consumeLcdTimerEvent()) {
    return;
  }

  if (!lcdFirstUpdateLogged) {
    lcdInitDebugPrint("[LCD] first update tick");
    lcdFirstUpdateLogged = true;
  }

  armLcdTimer(Config::LCD_REFRESH_MS);

  char row0[32];
  snprintf(
    row0,
    sizeof(row0),
    "%s:%s %s:%s %s:%u",
    Config::LCD_LABEL_WATCHDOG,
    isWatchdogHealthy() ? "OK" : "BAD",
    Config::LCD_LABEL_INTERLOCK,
    isInterlockSatisfied() ? "OK" : "BAD",
    Config::LCD_LABEL_FAULT,
    Runtime::faultLatched ? 1 : 0
  );
  writeRow(0, row0);

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT && (channel + 1) < Config::LCD_ROWS; ++channel) {
    char row[32];
    const Runtime::ChannelControl &cfg = Runtime::channels[channel];
    snprintf(
      row,
      sizeof(row),
      "%s %s O:%u R:%lu",
      Config::LCD_LABEL_CHANNELS[channel],
      modeToShortString(cfg.mode),
      cfg.lastLevel ? 1 : 0,
      static_cast<unsigned long>(cfg.pulsesRemaining)
    );
    writeRow(channel + 1, row);
  }
#endif
}

}  // namespace LcdDisplay

void setup() {
  if (!Config::USE_USB_SERIAL) {
    pinMode(Config::RS485_DE_RE_PIN, OUTPUT);
    rs485SetTransmit(false);
  }

  Config::RS485_SERIAL.begin(Config::RS485_BAUD);

  pinMode(
    Config::KB_INTERLOCK_PIN,
    Config::INTERLOCK_USE_PULLUP ? INPUT_PULLUP : INPUT
  );

  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    pinMode(Config::PULSER_GATE_OUTPUT_PINS[channel], OUTPUT);
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[channel], LOW);

    pinMode(Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[channel], OUTPUT);
    digitalWrite(
      Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[channel],
      Config::ENABLE_TOGGLE_ACTIVE_HIGH ? LOW : HIGH
    );

    Runtime::channels[channel].mode = Runtime::OutputMode::Off;
    Runtime::channels[channel].pulseDurationMs = 100;
    Runtime::channels[channel].pulseCount = 1;
    Runtime::channels[channel].pulsesRemaining = 0;
    Runtime::channels[channel].pulseStartMs = millis();
    Runtime::channels[channel].pulseTrainInLowGap = false;
    Runtime::channels[channel].lastLevel = false;
  }

  const uint8_t statusPinMode = Config::STATUS_USE_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT;
  for (uint8_t channel = 0; channel < Config::CHANNEL_COUNT; ++channel) {
    pinMode(Config::ENABLE_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::POWER_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::OVERCURRENT_STATUS_PINS[channel], statusPinMode);
    pinMode(Config::GATED_STATUS_PINS[channel], statusPinMode);
  }

  Runtime::lastCommandMillis = millis();
  Runtime::lastModbusByteMillis = 0;
  Runtime::lastModbusError = Runtime::ModbusError::None;

  setupChannelTimers();
  setupLcdTimer();

  if (Config::ENABLE_DEBUG_LCD) {
    LcdDisplay::begin();
  }
}

void loop() {
  serviceEnableToggleOutputs();
  pollModbus();

  if (isAnyOverCurrentAsserted()) {
    Runtime::faultLatched = true;
  }

  applyOutputs();
  LcdDisplay::update();
}
