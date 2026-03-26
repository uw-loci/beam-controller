// =============================================================================
// BCON Mega Modbus Firmware
// Target : Arduino Mega 2560 (ATmega2560)
// Serial : USB-CDC (Serial) used as Modbus RTU port when USE_USB_SERIAL = 1
//          Hardware RS-485 (Serial1 + DE/RE pin) otherwise
// LCD    : 20x4 I2C LCD via LiquidCrystal_I2C (auto-detects 0x27 / 0x3F)
// =============================================================================

#include <Arduino.h>
#include <string.h>
#include <avr/wdt.h>

// ---------------------------------------------------------------------------
// Forward-declare Runtime enums so the early-init hook compiles cleanly
// before the full namespace body is seen by the compiler.
// ---------------------------------------------------------------------------
namespace Runtime {
  enum class OutputMode    : uint8_t;
  enum class TopLevelState : uint8_t;
  enum class ModbusError   : uint16_t;
}

// ---------------------------------------------------------------------------
// Early-init hook  (.init3 runs before main() / before C++ constructors)
// Capture MCUSR and immediately disable the watchdog so a previous WDT reset
// cannot cause an infinite boot-loop.
// ---------------------------------------------------------------------------
uint8_t g_resetFlags __attribute__((section(".noinit")));

void captureResetFlagsAndDisableWatchdog() __attribute__((section(".init3")));
void captureResetFlagsAndDisableWatchdog() {
  g_resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// ---------------------------------------------------------------------------
// Build-time feature selection
// ---------------------------------------------------------------------------
#define BCON_MODBUS_USE_USB_SERIAL 1   // 1 = USB-CDC (Serial), 0 = Serial1 RS-485
#define BCON_ENABLE_I2C_LCD        1   // 1 = I2C LCD enabled, 0 = disabled

#if BCON_MODBUS_USE_USB_SERIAL
  #define BCON_MODBUS_PORT Serial
#else
  #define BCON_MODBUS_PORT Serial1
#endif

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===========================================================================
// Config  –  compile-time constants only, no mutable state here
// ===========================================================================
namespace Config {

  constexpr uint8_t  CHANNEL_COUNT   = 3;

  HardwareSerial    &RS485_SERIAL    = BCON_MODBUS_PORT;
  constexpr uint32_t RS485_BAUD      = 115200;
  constexpr uint8_t  RS485_DE_RE_PIN = 2;   // RS-485 driver-enable / receive-enable
  constexpr uint8_t  MODBUS_SLAVE_ID = 1;
  constexpr bool     USE_USB_SERIAL  = (BCON_MODBUS_USE_USB_SERIAL != 0);

  // Software watchdog: outputs are forced off when the host stops sending
  // heartbeat writes within this window.
  constexpr uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 1500;
  constexpr uint32_t HEARTBEAT_MIN_MS            = 50;
  constexpr uint32_t HEARTBEAT_MAX_MS            = 60000;

  // Grace window after boot before the SW watchdog is enforced; prevents a
  // cold-start disconnect before the host has had time to connect.
  constexpr uint32_t WATCHDOG_BOOT_GRACE_MS = 8000;

  constexpr uint32_t DEFAULT_TELEMETRY_MS = 1500;

  // Interlock input
  constexpr uint8_t KB_INTERLOCK_PIN      = 22;
  constexpr bool    INTERLOCK_ACTIVE_HIGH = true;
  constexpr bool    INTERLOCK_USE_PULLUP  = false;

  // Pulse-gate outputs (one per channel)
  constexpr uint8_t PULSER_GATE_OUTPUT_PINS[CHANNEL_COUNT] = {5, 6, 7};

  // Enable-toggle outputs (momentary pulse to toggle external enable latch)
  constexpr uint8_t  PULSER_ENABLE_TOGGLE_OUTPUT_PINS[CHANNEL_COUNT] = {8, 9, 10};
  constexpr bool     ENABLE_TOGGLE_ACTIVE_HIGH = true;
  constexpr uint16_t ENABLE_TOGGLE_PULSE_MS    = 100;

  // Status inputs (active-low)
  constexpr bool    STATUS_USE_INTERNAL_PULLUPS            = true;
  constexpr uint8_t ENABLE_STATUS_PINS[CHANNEL_COUNT]      = {23, 27, 31};
  constexpr uint8_t POWER_STATUS_PINS[CHANNEL_COUNT]       = {24, 28, 32};
  constexpr uint8_t OVERCURRENT_STATUS_PINS[CHANNEL_COUNT] = {25, 29, 33};
  constexpr uint8_t GATED_STATUS_PINS[CHANNEL_COUNT]       = {26, 30, 34};

  // Pulse parameter bounds
  constexpr uint32_t PULSE_DURATION_MIN_MS = 1;
  constexpr uint32_t PULSE_DURATION_MAX_MS = 60000;
  constexpr uint32_t PULSE_COUNT_MIN       = 1;
  constexpr uint32_t PULSE_COUNT_MAX       = 10000;

  // Modbus framing
  constexpr size_t   MODBUS_RX_BUFFER_SIZE = 256;
  constexpr uint32_t MODBUS_FRAME_GAP_MS   = 4;  // inter-frame silence (>= 3.5 char-times)

  // LCD
  constexpr bool     ENABLE_DEBUG_LCD         = (BCON_ENABLE_I2C_LCD != 0);
  constexpr uint8_t  LCD_I2C_ADDRESS          = 0x27; // most common PCF8574 backpack
  constexpr uint8_t  LCD_I2C_ADDRESS_FALLBACK = 0x3F; // alt PCF8574A backpack
  constexpr uint8_t  LCD_COLS                 = 20;
  constexpr uint8_t  LCD_ROWS                 = 4;
  constexpr uint32_t LCD_REFRESH_MS           = 1000;

  constexpr const char *LCD_LABEL_WATCHDOG              = "WDG";
  constexpr const char *LCD_LABEL_INTERLOCK             = "INT";
  constexpr const char *LCD_LABEL_FAULT                 = "FLT";
  constexpr const char *LCD_LABEL_CHANNELS[CHANNEL_COUNT] = {"CH1", "CH2", "CH3"};

}  // namespace Config

// ===========================================================================
// Runtime  –  mutable state
// ===========================================================================
namespace Runtime {

  enum class OutputMode : uint8_t {
    Off        = 0,
    DC         = 1,
    Pulse      = 2,
    PulseTrain = 3
  };

  enum class TopLevelState : uint8_t {
    Ready          = 0,
    SafeInterlock  = 1,
    SafeWatchdog   = 2,
    FaultLatched   = 3
  };

  enum class ModbusError : uint16_t {
    None              = 0,
    IllegalFunction   = 1,
    IllegalAddress    = 2,
    IllegalValue      = 3,
    DeviceFailure     = 4,
    NotReady          = 10,
    FaultStillActive  = 11,
    InterlockNotReady = 12,
    BufferOverflow    = 13
  };

  struct ChannelControl {
    OutputMode mode              = OutputMode::Off;
    uint32_t   pulseDurationMs   = 100;
    uint32_t   pulseCount        = 1;
    uint32_t   pulsesRemaining   = 0;
    uint32_t   pulseStartMs      = 0;
    bool       pulseTrainInLowGap = false;
    bool       lastLevel         = false;
  };

  ChannelControl channels[Config::CHANNEL_COUNT];

  // ISR-decremented 1 ms countdown timers for pulse state-machines
  volatile uint32_t chCountdownMs[Config::CHANNEL_COUNT]        = {0};
  volatile bool     chEventFlag[Config::CHANNEL_COUNT]          = {false};

  // ISR-decremented enable-toggle pulse timers
  volatile uint16_t enableToggleCountdownMs[Config::CHANNEL_COUNT] = {0};
  volatile bool     enableToggleEventFlag[Config::CHANNEL_COUNT]   = {false};

  // ISR-decremented LCD refresh timer
  volatile uint32_t lcdCountdownMs = 0;
  volatile bool     lcdEventFlag   = false;

  // Software watchdog
  uint32_t lastCommandMillis       = 0;
  uint32_t watchdogTimeoutMs       = Config::DEFAULT_WATCHDOG_TIMEOUT_MS;
  uint32_t watchdogGraceDeadlineMs = 0;

  uint32_t    telemetryPeriodMs = Config::DEFAULT_TELEMETRY_MS;
  bool        faultLatched      = false;
  ModbusError lastModbusError   = ModbusError::None;

  // Modbus receive buffer
  uint8_t  modbusRxBuffer[Config::MODBUS_RX_BUFFER_SIZE] = {0};
  size_t   modbusRxIndex        = 0;
  uint32_t lastModbusByteMillis = 0;

}  // namespace Runtime

// ===========================================================================
// ModbusMap  –  register addresses
// ===========================================================================
namespace ModbusMap {

  // System control (R/W unless noted)
  constexpr uint16_t REG_WATCHDOG_MS  = 0;   // SW watchdog timeout ms
  constexpr uint16_t REG_TELEMETRY_MS = 1;   // telemetry period ms (informational)
  constexpr uint16_t REG_COMMAND      = 2;   // W: 0=nop 1=all-off 2/3=clear-fault

  // Per-channel control  (base = 10*(ch+1), fields 0-3)
  //   field 0 = mode   (0=Off 1=DC 2=Pulse 3=PulseTrain)
  //   field 1 = pulse_duration_ms
  //   field 2 = pulse_count
  //   field 3 = enable_toggle (write 1 to pulse)
  constexpr uint16_t REG_CH1_MODE          = 10;
  constexpr uint16_t REG_CH1_PULSE_MS      = 11;
  constexpr uint16_t REG_CH1_COUNT         = 12;
  constexpr uint16_t REG_CH1_ENABLE_TOGGLE = 13;

  constexpr uint16_t REG_CH2_MODE          = 20;
  constexpr uint16_t REG_CH2_PULSE_MS      = 21;
  constexpr uint16_t REG_CH2_COUNT         = 22;
  constexpr uint16_t REG_CH2_ENABLE_TOGGLE = 23;

  constexpr uint16_t REG_CH3_MODE          = 30;
  constexpr uint16_t REG_CH3_PULSE_MS      = 31;
  constexpr uint16_t REG_CH3_COUNT         = 32;
  constexpr uint16_t REG_CH3_ENABLE_TOGGLE = 33;

  // System status (R)
  constexpr uint16_t REG_SYS_STATE     = 100;
  constexpr uint16_t REG_SYS_REASON    = 101;
  constexpr uint16_t REG_FAULT_LATCHED = 102;
  constexpr uint16_t REG_INTERLOCK_OK  = 103;
  constexpr uint16_t REG_WATCHDOG_OK   = 104;
  constexpr uint16_t REG_LAST_ERROR    = 105;

  // Per-channel status block (R)
  // Address = REG_CH_STATUS_BASE + stride*ch + field
  //   0=mode 1=pulse_ms 2=count 3=remaining
  //   4=enable_status 5=power_status 6=overcurrent 7=gated 8=output_level
  constexpr uint16_t REG_CH_STATUS_BASE   = 110;
  constexpr uint16_t REG_CH_STATUS_STRIDE = 10;

}  // namespace ModbusMap

// ===========================================================================
// Helper utilities
// ===========================================================================

static bool activeLowRead(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static bool isInterlockSatisfied() {
  const bool raw = (digitalRead(Config::KB_INTERLOCK_PIN) == HIGH);
  return Config::INTERLOCK_ACTIVE_HIGH ? raw : !raw;
}

// Software watchdog: also returns true during the boot grace window so the
// system stays in Ready state until the host has had a chance to connect.
static bool isWatchdogHealthy() {
  const uint32_t now = millis();
  if (now < Runtime::watchdogGraceDeadlineMs) return true;
  return (now - Runtime::lastCommandMillis) <= Runtime::watchdogTimeoutMs;
}

static bool isAnyOverCurrentAsserted() {
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    if (activeLowRead(Config::OVERCURRENT_STATUS_PINS[ch])) return true;
  }
  return false;
}

static Runtime::TopLevelState evaluateTopLevelState() {
  if (!isInterlockSatisfied()) return Runtime::TopLevelState::SafeInterlock;
  if (!isWatchdogHealthy())    return Runtime::TopLevelState::SafeWatchdog;
  if (Runtime::faultLatched)   return Runtime::TopLevelState::FaultLatched;
  return Runtime::TopLevelState::Ready;
}

static uint16_t topLevelStateToCode(Runtime::TopLevelState s) {
  return static_cast<uint16_t>(s);
}

static uint16_t topLevelReasonCode(Runtime::TopLevelState s) {
  switch (s) {
    case Runtime::TopLevelState::Ready:         return 0;
    case Runtime::TopLevelState::SafeInterlock: return 1;
    case Runtime::TopLevelState::SafeWatchdog:  return 2;
    case Runtime::TopLevelState::FaultLatched:  return 3;
    default:                                    return 0;
  }
}

static uint16_t modeToCode(Runtime::OutputMode m) {
  switch (m) {
    case Runtime::OutputMode::Off:        return 0;
    case Runtime::OutputMode::DC:         return 1;
    case Runtime::OutputMode::Pulse:      return 2;
    case Runtime::OutputMode::PulseTrain: return 3;
    default:                              return 0;
  }
}

static const char *modeToShortString(Runtime::OutputMode m) {
  switch (m) {
    case Runtime::OutputMode::Off:        return "OFF";
    case Runtime::OutputMode::DC:         return "DC ";
    case Runtime::OutputMode::Pulse:      return "PUL";
    case Runtime::OutputMode::PulseTrain: return "TRN";
    default:                              return "UNK";
  }
}

static void setModbusError(Runtime::ModbusError err) {
  Runtime::lastModbusError = err;
}

// ===========================================================================
// RS-485 direction control
// ===========================================================================

static void rs485SetTransmit(bool enabled) {
  if (Config::USE_USB_SERIAL) { (void)enabled; return; }
  digitalWrite(Config::RS485_DE_RE_PIN, enabled ? HIGH : LOW);
}

// ===========================================================================
// Enable-toggle output (momentary pulse to external enable latch)
// ===========================================================================

static void pulseEnableToggle(uint8_t ch) {
  const uint8_t pin   = Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[ch];
  const uint8_t level = Config::ENABLE_TOGGLE_ACTIVE_HIGH ? HIGH : LOW;
  digitalWrite(pin, level);
  noInterrupts();
  Runtime::enableToggleCountdownMs[ch] = Config::ENABLE_TOGGLE_PULSE_MS;
  Runtime::enableToggleEventFlag[ch]   = false;
  interrupts();
}

static void serviceEnableToggleOutputs() {
  const uint8_t inactive = Config::ENABLE_TOGGLE_ACTIVE_HIGH ? LOW : HIGH;
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    bool ev = false;
    noInterrupts();
    if (Runtime::enableToggleEventFlag[ch]) {
      Runtime::enableToggleEventFlag[ch] = false;
      ev = true;
    }
    interrupts();
    if (ev) digitalWrite(Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[ch], inactive);
  }
}

// ===========================================================================
// Hardware timer ISRs and setup
// 1 ms CTC: TIMER1/3/4 for per-channel countdowns, TIMER5 for LCD + enable-toggle
// ===========================================================================

static void channelTimerTick(uint8_t ch) {
  if (Runtime::chCountdownMs[ch] > 0) {
    --Runtime::chCountdownMs[ch];
    if (Runtime::chCountdownMs[ch] == 0) Runtime::chEventFlag[ch] = true;
  }
}

ISR(TIMER1_COMPA_vect) { channelTimerTick(0); }
ISR(TIMER3_COMPA_vect) { channelTimerTick(1); }
ISR(TIMER4_COMPA_vect) { channelTimerTick(2); }

ISR(TIMER5_COMPA_vect) {
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    if (Runtime::enableToggleCountdownMs[ch] > 0) {
      --Runtime::enableToggleCountdownMs[ch];
      if (Runtime::enableToggleCountdownMs[ch] == 0)
        Runtime::enableToggleEventFlag[ch] = true;
    }
  }
  if (Runtime::lcdCountdownMs > 0) {
    --Runtime::lcdCountdownMs;
    if (Runtime::lcdCountdownMs == 0) Runtime::lcdEventFlag = true;
  }
}

// 16 MHz / 64 prescaler / (249+1) = 1000 Hz  =>  1 ms per tick
static void setupChannelTimers() {
  noInterrupts();

  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0; OCR1A = 249;
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  TCCR3A = 0; TCCR3B = 0; TCNT3 = 0; OCR3A = 249;
  TCCR3B |= (1 << WGM32) | (1 << CS31) | (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);

  TCCR4A = 0; TCCR4B = 0; TCNT4 = 0; OCR4A = 249;
  TCCR4B |= (1 << WGM42) | (1 << CS41) | (1 << CS40);
  TIMSK4 |= (1 << OCIE4A);

  interrupts();
}

static void setupLcdTimer() {
  noInterrupts();

  TCCR5A = 0; TCCR5B = 0; TCNT5 = 0; OCR5A = 249;
  TCCR5B |= (1 << WGM52) | (1 << CS51) | (1 << CS50);
  TIMSK5 |= (1 << OCIE5A);

  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    Runtime::enableToggleCountdownMs[ch] = 0;
    Runtime::enableToggleEventFlag[ch]   = false;
  }
  Runtime::lcdCountdownMs = 0;
  Runtime::lcdEventFlag   = false;

  interrupts();
}

static void armChannelTimer(uint8_t ch, uint32_t ms) {
  noInterrupts();
  Runtime::chCountdownMs[ch] = ms;
  Runtime::chEventFlag[ch]   = false;
  interrupts();
}

static void clearChannelTimer(uint8_t ch) {
  noInterrupts();
  Runtime::chCountdownMs[ch] = 0;
  Runtime::chEventFlag[ch]   = false;
  interrupts();
}

static void armLcdTimer(uint32_t ms) {
  noInterrupts();
  Runtime::lcdCountdownMs = ms;
  Runtime::lcdEventFlag   = false;
  interrupts();
}

static bool consumeLcdTimerEvent() {
  bool ev = false;
  noInterrupts();
  if (Runtime::lcdEventFlag) { Runtime::lcdEventFlag = false; ev = true; }
  interrupts();
  return ev;
}

// ===========================================================================
// Channel output control
// ===========================================================================

static void forceAllOutputsLow() {
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
    Runtime::channels[ch].lastLevel = false;
  }
}

static void setChannelOff(uint8_t ch) {
  Runtime::ChannelControl &c = Runtime::channels[ch];
  c.mode               = Runtime::OutputMode::Off;
  c.pulsesRemaining    = 0;
  c.pulseTrainInLowGap = false;
  c.lastLevel          = false;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
  clearChannelTimer(ch);
}

static void setChannelDC(uint8_t ch) {
  Runtime::ChannelControl &c = Runtime::channels[ch];
  c.mode               = Runtime::OutputMode::DC;
  c.pulsesRemaining    = 0;
  c.pulseTrainInLowGap = false;
  c.lastLevel          = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], HIGH);
  clearChannelTimer(ch);
}

static void setChannelPulse(uint8_t ch, uint32_t durationMs) {
  Runtime::ChannelControl &c = Runtime::channels[ch];
  c.mode               = Runtime::OutputMode::Pulse;
  c.pulseDurationMs    = durationMs;
  c.pulseCount         = 1;
  c.pulsesRemaining    = 1;
  c.pulseStartMs       = millis();
  c.pulseTrainInLowGap = false;
  c.lastLevel          = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], HIGH);
  armChannelTimer(ch, durationMs);
}

static void setChannelPulseTrain(uint8_t ch, uint32_t durationMs, uint32_t count) {
  Runtime::ChannelControl &c = Runtime::channels[ch];
  c.mode               = Runtime::OutputMode::PulseTrain;
  c.pulseDurationMs    = durationMs;
  c.pulseCount         = count;
  c.pulsesRemaining    = count;
  c.pulseStartMs       = millis();
  c.pulseTrainInLowGap = false;
  c.lastLevel          = true;
  digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], HIGH);
  armChannelTimer(ch, durationMs);
}

// Called every loop() to service pulse-train state machines
static void applyOutputs() {
  const Runtime::TopLevelState state = evaluateTopLevelState();
  if (state != Runtime::TopLevelState::Ready) {
    forceAllOutputsLow();
    return;
  }

  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    Runtime::ChannelControl &c = Runtime::channels[ch];
    switch (c.mode) {

      case Runtime::OutputMode::Off:
        if (c.lastLevel) {
          c.lastLevel = false;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
        }
        clearChannelTimer(ch);
        break;

      case Runtime::OutputMode::DC:
        if (!c.lastLevel) {
          c.lastLevel = true;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], HIGH);
        }
        clearChannelTimer(ch);
        break;

      case Runtime::OutputMode::Pulse: {
        bool ev = false;
        noInterrupts();
        if (Runtime::chEventFlag[ch]) { Runtime::chEventFlag[ch] = false; ev = true; }
        interrupts();
        if (!ev) break;

        c.lastLevel       = false;
        c.mode            = Runtime::OutputMode::Off;
        c.pulsesRemaining = 0;
        digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
        clearChannelTimer(ch);
        break;
      }

      case Runtime::OutputMode::PulseTrain: {
        bool ev = false;
        noInterrupts();
        if (Runtime::chEventFlag[ch]) { Runtime::chEventFlag[ch] = false; ev = true; }
        interrupts();
        if (!ev) break;

        if (c.lastLevel) {
          // End of HIGH phase: drive LOW, decrement remaining count
          c.lastLevel = false;
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
          if (c.pulsesRemaining > 0) --c.pulsesRemaining;

          if (c.pulsesRemaining == 0) {
            c.mode               = Runtime::OutputMode::Off;
            c.pulseTrainInLowGap = false;
            clearChannelTimer(ch);
          } else {
            c.pulseTrainInLowGap = true;
            c.pulseStartMs       = millis();
            armChannelTimer(ch, c.pulseDurationMs); // gap == pulse width
          }
        } else {
          // End of LOW gap: drive HIGH for next pulse
          c.pulseTrainInLowGap = false;
          c.lastLevel          = true;
          c.pulseStartMs       = millis();
          digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], HIGH);
          armChannelTimer(ch, c.pulseDurationMs);

          if (c.pulsesRemaining == 0) {
            c.mode      = Runtime::OutputMode::Off;
            c.lastLevel = false;
            digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
            clearChannelTimer(ch);
          }
        }
        break;
      }

      default:
        c.lastLevel = false;
        digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);
        clearChannelTimer(ch);
        break;
    }
  }
}

// ===========================================================================
// Modbus register address decoders
// ===========================================================================

static bool decodeChannelControlRegister(uint16_t reg, uint8_t &ch, uint8_t &field) {
  if (reg < 10 || reg > 33) return false;
  const uint8_t decade = static_cast<uint8_t>(reg / 10);
  const uint8_t units  = static_cast<uint8_t>(reg % 10);
  if (decade < 1 || decade > 3 || units > 3) return false;
  ch    = decade - 1;
  field = units;
  return true;
}

static bool decodeChannelStatusRegister(uint16_t reg, uint8_t &ch, uint8_t &field) {
  if (reg < ModbusMap::REG_CH_STATUS_BASE) return false;
  const uint16_t delta = reg - ModbusMap::REG_CH_STATUS_BASE;
  const uint8_t  idx   = static_cast<uint8_t>(delta / ModbusMap::REG_CH_STATUS_STRIDE);
  const uint8_t  off   = static_cast<uint8_t>(delta % ModbusMap::REG_CH_STATUS_STRIDE);
  if (idx >= Config::CHANNEL_COUNT || off > 8) return false;
  ch    = idx;
  field = off;
  return true;
}

// ===========================================================================
// Modbus register read / write
// ===========================================================================

static bool readHoldingRegister(uint16_t reg, uint16_t &out) {
  switch (reg) {
    case ModbusMap::REG_WATCHDOG_MS:
      out = static_cast<uint16_t>(Runtime::watchdogTimeoutMs);       return true;
    case ModbusMap::REG_TELEMETRY_MS:
      out = static_cast<uint16_t>(Runtime::telemetryPeriodMs);       return true;
    case ModbusMap::REG_COMMAND:
      out = 0;                                                        return true;
    case ModbusMap::REG_SYS_STATE:
      out = topLevelStateToCode(evaluateTopLevelState());             return true;
    case ModbusMap::REG_SYS_REASON:
      out = topLevelReasonCode(evaluateTopLevelState());              return true;
    case ModbusMap::REG_FAULT_LATCHED:
      out = Runtime::faultLatched ? 1 : 0;                           return true;
    case ModbusMap::REG_INTERLOCK_OK:
      out = isInterlockSatisfied() ? 1 : 0;                          return true;
    case ModbusMap::REG_WATCHDOG_OK:
      out = isWatchdogHealthy() ? 1 : 0;                             return true;
    case ModbusMap::REG_LAST_ERROR:
      out = static_cast<uint16_t>(Runtime::lastModbusError);        return true;
    default: break;
  }

  uint8_t ch = 0, field = 0;

  if (decodeChannelControlRegister(reg, ch, field)) {
    const Runtime::ChannelControl &c = Runtime::channels[ch];
    switch (field) {
      case 0: out = modeToCode(c.mode);                        return true;
      case 1: out = static_cast<uint16_t>(c.pulseDurationMs); return true;
      case 2: out = static_cast<uint16_t>(c.pulseCount);      return true;
      case 3: out = 0;                                         return true; // write-only
    }
  }

  if (decodeChannelStatusRegister(reg, ch, field)) {
    const Runtime::ChannelControl &c = Runtime::channels[ch];
    switch (field) {
      case 0: out = modeToCode(c.mode);                                             return true;
      case 1: out = static_cast<uint16_t>(c.pulseDurationMs);                      return true;
      case 2: out = static_cast<uint16_t>(c.pulseCount);                           return true;
      case 3: out = static_cast<uint16_t>(c.pulsesRemaining);                      return true;
      case 4: out = activeLowRead(Config::ENABLE_STATUS_PINS[ch])      ? 1 : 0;    return true;
      case 5: out = activeLowRead(Config::POWER_STATUS_PINS[ch])       ? 1 : 0;    return true;
      case 6: out = activeLowRead(Config::OVERCURRENT_STATUS_PINS[ch]) ? 1 : 0;    return true;
      case 7: out = activeLowRead(Config::GATED_STATUS_PINS[ch])       ? 1 : 0;    return true;
      case 8: out = c.lastLevel                                         ? 1 : 0;    return true;
      default: out = 0;                                                             return true;
    }
  }

  // Any register address not covered above (gap addresses 3-9, 14-19, 24-29,
  // or anything above the defined status blocks) returns 0 instead of causing
  // the firmware to abort the entire batch read with an exception.  A single
  // exception aborts ALL values in a multi-register read, which causes the
  // Python driver to treat the whole poll cycle as a failure and eventually
  // auto-disconnect.  Returning 0 for unused addresses is safe and keeps batch
  // reads from wider address ranges working correctly.
  out = 0;
  return true;
}

static bool writeHoldingRegister(uint16_t reg, uint16_t value, uint8_t &exOut) {
  exOut = 0x03;

  if (reg == ModbusMap::REG_WATCHDOG_MS) {
    if (value < Config::HEARTBEAT_MIN_MS || value > Config::HEARTBEAT_MAX_MS) {
      setModbusError(Runtime::ModbusError::IllegalValue); return false;
    }
    Runtime::watchdogTimeoutMs = value;
    setModbusError(Runtime::ModbusError::None); return true;
  }

  if (reg == ModbusMap::REG_TELEMETRY_MS) {
    Runtime::telemetryPeriodMs = value;
    setModbusError(Runtime::ModbusError::None); return true;
  }

  if (reg == ModbusMap::REG_COMMAND) {
    if (value == 0) { setModbusError(Runtime::ModbusError::None); return true; }
    if (value == 1) {
      for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) setChannelOff(ch);
      setModbusError(Runtime::ModbusError::None); return true;
    }
    if (value == 2 || value == 3) {
      if (isAnyOverCurrentAsserted()) {
        setModbusError(Runtime::ModbusError::FaultStillActive); exOut = 0x04; return false;
      }
      if (!isInterlockSatisfied()) {
        setModbusError(Runtime::ModbusError::InterlockNotReady); exOut = 0x04; return false;
      }
      Runtime::faultLatched = false;
      setModbusError(Runtime::ModbusError::None); return true;
    }
    setModbusError(Runtime::ModbusError::IllegalValue); return false;
  }

  uint8_t ch = 0, field = 0;
  if (!decodeChannelControlRegister(reg, ch, field)) {
    setModbusError(Runtime::ModbusError::IllegalAddress); exOut = 0x02; return false;
  }

  Runtime::ChannelControl &c = Runtime::channels[ch];

  if (field == 1) {
    if (value < Config::PULSE_DURATION_MIN_MS || value > Config::PULSE_DURATION_MAX_MS) {
      setModbusError(Runtime::ModbusError::IllegalValue); return false;
    }
    c.pulseDurationMs = value;
    setModbusError(Runtime::ModbusError::None); return true;
  }

  if (field == 2) {
    if (value < Config::PULSE_COUNT_MIN || value > Config::PULSE_COUNT_MAX) {
      setModbusError(Runtime::ModbusError::IllegalValue); return false;
    }
    c.pulseCount = value;
    setModbusError(Runtime::ModbusError::None); return true;
  }

  if (field == 3) {
    if (value != 1) { setModbusError(Runtime::ModbusError::IllegalValue); return false; }
    pulseEnableToggle(ch);
    setModbusError(Runtime::ModbusError::None); return true;
  }

  if (field == 0) {
    const Runtime::TopLevelState state = evaluateTopLevelState();

    if (value == 0) {
      setChannelOff(ch);
      setModbusError(Runtime::ModbusError::None); return true;
    }
    if (state != Runtime::TopLevelState::Ready) {
      setModbusError(Runtime::ModbusError::NotReady); exOut = 0x04; return false;
    }
    if (value == 1) {
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[ch])) pulseEnableToggle(ch);
      setChannelDC(ch);
      setModbusError(Runtime::ModbusError::None); return true;
    }
    if (value == 2) {
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[ch])) pulseEnableToggle(ch);
      if (c.pulseCount <= 1) setChannelPulse(ch, c.pulseDurationMs);
      else                   setChannelPulseTrain(ch, c.pulseDurationMs, c.pulseCount);
      setModbusError(Runtime::ModbusError::None); return true;
    }
    if (value == 3) {
      if (c.pulseCount < 2) { setModbusError(Runtime::ModbusError::IllegalValue); return false; }
      if (!activeLowRead(Config::ENABLE_STATUS_PINS[ch])) pulseEnableToggle(ch);
      setChannelPulseTrain(ch, c.pulseDurationMs, c.pulseCount);
      setModbusError(Runtime::ModbusError::None); return true;
    }
    setModbusError(Runtime::ModbusError::IllegalValue); return false;
  }

  setModbusError(Runtime::ModbusError::IllegalAddress); exOut = 0x02; return false;
}

// ===========================================================================
// Modbus RTU framing  –  CRC-16, send, receive, dispatch
// ===========================================================================

static uint16_t modbusCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}

static void modbusSendFrame(const uint8_t *payload, size_t payloadLen) {
  uint8_t frame[260];
  if (payloadLen + 2 > sizeof(frame)) return;

  memcpy(frame, payload, payloadLen);
  const uint16_t crc    = modbusCrc16(payload, payloadLen);
  frame[payloadLen]     = static_cast<uint8_t>(crc & 0xFF);
  frame[payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  rs485SetTransmit(true);
  Config::RS485_SERIAL.write(frame, payloadLen + 2);
  Config::RS485_SERIAL.flush();
  rs485SetTransmit(false);
}

static void modbusSendException(uint8_t slaveId, uint8_t fc, uint8_t exCode) {
  const uint8_t payload[3] = {slaveId, static_cast<uint8_t>(fc | 0x80), exCode};
  modbusSendFrame(payload, 3);
}

static void processModbusFrame(const uint8_t *frame, size_t len) {
  if (len < 4) return;

  const uint8_t slaveId     = frame[0];
  const bool    isBroadcast = (slaveId == 0);
  if (!isBroadcast && slaveId != Config::MODBUS_SLAVE_ID) return;

  const uint16_t rxCrc  = static_cast<uint16_t>(frame[len - 2]) |
                          static_cast<uint16_t>(frame[len - 1] << 8);
  const uint16_t calcCrc = modbusCrc16(frame, len - 2);
  if (rxCrc != calcCrc) return;

  // Any valid frame resets the software watchdog
  Runtime::lastCommandMillis = millis();

  const uint8_t fc = frame[1];

  // --- FC 0x03: Read Holding Registers ---
  if (fc == 0x03) {
    if (len != 8) {
      if (!isBroadcast) modbusSendException(slaveId, fc, 0x03);
      setModbusError(Runtime::ModbusError::IllegalValue); return;
    }
    const uint16_t startReg = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    const uint16_t quantity = static_cast<uint16_t>((frame[4] << 8) | frame[5]);
    if (quantity == 0 || quantity > 125) {
      if (!isBroadcast) modbusSendException(slaveId, fc, 0x03);
      setModbusError(Runtime::ModbusError::IllegalValue); return;
    }
    uint8_t payload[260];
    payload[0] = slaveId;
    payload[1] = fc;
    payload[2] = static_cast<uint8_t>(quantity * 2);
    for (uint16_t i = 0; i < quantity; ++i) {
      uint16_t val = 0;
      if (!readHoldingRegister(static_cast<uint16_t>(startReg + i), val)) {
        if (!isBroadcast) modbusSendException(slaveId, fc, 0x02);
        setModbusError(Runtime::ModbusError::IllegalAddress); return;
      }
      payload[3 + i * 2] = static_cast<uint8_t>((val >> 8) & 0xFF);
      payload[4 + i * 2] = static_cast<uint8_t>(val & 0xFF);
    }
    if (!isBroadcast) modbusSendFrame(payload, static_cast<size_t>(3 + quantity * 2));
    setModbusError(Runtime::ModbusError::None); return;
  }

  // --- FC 0x06: Write Single Register ---
  if (fc == 0x06) {
    if (len != 8) {
      if (!isBroadcast) modbusSendException(slaveId, fc, 0x03);
      setModbusError(Runtime::ModbusError::IllegalValue); return;
    }
    const uint16_t reg   = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    const uint16_t value = static_cast<uint16_t>((frame[4] << 8) | frame[5]);
    uint8_t exCode = 0x03;
    if (!writeHoldingRegister(reg, value, exCode)) {
      if (!isBroadcast) modbusSendException(slaveId, fc, exCode); return;
    }
    if (!isBroadcast) modbusSendFrame(frame, 6);
    return;
  }

  // --- FC 0x10: Write Multiple Registers ---
  if (fc == 0x10) {
    if (len < 11) {
      if (!isBroadcast) modbusSendException(slaveId, fc, 0x03);
      setModbusError(Runtime::ModbusError::IllegalValue); return;
    }
    const uint16_t startReg  = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    const uint16_t quantity  = static_cast<uint16_t>((frame[4] << 8) | frame[5]);
    const uint8_t  byteCount = frame[6];
    if (quantity == 0 || quantity > 123 || byteCount != quantity * 2 ||
        len != static_cast<size_t>(9 + byteCount)) {
      if (!isBroadcast) modbusSendException(slaveId, fc, 0x03);
      setModbusError(Runtime::ModbusError::IllegalValue); return;
    }
    for (uint16_t i = 0; i < quantity; ++i) {
      const size_t   off   = 7 + i * 2;
      const uint16_t value = static_cast<uint16_t>((frame[off] << 8) | frame[off + 1]);
      uint8_t exCode = 0x03;
      if (!writeHoldingRegister(static_cast<uint16_t>(startReg + i), value, exCode)) {
        if (!isBroadcast) modbusSendException(slaveId, fc, exCode); return;
      }
    }
    if (!isBroadcast) {
      uint8_t payload[6] = {
        slaveId, fc,
        static_cast<uint8_t>((startReg >> 8) & 0xFF),
        static_cast<uint8_t>(startReg & 0xFF),
        static_cast<uint8_t>((quantity >> 8) & 0xFF),
        static_cast<uint8_t>(quantity & 0xFF)
      };
      modbusSendFrame(payload, 6);
    }
    return;
  }

  // --- Unsupported function code ---
  if (!isBroadcast) modbusSendException(slaveId, fc, 0x01);
  setModbusError(Runtime::ModbusError::IllegalFunction);
}

static void pollModbus() {
  while (Config::RS485_SERIAL.available() > 0) {
    const int raw = Config::RS485_SERIAL.read();
    if (raw < 0) break;
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

// ===========================================================================
// LCD display subsystem
// ===========================================================================
namespace LcdDisplay {

#if BCON_ENABLE_I2C_LCD

LiquidCrystal_I2C lcdPrimary (Config::LCD_I2C_ADDRESS,          Config::LCD_COLS, Config::LCD_ROWS);
LiquidCrystal_I2C lcdFallback(Config::LCD_I2C_ADDRESS_FALLBACK, Config::LCD_COLS, Config::LCD_ROWS);
LiquidCrystal_I2C *lcd = &lcdPrimary;

bool lcdInitialized = false;
char lineCache[Config::LCD_ROWS][Config::LCD_COLS + 1] = {{0}};

static bool i2cAddressPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

// Write a padded row, skipping if content is unchanged (reduces I2C traffic)
static void writeRow(uint8_t row, const char *text) {
  if (!lcdInitialized || row >= Config::LCD_ROWS || text == nullptr) return;

  char padded[Config::LCD_COLS + 1];
  memset(padded, ' ', Config::LCD_COLS);
  padded[Config::LCD_COLS] = '\0';

  const size_t textLen = strlen(text);
  const size_t copyLen = (textLen < Config::LCD_COLS) ? textLen : Config::LCD_COLS;
  memcpy(padded, text, copyLen);

  if (memcmp(lineCache[row], padded, Config::LCD_COLS) == 0) return;

  memcpy(lineCache[row], padded, Config::LCD_COLS + 1);
  lcd->setCursor(0, row);
  lcd->print(padded);
}

#endif // BCON_ENABLE_I2C_LCD

void begin() {
#if BCON_ENABLE_I2C_LCD
  if (!Config::ENABLE_DEBUG_LCD) return;

  // Manually recover the I2C bus in case a previous crash left SDA held low.
  // Bit-bang 9 SCL clocks with SDA released so the slave releases the bus.
  pinMode(20, INPUT_PULLUP);   // SDA
  pinMode(21, OUTPUT);         // SCL
  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(21, LOW);  delayMicroseconds(5);
    digitalWrite(21, HIGH); delayMicroseconds(5);
  }
  pinMode(21, INPUT_PULLUP);   // release SCL back to open-drain before Wire takes over
  delayMicroseconds(10);

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT) || !defined(__AVR__)
  Wire.setWireTimeout(3000, true);  // 3 ms timeout; auto-reset bus on hang
#endif
  // Allow the I2C backpack power rail, oscillator, and HD44780 controller
  // to complete their power-on reset before we send any commands.
  // HD44780 datasheet requires >40 ms after VCC > 2.7 V.
  delay(50);

  // Auto-detect I2C address: 0x27 (PCF8574) or 0x3F (PCF8574A)
  if (i2cAddressPresent(Config::LCD_I2C_ADDRESS)) {
    lcd = &lcdPrimary;
  } else if (i2cAddressPresent(Config::LCD_I2C_ADDRESS_FALLBACK)) {
    lcd = &lcdFallback;
  } else {
    lcdInitialized = false;
    return; // no LCD found – continue silently
  }

  // Call init() twice: the first puts the HD44780 into 4-bit mode,
  // the second performs the full function-set / display-on sequence.
  // Some PCF8574 backpack variants require this double-init.
  lcd->init();
  delay(5);
  lcd->init();

  lcd->backlight();
  lcd->clear();
  delay(2); // clear command needs >1.52 ms to execute on the HD44780

  memset(lineCache, 0, sizeof(lineCache));
  lcdInitialized = true;

  writeRow(0, "BCON Pulser v2");
  writeRow(1, "Booting...");
  writeRow(2, "");
  writeRow(3, "");

  armLcdTimer(Config::LCD_REFRESH_MS);
#endif
}




void update() {
#if BCON_ENABLE_I2C_LCD
  if (!Config::ENABLE_DEBUG_LCD || !lcdInitialized) return;
  if (!consumeLcdTimerEvent()) return;

  // I2C / TWI safety strategy
  // ─────────────────────────
  // Problem 1 — timing: a full 4-row LCD write at 100 kHz takes ~80 ms,
  //   easily overflowing the 64-byte UART ring-buffer during an active poll.
  //   Fix: run I2C at 400 kHz (reduces full refresh to ~20 ms) and only
  //   update when no Modbus byte has arrived in the last 50 ms (i.e., we are
  //   solidly inside the ~250 ms inter-poll gap).
  //
  // Problem 2 — electrical noise: USB serial traffic induces glitches on the
  //   I2C lines that trigger spurious TWI interrupts, corrupting the PCF8574
  //   output register and killing the backlight.
  //   Fix: keep TWCR = 0 (TWI hardware off) between updates; re-enable only
  //   for the brief render window (~20 ms) then disable immediately after.
  const uint32_t now = millis();

  // "Host connected" flag: any valid Modbus frame in last 5 s.
  const bool hostPolling = (Runtime::lastCommandMillis > 0) &&
                           (now - Runtime::lastCommandMillis < 5000);

  // "Frame in flight" guard: a byte arrived very recently (frame being
  // received or just processed).  50 ms is very conservative — a full
  // request/response cycle takes < 2 ms — but gives margin for jitter.
  // Gate on framePending ALONE: hostPolling only becomes true after the
  // FIRST valid frame completes, so checking (hostPolling && framePending)
  // would miss the very first frame after boot or after >5 s idle, allowing
  // an I2C write to race with an incoming Modbus byte.
  const bool framePending = (Runtime::modbusRxIndex > 0) ||
      (Runtime::lastModbusByteMillis > 0 &&
       (now - Runtime::lastModbusByteMillis) < 50);

  static bool twiDisabled = false;

  if (framePending) {
    // Any recent serial activity — defer until we are in the quiet gap.
    if (!twiDisabled) { TWCR = 0; twiDisabled = true; }
    armLcdTimer(50);   // retry in 50 ms
    return;
  }

  // Safe window: re-enable TWI at 400 kHz, refresh all rows, then disable.
  Wire.begin();
  Wire.setClock(400000);   // 400 kHz ≈ 20 ms for a full 4-row refresh
#if defined(WIRE_HAS_TIMEOUT) || !defined(__AVR__)
  Wire.setWireTimeout(3000, true);
#endif
  lcd->backlight();

  // Row 0: system health
  char row0[Config::LCD_COLS + 1];
  snprintf(row0, sizeof(row0), "%s:%s %s:%s %s:%u",
    Config::LCD_LABEL_WATCHDOG,  isWatchdogHealthy()    ? "OK" : "NO",
    Config::LCD_LABEL_INTERLOCK, isInterlockSatisfied() ? "OK" : "NO",
    Config::LCD_LABEL_FAULT,     Runtime::faultLatched  ? 1 : 0);
  writeRow(0, row0);

  // Rows 1–3: per-channel status
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT && (ch + 1) < Config::LCD_ROWS; ++ch) {
    const Runtime::ChannelControl &c = Runtime::channels[ch];
    char row[Config::LCD_COLS + 1];
    snprintf(row, sizeof(row), "%s %s O:%u R:%lu",
      Config::LCD_LABEL_CHANNELS[ch],
      modeToShortString(c.mode),
      c.lastLevel ? 1 : 0,
      static_cast<unsigned long>(c.pulsesRemaining));
    writeRow(ch + 1, row);
  }

  // Disable TWI immediately after render to block noise during the next
  // inter-render idle period.  Always disabled regardless of host state —
  // re-enabled at the start of every update tick.
  TWCR = 0;
  twiDisabled = true;

  armLcdTimer(Config::LCD_REFRESH_MS);
#endif
}

} // namespace LcdDisplay

// ===========================================================================
// Arduino entry points
// ===========================================================================

void setup() {
  // The hardware WDT is enabled at the END of setup() to avoid timeout during
  // long init sequences (USB serial enumeration, I2C scan, LCD init).
  // The .init3 hook already disabled WDT early in the boot sequence.
  wdt_disable();

  if (!Config::USE_USB_SERIAL) {
    pinMode(Config::RS485_DE_RE_PIN, OUTPUT);
    rs485SetTransmit(false);
  }

  Config::RS485_SERIAL.begin(Config::RS485_BAUD);
  // Drain any garbage that accumulated in the UART buffer during reset
  while (Config::RS485_SERIAL.available()) Config::RS485_SERIAL.read();

  pinMode(Config::KB_INTERLOCK_PIN,
          Config::INTERLOCK_USE_PULLUP ? INPUT_PULLUP : INPUT);

  // Pulse-gate outputs and enable-toggle outputs
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    pinMode(Config::PULSER_GATE_OUTPUT_PINS[ch], OUTPUT);
    digitalWrite(Config::PULSER_GATE_OUTPUT_PINS[ch], LOW);

    pinMode(Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[ch], OUTPUT);
    digitalWrite(Config::PULSER_ENABLE_TOGGLE_OUTPUT_PINS[ch],
                 Config::ENABLE_TOGGLE_ACTIVE_HIGH ? LOW : HIGH);

    Runtime::channels[ch] = Runtime::ChannelControl(); // reset to defaults
  }

  // Status inputs
  const uint8_t statusMode = Config::STATUS_USE_INTERNAL_PULLUPS ? INPUT_PULLUP : INPUT;
  for (uint8_t ch = 0; ch < Config::CHANNEL_COUNT; ++ch) {
    pinMode(Config::ENABLE_STATUS_PINS[ch],      statusMode);
    pinMode(Config::POWER_STATUS_PINS[ch],       statusMode);
    pinMode(Config::OVERCURRENT_STATUS_PINS[ch], statusMode);
    pinMode(Config::GATED_STATUS_PINS[ch],       statusMode);
  }

  // Initialize SW watchdog timestamps
  // Start with no host command yet (prevents boot-state from being treated as active host polling).
  Runtime::lastCommandMillis       = 0;
  Runtime::watchdogGraceDeadlineMs = millis() + Config::WATCHDOG_BOOT_GRACE_MS;
  Runtime::lastModbusByteMillis    = 0;
  Runtime::lastModbusError         = Runtime::ModbusError::None;

  setupChannelTimers();
  setupLcdTimer();

  LcdDisplay::begin(); // Wire.begin() + address probe + lcd->init()

  // Enable hardware watchdog only after all init is complete.
  // loop() pets it every iteration.
  wdt_enable(WDTO_8S);
  wdt_reset();
}

void loop() {
  wdt_reset(); // pet hardware WDT; stalls >8 s cause an MCU reset

  serviceEnableToggleOutputs(); // deassert enable-toggle pins after pulse width
  pollModbus();                 // receive + dispatch Modbus RTU frames

  if (isAnyOverCurrentAsserted()) Runtime::faultLatched = true;

  applyOutputs();       // drive gate pins per channel state-machines
  LcdDisplay::update(); // redraw LCD if refresh timer fired
}
