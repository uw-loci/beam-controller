//=============================================================================
// BCON Mega Modbus Firmware
// Target  : Arduino Mega 2560 (ATmega2560)
// Comm    : Modbus RTU slave via RS-485 (Serial1 + DE/RE pin D17)
//           Set BCON_USE_USB_SERIAL 1 for bench/debug via USB-CDC (Serial)
// LCD     : 20x4 I2C LCD (LiquidCrystal_I2C, auto-detects 0x27 / 0x3F)
// Library : modbus-esp8266 (ModbusRTU) from libs/modbus-esp8266
// =============================================================================
//
// Modbus register map (holding registers, all R/W unless noted):
//
//  System control:
//    0  WATCHDOG_MS      SW watchdog timeout (50-60000 ms, default 1500)
//    1  TELEMETRY_MS     Informational host poll interval (not enforced)
//    2  COMMAND          0=NOP, 1=AllOff, 2/3=ClearFault, 4=ApplyStagedModes
//
//  Channel control  (base = ch*10, ch=1..3):
//    base+0  MODE           0=Off 1=DC 2=Pulse 3=PulseTrain
//    base+1  PULSE_MS       pulse duration ms (1-60000)
//    base+2  COUNT          pulse count (1-10000)
//    base+3  ENABLE_TOGGLE  write 1 = 100 ms enable-toggle pulse
//
//  System status (read-only):
//    100  SYS_STATE      0=Ready 1=SafeInterlock 2=SafeWatchdog 3=FaultLatched
//    101  SYS_REASON     mirrors SYS_STATE
//    102  FAULT_LATCHED  1 if OC fault latched
//    103  INTERLOCK_OK   1 if arm-beams interlock asserted
//    104  WATCHDOG_OK    1 if SW watchdog alive
//    105  LAST_ERROR     last error code (auto-cleared on read)
//
//  Per-channel status  (base = 110 + (ch-1)*10, offsets 0-8):
//    +0  mode           active output mode
//    +1  pulse_ms       configured pulse duration
//    +2  count          configured pulse count
//    +3  remaining      pulses remaining in current train
//    +4  enable_status  (not wired in HW rev 2026-03-17 - always 0)
//    +5  power_status   (not wired - always 0)
//    +6  overcurrent    (not wired - always 0)
//    +7  gated          (not wired - always 0)
//    +8  output_level   1 if gate pin currently HIGH
// =============================================================================

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include "bcon_types.h"

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
#define BCON_USE_USB_SERIAL  1   // 0 = RS-485 (production), 1 = USB-CDC (bench)
#define BCON_ENABLE_LCD      1   // 1 = enable 20x4 I2C LCD

// ---------------------------------------------------------------------------
// Early-init: capture reset flags and disable WDT before main() runs.
// Prevents infinite reset loop after a WDT-caused reboot.
// ---------------------------------------------------------------------------
uint8_t g_resetFlags __attribute__((section(".noinit")));

void captureAndDisableWdt() __attribute__((naked, section(".init3")));
void captureAndDisableWdt() {
    g_resetFlags = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

// ===========================================================================
// Pin assignments  (BCON HW Dev Spec Rev 2026-03-17)
// ===========================================================================
namespace Pins {
    // Gate drive outputs -- AND gate (SN74HCT08) -- TC4427 -- BNC
    constexpr uint8_t GATE_A  = A11;  // Pulser A gate drive
    constexpr uint8_t GATE_B  = A13;  // Pulser B gate drive
    constexpr uint8_t GATE_C  = A9;   // Pulser C gate drive
    constexpr uint8_t GATE[3] = { GATE_A, GATE_B, GATE_C };

    // Gate state indicator LEDs (blue panel LEDs)
    constexpr uint8_t LED_A  = 3;
    constexpr uint8_t LED_B  = 5;
    constexpr uint8_t LED_C  = 7;
    constexpr uint8_t LED[3] = { LED_A, LED_B, LED_C };

    // Enable-toggle outputs -- 2N7000 NMOS -- PVX DA-15 pin 9
    constexpr uint8_t ENA_A    = 12;  // Pulser A enable
    constexpr uint8_t ENA_B    = 11;  // Pulser B enable
    constexpr uint8_t ENA_C    = 10;  // Pulser C enable
    constexpr uint8_t ENA[3]   = { ENA_A, ENA_B, ENA_C };

    // RS-485 direction control
    constexpr uint8_t RS485_DE_RE = 17;

    // Interlock input: isolated Arm Beams signal from Knob Box (active HIGH)
    constexpr uint8_t INTERLOCK = A0;

    // I2C (fixed on Mega): SDA=D20, SCL=D21 (managed by Wire)
}

// ===========================================================================
// Timing & limits
// ===========================================================================
namespace Cfg {
    constexpr uint8_t  SLAVE_ID               = 1;
    constexpr uint32_t BAUD                   = 115200;

    constexpr uint16_t PULSE_TIMER_TICK_US    = 100;
    constexpr uint16_t PULSE_TIMER_OCR        = (uint16_t)((F_CPU / 8UL / (1000000UL / PULSE_TIMER_TICK_US)) - 1UL);

    constexpr uint32_t WD_DEFAULT_MS          = 1500;
    constexpr uint32_t WD_MIN_MS              = 50;
    constexpr uint32_t WD_MAX_MS              = 60000;
    constexpr uint32_t WD_BOOT_GRACE_MS       = 8000;

    constexpr uint32_t PULSE_MS_MIN           = 1;
    constexpr uint32_t PULSE_MS_MAX           = 60000;
    constexpr uint32_t COUNT_MIN              = 1;
    constexpr uint32_t COUNT_MAX              = 10000;

    constexpr uint32_t ENA_TOGGLE_MS          = 100;

    constexpr uint32_t LCD_REFRESH_MS         = 1000;
    constexpr uint32_t LCD_SERIAL_DEFER_MS    = 50;

    constexpr uint8_t  LCD_COLS               = 20;
    constexpr uint8_t  LCD_ROWS               = 4;
    constexpr uint8_t  LCD_ADDR_PRIMARY       = 0x27;
    constexpr uint8_t  LCD_ADDR_FALLBACK      = 0x3F;
}

// ===========================================================================
// Modbus register addresses
// ===========================================================================
namespace Reg {
    constexpr uint16_t WATCHDOG_MS   = 0;    // register address 0
    constexpr uint16_t TELEMETRY_MS  = 1;    // register address 1
    constexpr uint16_t COMMAND       = 2;

    // Channel control (ch = 1..3)
    inline constexpr uint16_t CH_BASE(uint8_t ch)       { return (uint16_t)(ch * 10); }
    inline constexpr uint16_t CH_MODE(uint8_t ch)       { return CH_BASE(ch) + 0; }
    inline constexpr uint16_t CH_PULSE_MS(uint8_t ch)   { return CH_BASE(ch) + 1; }
    inline constexpr uint16_t CH_COUNT(uint8_t ch)      { return CH_BASE(ch) + 2; }
    inline constexpr uint16_t CH_ENA_TOGGLE(uint8_t ch) { return CH_BASE(ch) + 3; }

    // System status
    constexpr uint16_t SYS_STATE     = 100;
    constexpr uint16_t SYS_REASON    = 101;
    constexpr uint16_t FAULT_LATCHED = 102;
    constexpr uint16_t INTERLOCK_OK  = 103;
    constexpr uint16_t WATCHDOG_OK   = 104;
    constexpr uint16_t LAST_ERROR    = 105;

    // Per-channel status (ch = 1..3)
    inline constexpr uint16_t CH_STAT_BASE(uint8_t ch) { return (uint16_t)(110 + (ch - 1) * 10); }
}

// ===========================================================================
// Global state
// ===========================================================================
static Channel  g_ch[3];
static TopState g_state        = TopState::SafeWatchdog;
static bool     g_faultLatched = false;
static uint32_t g_wdTimeoutMs  = Cfg::WD_DEFAULT_MS;
static uint32_t g_wdLastFeedMs = 0;
static uint32_t g_bootMs       = 0;
static uint16_t g_lastError    = 0;
static uint32_t g_telemMs      = Cfg::WD_DEFAULT_MS;
static uint32_t g_lastSerialByteMs  = 0;
static volatile bool    g_pulseOutputsEnabled = false;
static volatile uint8_t g_modeSyncPendingMask = 0;
static volatile uint8_t g_applyEventPendingMask = 0;
static bool g_commandClearPending = false;

// LCD row caches
static char g_lcdBuf [Cfg::LCD_ROWS][Cfg::LCD_COLS + 1];
static char g_lcdLast[Cfg::LCD_ROWS][Cfg::LCD_COLS + 1];
static uint32_t g_lcdLastRefreshMs = 0;

// ModbusRTU instance
ModbusRTU mb;

#if BCON_ENABLE_LCD
static LiquidCrystal_I2C *g_lcd = nullptr;
#endif

// ===========================================================================
// Helpers
// ===========================================================================
static inline uint32_t msToPulseTicks(uint32_t durationMs) {
    return ((durationMs * 1000UL) + (Cfg::PULSE_TIMER_TICK_US - 1UL)) / Cfg::PULSE_TIMER_TICK_US;
}

static inline void setGateAndLed(uint8_t idx, bool high) {
    digitalWrite(Pins::GATE[idx], high ? HIGH : LOW);
    digitalWrite(Pins::LED[idx],  high ? HIGH : LOW);
}

static inline uint8_t gatePortMaskFromState() {
    uint8_t mask = 0;
    if (g_ch[0].outputAsserted) mask |= _BV(3);
    if (g_ch[1].outputAsserted) mask |= _BV(5);
    if (g_ch[2].outputAsserted) mask |= _BV(1);
    return mask;
}

static inline void syncGatePortFromState() {
    const uint8_t gateMask = (uint8_t)(_BV(1) | _BV(3) | _BV(5));
    PORTK = (uint8_t)((PORTK & (uint8_t)~gateMask) | gatePortMaskFromState());
}

static void forceAllOutputsLow() {
    for (uint8_t i = 0; i < 3; i++) {
        g_ch[i].outputAsserted = false;
        digitalWrite(Pins::LED[i], LOW);
    }
    syncGatePortFromState();
}

static bool interlockOk() {
    return digitalRead(Pins::INTERLOCK) == HIGH;
}

static bool watchdogOk() {
    uint32_t now = millis();
    if ((now - g_bootMs) < Cfg::WD_BOOT_GRACE_MS) return true;
    return (now - g_wdLastFeedMs) <= g_wdTimeoutMs;
}

static void feedWatchdog() {
    g_wdLastFeedMs = millis();
}

static TopState evaluateState() {
    if (!interlockOk())  return TopState::SafeInterlock;
    if (!watchdogOk())   return TopState::SafeWatchdog;
    if (g_faultLatched)  return TopState::FaultLatched;
    return TopState::Ready;
}

static void triggerEnableToggle(uint8_t idx) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_ch[idx].enaToggleTicksRemaining = msToPulseTicks(Cfg::ENA_TOGGLE_MS);
        g_ch[idx].enaToggleActive         = true;
        digitalWrite(Pins::ENA[idx], HIGH);
    }
}

static void stopChannel(uint8_t idx) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        Channel& c            = g_ch[idx];
        g_modeSyncPendingMask &= (uint8_t)~(1U << idx);
        g_applyEventPendingMask &= (uint8_t)~(1U << idx);
        c.mode                = Mode::Off;
        c.requestedMode       = Mode::Off;
        c.pulsesLeft          = 0;
        c.phaseTicksRemaining = 0;
        c.inHighPhase         = false;
        c.outputAsserted      = false;
        c.applyPending        = false;
        digitalWrite(Pins::LED[idx], LOW);
    }
    syncGatePortFromState();
}

// ===========================================================================
// Timer-driven pulse engine
// ===========================================================================
static void pulseTimerSetup() {
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5  = 0;
    OCR5A  = Cfg::PULSE_TIMER_OCR;
    TCCR5B |= _BV(WGM52);  // CTC mode
    TCCR5B |= _BV(CS51);   // prescaler 8
    TIMSK5 |= _BV(OCIE5A); // compare A interrupt enable
}

ISR(TIMER5_COMPA_vect) {
    bool gateStateChanged = false;

    for (uint8_t idx = 0; idx < 3; idx++) {
        Channel& c = g_ch[idx];
        if (!c.enaToggleActive) continue;
        if (c.enaToggleTicksRemaining > 0) c.enaToggleTicksRemaining--;
        if (c.enaToggleTicksRemaining == 0) {
            digitalWrite(Pins::ENA[idx], LOW);
            c.enaToggleActive = false;
        }
    }

    uint8_t applyMask = g_applyEventPendingMask;
    if (applyMask != 0) g_applyEventPendingMask = 0;

    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((applyMask & (uint8_t)(1U << idx)) == 0) continue;

        Channel& c = g_ch[idx];
        c.applyPending = false;

        switch (c.requestedMode) {
            case Mode::Off:
                c.mode                = Mode::Off;
                c.pulsesLeft          = 0;
                c.phaseTicksRemaining = 0;
                c.inHighPhase         = false;
                c.outputAsserted      = false;
                digitalWrite(Pins::LED[idx], LOW);
                gateStateChanged = true;
                break;

            case Mode::DC:
                c.mode                = Mode::DC;
                c.pulsesLeft          = 0;
                c.phaseTicksRemaining = 0;
                c.inHighPhase         = false;
                c.outputAsserted      = true;
                digitalWrite(Pins::LED[idx], HIGH);
                gateStateChanged = true;
                break;

            case Mode::Pulse:
            case Mode::PulseTrain:
                c.mode                = (c.requestedMode == Mode::Pulse && c.count > 1)
                                        ? Mode::PulseTrain
                                        : c.requestedMode;
                c.pulsesLeft          = c.count;
                c.phaseTicksRemaining = msToPulseTicks(c.pulseMs);
                c.inHighPhase         = true;
                c.outputAsserted      = true;
                digitalWrite(Pins::LED[idx], HIGH);
                gateStateChanged = true;
                break;
        }
    }

    if (gateStateChanged) {
        syncGatePortFromState();
        gateStateChanged = false;
    }

    if (!g_pulseOutputsEnabled) return;

    for (uint8_t idx = 0; idx < 3; idx++) {
        Channel& c = g_ch[idx];
        if ((applyMask & (uint8_t)(1U << idx)) != 0) continue;
        if (c.mode != Mode::Pulse && c.mode != Mode::PulseTrain) continue;

        if (c.inHighPhase && !c.outputAsserted) {
            c.outputAsserted = true;
            digitalWrite(Pins::LED[idx], HIGH);
            gateStateChanged = true;
        }

        if (c.phaseTicksRemaining > 0) c.phaseTicksRemaining--;
        if (c.phaseTicksRemaining > 0) continue;

        if (c.inHighPhase) {
            c.outputAsserted = false;
            digitalWrite(Pins::LED[idx], LOW);
            gateStateChanged = true;
            if (c.pulsesLeft > 0) c.pulsesLeft--;
            if (c.pulsesLeft == 0) {
                c.mode                = Mode::Off;
                c.requestedMode       = Mode::Off;
                c.inHighPhase         = false;
                c.phaseTicksRemaining = 0;
                g_modeSyncPendingMask |= (uint8_t)(1U << idx);
            } else {
                c.inHighPhase         = false;
                c.phaseTicksRemaining = msToPulseTicks(c.pulseMs);
            }
        } else {
            c.inHighPhase = true;
            c.outputAsserted      = true;
            c.phaseTicksRemaining = msToPulseTicks(c.pulseMs);
            digitalWrite(Pins::LED[idx], HIGH);
            gateStateChanged = true;
        }
    }

    if (gateStateChanged) syncGatePortFromState();
}

static void syncRuntimeModeToRegisters() {
    uint8_t pendingMask = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pendingMask = g_modeSyncPendingMask;
        g_modeSyncPendingMask = 0;
    }
    bool clearCommand = g_commandClearPending;
    g_commandClearPending = false;
    if (!pendingMask && !clearCommand) return;

    mb.cbDisable();
    if (clearCommand) mb.Hreg(Reg::COMMAND, 0);
    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((pendingMask & (uint8_t)(1U << idx)) != 0) {
            Mode mode = Mode::Off;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                mode = g_ch[idx].mode;
            }
            if (mode == Mode::Off) mb.Hreg(Reg::CH_MODE(idx + 1), 0);
        }
    }
    mb.cbEnable(true);
}

static void applyDC() {
    if (g_state != TopState::Ready) {
        forceAllOutputsLow();
        return;
    }
    for (uint8_t i = 0; i < 3; i++) {
        if (g_ch[i].mode == Mode::DC) {
            g_ch[i].outputAsserted = true;
            digitalWrite(Pins::LED[i], HIGH);
        }
    }
    syncGatePortFromState();
}

// ===========================================================================
// Mode write helper (used by onSet callbacks)
// ===========================================================================
static void handleModeWrite(uint8_t idx, uint16_t val) {
    Mode newMode = (Mode)(val & 0x03);

    // Preserve the written control-register value so FC06 write verification passes.
    // Effective Pulse -> PulseTrain promotion happens later when the staged mode is applied.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_modeSyncPendingMask &= (uint8_t)~(1U << idx);
        g_ch[idx].requestedMode = newMode;
        g_ch[idx].applyPending = true;
    }
}

// ===========================================================================
// Modbus onSet callbacks
// ===========================================================================
uint16_t cbSetWatchdog(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::WD_MIN_MS && val <= Cfg::WD_MAX_MS) {
        g_wdTimeoutMs = val; return val;
    }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError);
    return (uint16_t)g_wdTimeoutMs;
}

uint16_t cbSetTelemetry(TRegister* reg, uint16_t val) {
    feedWatchdog(); g_telemMs = val; return val;
}

uint16_t cbSetCommand(TRegister* reg, uint16_t val) {
    feedWatchdog();
    switch (val) {
        case 0: break;
        case 1:
            for (uint8_t i = 0; i < 3; i++) stopChannel(i);
            // Disable callbacks while updating control registers to avoid triggering
            // cbSetCHxMode from within cbSetCommand (cross-callback writes).
            mb.cbDisable();
            for (uint8_t i = 0; i < 3; i++) mb.Hreg(Reg::CH_MODE(i + 1), 0);
            mb.cbEnable(true);
            break;
        case 4: {
            uint8_t applyMask = 0;
            uint8_t startMask = 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                for (uint8_t i = 0; i < 3; i++) {
                    if (!g_ch[i].applyPending) continue;
                    applyMask |= (uint8_t)(1U << i);
                    if (g_ch[i].requestedMode != Mode::Off) startMask |= (uint8_t)(1U << i);
                }
            }

            if (startMask != 0 && evaluateState() != TopState::Ready) {
                g_lastError = 10;
                mb.Hreg(Reg::LAST_ERROR, g_lastError);
                applyMask &= (uint8_t)~startMask;
            }

            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                g_applyEventPendingMask |= applyMask;
            }
            break;
        }
        case 2: case 3:
            if (!interlockOk()) { g_lastError = 12; mb.Hreg(Reg::LAST_ERROR, g_lastError); }
            else g_faultLatched = false;
            break;
        default: break;
    }
    // FC06 expects the register to echo the written value; clear non-zero commands later.
    if (val != 0) g_commandClearPending = true;
    return val;
}

// CH_MODE writes now stage requested modes. COMMAND=4 applies all pending
// channels together on the next timer event.
uint16_t cbSetCH1Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(0, val); return (uint16_t)g_ch[0].requestedMode; }
uint16_t cbSetCH2Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(1, val); return (uint16_t)g_ch[1].requestedMode; }
uint16_t cbSetCH3Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(2, val); return (uint16_t)g_ch[2].requestedMode; }

uint16_t cbSetCH1PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[0].pulseMs = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[0].pulseMs;
}
uint16_t cbSetCH2PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[1].pulseMs = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[1].pulseMs;
}
uint16_t cbSetCH3PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[2].pulseMs = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[2].pulseMs;
}

uint16_t cbSetCH1Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[0].count = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[0].count;
}
uint16_t cbSetCH2Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[1].count = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[1].count;
}
uint16_t cbSetCH3Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_ch[2].count = val; } return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[2].count;
}

uint16_t cbSetCH1EnaToggle(TRegister* reg, uint16_t val) { feedWatchdog(); if (val == 1) triggerEnableToggle(0); return 0; }
uint16_t cbSetCH2EnaToggle(TRegister* reg, uint16_t val) { feedWatchdog(); if (val == 1) triggerEnableToggle(1); return 0; }
uint16_t cbSetCH3EnaToggle(TRegister* reg, uint16_t val) { feedWatchdog(); if (val == 1) triggerEnableToggle(2); return 0; }

// ===========================================================================
// Modbus onGet callbacks (status registers -- live values)
// ===========================================================================
// Any read of SYS_STATE counts as a keep-alive: the GUI polls this register
// every cycle, so it naturally feeds the SW watchdog via reads (FC03) as well
// as via writes, matching the README intent ("any valid Modbus frame").
uint16_t cbGetSysState    (TRegister* reg, uint16_t val) { feedWatchdog(); return (uint16_t)g_state; }
uint16_t cbGetSysReason   (TRegister* reg, uint16_t val) { return (uint16_t)g_state; }
uint16_t cbGetFaultLatched(TRegister* reg, uint16_t val) { return g_faultLatched ? 1 : 0; }
uint16_t cbGetInterlockOk (TRegister* reg, uint16_t val) { return interlockOk()  ? 1 : 0; }
uint16_t cbGetWatchdogOk  (TRegister* reg, uint16_t val) { return watchdogOk()   ? 1 : 0; }
uint16_t cbGetLastError   (TRegister* reg, uint16_t val) {
    uint16_t e = g_lastError; g_lastError = 0; mb.Hreg(Reg::LAST_ERROR, 0); return e;
}

static uint16_t liveChField(uint8_t idx, uint8_t field) {
    uint32_t pulseMs = 0;
    uint32_t count = 0;
    uint32_t pulsesLeft = 0;
    Mode mode = Mode::Off;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        mode = g_ch[idx].mode;
        pulseMs = g_ch[idx].pulseMs;
        count = g_ch[idx].count;
        pulsesLeft = g_ch[idx].pulsesLeft;
    }
    switch (field) {
        case 0: return (uint16_t)mode;
        case 1: return (uint16_t)pulseMs;
        case 2: return (uint16_t)count;
        case 3: return (uint16_t)pulsesLeft;
        case 4: case 5: case 6: case 7: return 0;  // not wired in HW rev 2026-03-17
        case 8: return (digitalRead(Pins::GATE[idx]) == HIGH) ? 1 : 0;
        default: return 0;
    }
}

#define MAKE_CH_STAT_CALLBACKS(IDX)                                                \
uint16_t cbGetCH##IDX##S0(TRegister* r, uint16_t v){ return liveChField(IDX-1,0);}\
uint16_t cbGetCH##IDX##S1(TRegister* r, uint16_t v){ return liveChField(IDX-1,1);}\
uint16_t cbGetCH##IDX##S2(TRegister* r, uint16_t v){ return liveChField(IDX-1,2);}\
uint16_t cbGetCH##IDX##S3(TRegister* r, uint16_t v){ return liveChField(IDX-1,3);}\
uint16_t cbGetCH##IDX##S4(TRegister* r, uint16_t v){ return liveChField(IDX-1,4);}\
uint16_t cbGetCH##IDX##S5(TRegister* r, uint16_t v){ return liveChField(IDX-1,5);}\
uint16_t cbGetCH##IDX##S6(TRegister* r, uint16_t v){ return liveChField(IDX-1,6);}\
uint16_t cbGetCH##IDX##S7(TRegister* r, uint16_t v){ return liveChField(IDX-1,7);}\
uint16_t cbGetCH##IDX##S8(TRegister* r, uint16_t v){ return liveChField(IDX-1,8);}

MAKE_CH_STAT_CALLBACKS(1)
MAKE_CH_STAT_CALLBACKS(2)
MAKE_CH_STAT_CALLBACKS(3)

// ===========================================================================
// Modbus register setup
// ===========================================================================
static void modbusSetupRegisters() {
    // System control
    mb.addHreg(Reg::WATCHDOG_MS,  (uint16_t)Cfg::WD_DEFAULT_MS);
    mb.addHreg(Reg::TELEMETRY_MS, (uint16_t)Cfg::WD_DEFAULT_MS);
    mb.addHreg(Reg::COMMAND, 0);
    mb.onSetHreg(Reg::WATCHDOG_MS,  cbSetWatchdog);
    mb.onSetHreg(Reg::TELEMETRY_MS, cbSetTelemetry);
    mb.onSetHreg(Reg::COMMAND,      cbSetCommand);

    // Channel control
    for (uint8_t ch = 1; ch <= 3; ch++) {
        mb.addHreg(Reg::CH_MODE(ch),       0);
        mb.addHreg(Reg::CH_PULSE_MS(ch),   10);
        mb.addHreg(Reg::CH_COUNT(ch),      1);
        mb.addHreg(Reg::CH_ENA_TOGGLE(ch), 0);
    }
    mb.onSetHreg(Reg::CH_MODE(1),       cbSetCH1Mode);
    mb.onSetHreg(Reg::CH_MODE(2),       cbSetCH2Mode);
    mb.onSetHreg(Reg::CH_MODE(3),       cbSetCH3Mode);
    mb.onSetHreg(Reg::CH_PULSE_MS(1),   cbSetCH1PulseMs);
    mb.onSetHreg(Reg::CH_PULSE_MS(2),   cbSetCH2PulseMs);
    mb.onSetHreg(Reg::CH_PULSE_MS(3),   cbSetCH3PulseMs);
    mb.onSetHreg(Reg::CH_COUNT(1),      cbSetCH1Count);
    mb.onSetHreg(Reg::CH_COUNT(2),      cbSetCH2Count);
    mb.onSetHreg(Reg::CH_COUNT(3),      cbSetCH3Count);
    mb.onSetHreg(Reg::CH_ENA_TOGGLE(1), cbSetCH1EnaToggle);
    mb.onSetHreg(Reg::CH_ENA_TOGGLE(2), cbSetCH2EnaToggle);
    mb.onSetHreg(Reg::CH_ENA_TOGGLE(3), cbSetCH3EnaToggle);

    // System status
    mb.addHreg(Reg::SYS_STATE,     0);
    mb.addHreg(Reg::SYS_REASON,    0);
    mb.addHreg(Reg::FAULT_LATCHED, 0);
    mb.addHreg(Reg::INTERLOCK_OK,  0);
    mb.addHreg(Reg::WATCHDOG_OK,   0);
    mb.addHreg(Reg::LAST_ERROR,    0);
    mb.onGetHreg(Reg::SYS_STATE,     cbGetSysState);
    mb.onGetHreg(Reg::SYS_REASON,    cbGetSysReason);
    mb.onGetHreg(Reg::FAULT_LATCHED, cbGetFaultLatched);
    mb.onGetHreg(Reg::INTERLOCK_OK,  cbGetInterlockOk);
    mb.onGetHreg(Reg::WATCHDOG_OK,   cbGetWatchdogOk);
    mb.onGetHreg(Reg::LAST_ERROR,    cbGetLastError);

    // Per-channel status
    typedef uint16_t (*CbFn)(TRegister*, uint16_t);
    static const CbFn ch1cbs[9] = {
        cbGetCH1S0, cbGetCH1S1, cbGetCH1S2, cbGetCH1S3,
        cbGetCH1S4, cbGetCH1S5, cbGetCH1S6, cbGetCH1S7, cbGetCH1S8 };
    static const CbFn ch2cbs[9] = {
        cbGetCH2S0, cbGetCH2S1, cbGetCH2S2, cbGetCH2S3,
        cbGetCH2S4, cbGetCH2S5, cbGetCH2S6, cbGetCH2S7, cbGetCH2S8 };
    static const CbFn ch3cbs[9] = {
        cbGetCH3S0, cbGetCH3S1, cbGetCH3S2, cbGetCH3S3,
        cbGetCH3S4, cbGetCH3S5, cbGetCH3S6, cbGetCH3S7, cbGetCH3S8 };

    for (uint8_t f = 0; f < 9; f++) {
        mb.addHreg(Reg::CH_STAT_BASE(1) + f, 0);
        mb.addHreg(Reg::CH_STAT_BASE(2) + f, 0);
        mb.addHreg(Reg::CH_STAT_BASE(3) + f, 0);
        mb.onGetHreg(Reg::CH_STAT_BASE(1) + f, ch1cbs[f]);
        mb.onGetHreg(Reg::CH_STAT_BASE(2) + f, ch2cbs[f]);
        mb.onGetHreg(Reg::CH_STAT_BASE(3) + f, ch3cbs[f]);
    }
}

// ===========================================================================
// LCD helpers
// ===========================================================================
#if BCON_ENABLE_LCD

static const char* modeAbbrev(Mode m) {
    switch (m) {
        case Mode::Off:        return "OFF";
        case Mode::DC:         return "DC ";
        case Mode::Pulse:      return "PUL";
        case Mode::PulseTrain: return "TRN";
    }
    return "???";
}

static bool lcdProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

static void lcdInit() {
    uint8_t addr = 0;
    if      (lcdProbe(Cfg::LCD_ADDR_PRIMARY))  addr = Cfg::LCD_ADDR_PRIMARY;
    else if (lcdProbe(Cfg::LCD_ADDR_FALLBACK)) addr = Cfg::LCD_ADDR_FALLBACK;
    else return;

    g_lcd = new LiquidCrystal_I2C(addr, Cfg::LCD_COLS, Cfg::LCD_ROWS);
    g_lcd->begin(Cfg::LCD_COLS, Cfg::LCD_ROWS);
    g_lcd->backlight();
    g_lcd->clear();

    memset(g_lcdBuf,  ' ', sizeof(g_lcdBuf));
    memset(g_lcdLast, ' ', sizeof(g_lcdLast));
    for (uint8_t r = 0; r < Cfg::LCD_ROWS; r++) {
        g_lcdBuf[r][Cfg::LCD_COLS]  = '\0';
        g_lcdLast[r][Cfg::LCD_COLS] = '\0';
    }
}

static void lcdUpdate() {
    if (!g_lcd) return;
    uint32_t now = millis();
    // Defer if a serial byte arrived recently (I2C / RS-485 noise guard)
    if ((now - g_lastSerialByteMs) < Cfg::LCD_SERIAL_DEFER_MS) return;
    if ((now - g_lcdLastRefreshMs) < Cfg::LCD_REFRESH_MS)      return;
    g_lcdLastRefreshMs = now;

    // Build row 0: system status
    snprintf(g_lcdBuf[0], Cfg::LCD_COLS + 1,
             "WDG:%-2s INT:%-2s FLT:%d  ",
             watchdogOk()  ? "OK" : "NO",
             interlockOk() ? "OK" : "NO",
             (int)g_faultLatched);

    // Rows 1-3: per channel
    for (uint8_t i = 0; i < 3; i++) {
        Mode mode = Mode::Off;
        uint32_t pulsesLeft = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            mode = g_ch[i].mode;
            pulsesLeft = g_ch[i].pulsesLeft;
        }
        snprintf(g_lcdBuf[i + 1], Cfg::LCD_COLS + 1,
                 "CH%d %s O:%d R:%-5u   ",
                 i + 1,
                 modeAbbrev(mode),
                 (digitalRead(Pins::GATE[i]) == HIGH) ? 1 : 0,
                 (unsigned int)(pulsesLeft & 0xFFFFu));
    }

    // Write only changed rows; keep TWI enabled between refreshes.
    for (uint8_t r = 0; r < Cfg::LCD_ROWS; r++) {
        if (memcmp(g_lcdBuf[r], g_lcdLast[r], Cfg::LCD_COLS) != 0) {
            g_lcd->setCursor(0, r);
            g_lcd->print(g_lcdBuf[r]);
            memcpy(g_lcdLast[r], g_lcdBuf[r], Cfg::LCD_COLS + 1);
        }
    }
}
#endif // BCON_ENABLE_LCD

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
    g_bootMs = millis();

    // GPIO
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(Pins::GATE[i], OUTPUT); digitalWrite(Pins::GATE[i], LOW);
        pinMode(Pins::LED[i],  OUTPUT); digitalWrite(Pins::LED[i],  LOW);
        pinMode(Pins::ENA[i],  OUTPUT); digitalWrite(Pins::ENA[i],  LOW);
    }
    pinMode(Pins::INTERLOCK, INPUT);  // external circuit provides signal; no pull-up

    // Modbus
#if BCON_USE_USB_SERIAL
    Serial.begin(Cfg::BAUD, SERIAL_8N1);
    mb.begin(&Serial);
#else
    Serial1.begin(Cfg::BAUD, SERIAL_8N1);
    mb.begin(&Serial1, Pins::RS485_DE_RE);
    mb.setBaudrate(Cfg::BAUD);
#endif
    mb.slave(Cfg::SLAVE_ID);
    modbusSetupRegisters();

    // LCD
#if BCON_ENABLE_LCD
    Wire.begin();
    Wire.setClock(400000);
    lcdInit();
#endif

    pulseTimerSetup();

    // SW watchdog first feed
    feedWatchdog();

    // Hardware WDT last (8 s)
    wdt_enable(WDTO_8S);
}

// ===========================================================================
// loop()
// ===========================================================================
void loop() {
    wdt_reset();

    // Track last serial byte time for LCD deferral
#if BCON_USE_USB_SERIAL
    if (Serial.available()) g_lastSerialByteMs = millis();
#else
    if (Serial1.available()) g_lastSerialByteMs = millis();
#endif

    mb.task();
    syncRuntimeModeToRegisters();

    g_state = evaluateState();
    g_pulseOutputsEnabled = (g_state == TopState::Ready);

    if (g_state != TopState::Ready) {
        forceAllOutputsLow();
    } else {
        applyDC();
    }

#if BCON_ENABLE_LCD
    lcdUpdate();
#endif
}
