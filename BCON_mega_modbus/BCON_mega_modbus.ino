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
//    2  COMMAND          0=NOP, 1=AllOff, 4=ApplyStagedModes (2/3 reserved)
//
//  Channel control  (base = ch*10, ch=1..3):
//    base+0  MODE           staged requested mode (including Off); write COMMAND=4 to apply
//    base+1  PULSE_MS       pulse duration ms (1-60000)
//    base+2  COUNT          pulse count (1-10000)
//    base+3  ENABLE_STATE   explicit enable request (0=disabled, 1=enabled)
//
//  System status (read-only):
//    100  SYS_STATE      0=Ready 1=SafeInterlock 2=SafeWatchdog
//    101  SYS_REASON     mirrors SYS_STATE
//    102  FAULT_LATCHED  reserved compatibility register (always 0)
//    103  INTERLOCK_OK   1 if arm-beams interlock asserted
//    104  WATCHDOG_OK    1 if SW watchdog alive
//    105  LAST_ERROR     last error code (auto-cleared on read)
//    106  SUP_STATE      supervisor summary state
//    107  CMD_QUEUE_DEPTH pending supervisor-command count (0 or 1)
//    108  LAST_CMD_CODE  most recent supervisor command code
//    109  LAST_CMD_RESULT 0=None 1=Queued 2=Executed 3=Rejected
//
//  Per-channel status  (base = 110 + (ch-1)*10, offsets 0-8):
//    +0  mode           active output mode
//    +1  pulse_ms       configured pulse duration
//    +2  count          configured pulse count
//    +3  remaining      pulses remaining in current train
//    +4  enable_status  firmware-latched channel enable state (0/1)
//    +5  power_status   (not wired - always 0)
//    +6  overcurrent    (not wired - always 0)
//    +7  gated          (not wired - always 0)
//    +8  output_level   1 if gate pin currently HIGH
//
//  Supervisor extension status (read-only):
//    140+(ch-1)*4 +0  CH_SUP_STATE    per-channel semantic state
//    140+(ch-1)*4 +1  CH_STOP_REASON  most recent stop/abort reason
//    140+(ch-1)*4 +2  CH_COMPLETE     1 if completion latched
//    140+(ch-1)*4 +3  CH_ABORTED      1 if abort latched
//    152  LAST_REJECT_REASON most recent command reject reason
//    153  LAST_CMD_SEQ      most recent accepted command sequence
// =============================================================================

#include <Arduino.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include "bcon_types.h"

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
#define BCON_USE_USB_SERIAL  0   // 0 = RS-485 (production), 1 = USB-CDC (bench)
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
    constexpr uint8_t GATE_PORT_MASK[3] = { _BV(PK3), _BV(PK5), _BV(PK1) };
    constexpr uint8_t GATE_ALL_PORT_MASK =
        (uint8_t)(GATE_PORT_MASK[0] | GATE_PORT_MASK[1] | GATE_PORT_MASK[2]);

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

    constexpr uint32_t WD_DEFAULT_MS          = 1500;
    constexpr uint32_t WD_MIN_MS              = 50;
    constexpr uint32_t WD_MAX_MS              = 60000;
    constexpr uint32_t WD_BOOT_GRACE_MS       = 8000;

    constexpr uint32_t PULSE_MS_MIN           = 1;
    constexpr uint32_t PULSE_MS_MAX           = 60000;
    constexpr uint32_t COUNT_MIN              = 1;
    constexpr uint32_t COUNT_MAX              = 10000;

    constexpr uint32_t ENA_TOGGLE_MS          = 100;
    constexpr uint32_t ENA_TOGGLE_US          = ENA_TOGGLE_MS * 1000UL;

    constexpr uint8_t  TIMER_CS_64_BITS       = 0x03;
    constexpr uint8_t  TIMER_SYNC_HOLD_BITS   = (uint8_t)(_BV(TSM) | _BV(PSRSYNC));
    constexpr uint32_t TIMER_TICKS_PER_MS     = 250UL;
    constexpr uint32_t TIMER_MAX_CHUNK_TICKS  = 65535UL;

    constexpr uint32_t LCD_REFRESH_MS         = 1000;
    constexpr uint32_t LCD_SERIAL_DEFER_MS    = 50;
    // Single in-flight command keeps host behavior deterministic while still
    // routing execution through the supervisor/result-reporting path.
    constexpr uint8_t  COMMAND_QUEUE_LEN      = 1;

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
    inline constexpr uint16_t CH_ENABLE_STATE(uint8_t ch) { return CH_BASE(ch) + 3; }

    // System status
    constexpr uint16_t SYS_STATE     = 100;
    constexpr uint16_t SYS_REASON    = 101;
    constexpr uint16_t FAULT_LATCHED = 102;
    constexpr uint16_t INTERLOCK_OK  = 103;
    constexpr uint16_t WATCHDOG_OK   = 104;
    constexpr uint16_t LAST_ERROR    = 105;
    constexpr uint16_t SUP_STATE     = 106;
    constexpr uint16_t CMD_QUEUE_DEPTH = 107;
    constexpr uint16_t LAST_CMD_CODE = 108;
    constexpr uint16_t LAST_CMD_RESULT = 109;

    // Per-channel status (ch = 1..3)
    inline constexpr uint16_t CH_STAT_BASE(uint8_t ch) { return (uint16_t)(110 + (ch - 1) * 10); }
    inline constexpr uint16_t CH_SUP_BASE(uint8_t ch)  { return (uint16_t)(140 + (ch - 1) * 4); }

    constexpr uint16_t LAST_REJECT_REASON = 152;
    constexpr uint16_t LAST_CMD_SEQ       = 153;
}

// ===========================================================================
// Supervisory state bookkeeping (first-pass refactor)
// Preserves the existing pulse engine / timer path while moving command
// acceptance and high-level control decisions toward a mailbox + supervisor loop.
// ===========================================================================
struct CommandMailbox {
    QueuedCommand queue[Cfg::COMMAND_QUEUE_LEN];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
    uint16_t nextSeq = 1;
};

// Explicit forward declarations prevent Arduino's auto-prototype pass from
// emitting prototypes before these enum/struct types are defined.
static StopReason stopReasonForTopState(TopState state);
static RejectReason rejectReasonForTopState(TopState state);
static SupervisorCommandType normalizeSupervisorCommand(uint16_t rawCode);
static void recordCommandStatus(uint16_t rawCode,
                                uint16_t seq,
                                CommandResultCode result,
                                RejectReason rejectReason = RejectReason::None);
static bool supervisorAnyChannelActive();
static SupervisorState currentSupervisorState();
static bool enqueueSupervisorCommand(uint16_t rawCode);
static bool dequeueSupervisorCommand(QueuedCommand& out);
static void supervisorMarkChannelStaged(uint8_t idx);
static void supervisorMarkChannelComplete(uint8_t idx);
static void supervisorMarkChannelAborted(uint8_t idx, StopReason reason);
static void supervisorRefreshChannelState(uint8_t idx);
static void supervisorRefreshAllChannelStates();
static bool applyPendingModes();
static void stopChannel(uint8_t idx, bool clearRequestedMode = true);
static uint16_t supervisorChField(uint8_t idx, uint8_t field);
static void setChannelEnabledState(uint8_t idx, bool enabled);

// ===========================================================================
// Global state
// ===========================================================================
static Channel  g_ch[3];
static TopState g_state        = TopState::SafeWatchdog;
static uint32_t g_wdTimeoutMs  = Cfg::WD_DEFAULT_MS;
static uint32_t g_wdLastFeedMs = 0;
static uint32_t g_bootMs       = 0;
static uint16_t g_lastError    = 0;
static uint32_t g_telemMs      = Cfg::WD_DEFAULT_MS;
static uint32_t g_lastSerialByteMs  = 0;
static volatile uint8_t g_modeRegisterClearMask = 0;
static bool     g_commandRegisterClearPending = false;
static CommandMailbox g_cmdMailbox;
static SupervisorChannelState g_supCh[3];
static uint16_t g_lastCommandSeq = 0;
static uint16_t g_lastCommandCode = 0;
static CommandResultCode g_lastCommandResult = CommandResultCode::None;
static RejectReason g_lastRejectReason = RejectReason::None;
static bool     g_lastInterlockSample = false;
static bool     g_lastWatchdogSample  = true;
static bool     g_lcdRefreshPending   = false;

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
static inline uint8_t channelMaskBit(uint8_t idx) {
    return (uint8_t)(1u << idx);
}

static uint8_t channelMaskToGatePortMask(uint8_t channelMask) {
    uint8_t portMask = 0;
    for (uint8_t idx = 0; idx < 3; idx++) {
        if (channelMask & channelMaskBit(idx)) portMask |= Pins::GATE_PORT_MASK[idx];
    }
    return portMask;
}

static void setLedStateUnsafe(uint8_t idx, bool high) {
    switch (idx) {
        case 0:
            if (high) PORTE |= _BV(PE5);
            else      PORTE &= (uint8_t)~_BV(PE5);
            break;
        case 1:
            if (high) PORTE |= _BV(PE3);
            else      PORTE &= (uint8_t)~_BV(PE3);
            break;
        default:
            if (high) PORTH |= _BV(PH4);
            else      PORTH &= (uint8_t)~_BV(PH4);
            break;
    }
}

static void writeGateChannelsUnsafe(uint8_t highChannelMask, uint8_t lowChannelMask) {
    const uint8_t changeChannelMask = (uint8_t)(highChannelMask | lowChannelMask);
    const uint8_t changePortMask = channelMaskToGatePortMask(changeChannelMask);
    const uint8_t highPortMask = channelMaskToGatePortMask(highChannelMask);

    PORTK = (uint8_t)((PORTK & (uint8_t)~changePortMask) | highPortMask);

    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((changeChannelMask & channelMaskBit(idx)) == 0) continue;
        setLedStateUnsafe(idx, (highChannelMask & channelMaskBit(idx)) != 0);
    }
}

static void setGateStateUnsafe(uint8_t idx, bool high) {
    if (high) writeGateChannelsUnsafe(channelMaskBit(idx), 0);
    else      writeGateChannelsUnsafe(0, channelMaskBit(idx));
}

static void setGateState(uint8_t idx, bool high) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setGateStateUnsafe(idx, high);
    }
}

static void forceAllOutputsLow() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        writeGateChannelsUnsafe(0, channelMaskBit(0) | channelMaskBit(1) | channelMaskBit(2));
    }
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

static uint32_t pulseDurationTicks(uint32_t pulseMs) {
    return pulseMs * Cfg::TIMER_TICKS_PER_MS;
}

static TopState evaluateState(bool interlockIsOk, bool watchdogIsOk) {
    if (!interlockIsOk)  return TopState::SafeInterlock;
    if (!watchdogIsOk)   return TopState::SafeWatchdog;
    return TopState::Ready;
}

static TopState evaluateState() {
    return evaluateState(interlockOk(), watchdogOk());
}

static StopReason stopReasonForTopState(TopState state) {
    switch (state) {
        case TopState::SafeInterlock: return StopReason::SafeInterlock;
        case TopState::SafeWatchdog:  return StopReason::SafeWatchdog;
        default:                      return StopReason::None;
    }
}

static RejectReason rejectReasonForTopState(TopState state) {
    switch (state) {
        case TopState::SafeInterlock: return RejectReason::UnsafeInterlock;
        case TopState::SafeWatchdog:  return RejectReason::UnsafeWatchdog;
        default:                      return RejectReason::None;
    }
}

static SupervisorCommandType normalizeSupervisorCommand(uint16_t rawCode) {
    switch (rawCode) {
        case 1: return SupervisorCommandType::AllOff;
        case 4: return SupervisorCommandType::Apply;
        default: return SupervisorCommandType::None;
    }
}

static void recordCommandStatus(uint16_t rawCode,
                                uint16_t seq,
                                CommandResultCode result,
                                RejectReason rejectReason) {
    g_lastCommandCode = rawCode;
    g_lastCommandSeq = seq;
    g_lastCommandResult = result;
    g_lastRejectReason = rejectReason;
}

static bool supervisorAnyChannelActive() {
    for (uint8_t idx = 0; idx < 3; idx++) {
        const ChannelRunState runState = g_supCh[idx].runState;
        if (runState == ChannelRunState::Staged ||
            runState == ChannelRunState::RunningDC ||
            runState == ChannelRunState::RunningPulse ||
            runState == ChannelRunState::RunningTrain) {
            return true;
        }
    }
    return false;
}

static SupervisorState currentSupervisorState() {
    switch (g_state) {
        case TopState::SafeInterlock: return SupervisorState::SafeInterlockHold;
        case TopState::SafeWatchdog:  return SupervisorState::SafeWatchdogHold;
        case TopState::Ready:
        default:
            break;
    }

    if (g_cmdMailbox.count != 0) return SupervisorState::CommandQueued;
    if (supervisorAnyChannelActive()) return SupervisorState::Active;
    return SupervisorState::Idle;
}

static bool enqueueSupervisorCommand(uint16_t rawCode) {
    if (rawCode == 0) return true;

    const SupervisorCommandType type = normalizeSupervisorCommand(rawCode);
    if (type == SupervisorCommandType::None) {
        recordCommandStatus(rawCode, 0, CommandResultCode::Rejected, RejectReason::InvalidCommand);
        g_commandRegisterClearPending = true;
        return false;
    }

    if (g_cmdMailbox.count >= Cfg::COMMAND_QUEUE_LEN) {
        recordCommandStatus(rawCode, 0, CommandResultCode::Rejected, RejectReason::QueueFull);
        g_commandRegisterClearPending = true;
        return false;
    }

    QueuedCommand& slot = g_cmdMailbox.queue[g_cmdMailbox.tail];
    slot.seq = g_cmdMailbox.nextSeq++;
    slot.rawCode = rawCode;
    slot.type = type;

    g_cmdMailbox.tail = (uint8_t)((g_cmdMailbox.tail + 1u) % Cfg::COMMAND_QUEUE_LEN);
    g_cmdMailbox.count++;

    recordCommandStatus(rawCode, slot.seq, CommandResultCode::Queued);
    return true;
}

static bool dequeueSupervisorCommand(QueuedCommand& out) {
    if (g_cmdMailbox.count == 0) return false;

    out = g_cmdMailbox.queue[g_cmdMailbox.head];
    g_cmdMailbox.head = (uint8_t)((g_cmdMailbox.head + 1u) % Cfg::COMMAND_QUEUE_LEN);
    g_cmdMailbox.count--;
    return true;
}

static void supervisorMarkChannelStaged(uint8_t idx) {
    g_supCh[idx].runState = (g_ch[idx].requestedMode == Mode::Off)
        ? ChannelRunState::Off
        : ChannelRunState::Staged;
    g_supCh[idx].completionLatched = false;
    g_supCh[idx].abortLatched = false;
    if (g_ch[idx].requestedMode == Mode::Off) {
        g_supCh[idx].stopReason = StopReason::None;
    }
}

static void supervisorMarkChannelComplete(uint8_t idx) {
    g_supCh[idx].runState = ChannelRunState::Complete;
    g_supCh[idx].stopReason = StopReason::NormalComplete;
    g_supCh[idx].completionLatched = true;
    g_supCh[idx].abortLatched = false;
}

static void supervisorMarkChannelAborted(uint8_t idx, StopReason reason) {
    g_supCh[idx].runState = ChannelRunState::Aborted;
    g_supCh[idx].stopReason = reason;
    g_supCh[idx].completionLatched = false;
    g_supCh[idx].abortLatched = true;
}

static void supervisorRefreshChannelState(uint8_t idx) {
    Mode activeMode = Mode::Off;
    bool applyPending = false;
    Mode requestedMode = Mode::Off;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        activeMode = g_ch[idx].mode;
        applyPending = g_ch[idx].modeApplyPending;
        requestedMode = g_ch[idx].requestedMode;
    }

    if (activeMode == Mode::DC) {
        g_supCh[idx].runState = ChannelRunState::RunningDC;
        g_supCh[idx].stopReason = StopReason::None;
        g_supCh[idx].completionLatched = false;
        g_supCh[idx].abortLatched = false;
        return;
    }
    if (activeMode == Mode::Pulse) {
        g_supCh[idx].runState = ChannelRunState::RunningPulse;
        g_supCh[idx].stopReason = StopReason::None;
        g_supCh[idx].completionLatched = false;
        g_supCh[idx].abortLatched = false;
        return;
    }
    if (activeMode == Mode::PulseTrain) {
        g_supCh[idx].runState = ChannelRunState::RunningTrain;
        g_supCh[idx].stopReason = StopReason::None;
        g_supCh[idx].completionLatched = false;
        g_supCh[idx].abortLatched = false;
        return;
    }
    if (applyPending && requestedMode != Mode::Off) {
        g_supCh[idx].runState = ChannelRunState::Staged;
        return;
    }
    if (g_supCh[idx].completionLatched || g_supCh[idx].abortLatched) {
        return;
    }
    g_supCh[idx].runState = ChannelRunState::Off;
    g_supCh[idx].stopReason = StopReason::None;
}

static void supervisorRefreshAllChannelStates() {
    for (uint8_t idx = 0; idx < 3; idx++) {
        supervisorRefreshChannelState(idx);
    }
}

static void supervisorStageCommand(uint16_t val) {
    enqueueSupervisorCommand(val);
}

static void supervisorProcessCommandMailbox() {
    QueuedCommand cmd;
    while (dequeueSupervisorCommand(cmd)) {
        bool executed = false;
        RejectReason rejectReason = RejectReason::None;

        switch (cmd.type) {
            case SupervisorCommandType::AllOff:
                for (uint8_t i = 0; i < 3; i++) {
                    stopChannel(i, true);
                    supervisorMarkChannelAborted(i, StopReason::AllOffCommand);
                }
                mb.cbDisable();
                for (uint8_t i = 0; i < 3; i++) mb.Hreg(Reg::CH_MODE(i + 1), 0);
                mb.cbEnable(true);
                executed = true;
                break;

            case SupervisorCommandType::Apply:
                executed = applyPendingModes();
                if (!executed) rejectReason = rejectReasonForTopState(evaluateState());
                break;

            case SupervisorCommandType::None:
            default:
                rejectReason = RejectReason::InvalidCommand;
                break;
        }

        recordCommandStatus(cmd.rawCode,
                            cmd.seq,
                            executed ? CommandResultCode::Executed : CommandResultCode::Rejected,
                            rejectReason);
        g_commandRegisterClearPending = true;
    }
}

static void timerHardwareInit() {
    TCCR3A = 0; TCCR3B = 0; TIMSK3 &= (uint8_t)~_BV(OCIE3A); TIFR3 = _BV(OCF3A); TCNT3 = 0; OCR3A = 0;
    TCCR4A = 0; TCCR4B = 0; TIMSK4 &= (uint8_t)~_BV(OCIE4A); TIFR4 = _BV(OCF4A); TCNT4 = 0; OCR4A = 0;
    TCCR5A = 0; TCCR5B = 0; TIMSK5 &= (uint8_t)~_BV(OCIE5A); TIFR5 = _BV(OCF5A); TCNT5 = 0; OCR5A = 0;
}

static void timerStopUnsafe(uint8_t idx) {
    switch (idx) {
        case 0:
            TCCR3B = 0;
            TIMSK3 &= (uint8_t)~_BV(OCIE3A);
            TIFR3 = _BV(OCF3A);
            break;
        case 1:
            TCCR4B = 0;
            TIMSK4 &= (uint8_t)~_BV(OCIE4A);
            TIFR4 = _BV(OCF4A);
            break;
        default:
            TCCR5B = 0;
            TIMSK5 &= (uint8_t)~_BV(OCIE5A);
            TIFR5 = _BV(OCF5A);
            break;
    }
}

static uint16_t timerReadCounterUnsafe(uint8_t idx) {
    switch (idx) {
        case 0: return TCNT3;
        case 1: return TCNT4;
        default: return TCNT5;
    }
}

static uint16_t timerReadCompareUnsafe(uint8_t idx) {
    switch (idx) {
        case 0: return OCR3A;
        case 1: return OCR4A;
        default: return OCR5A;
    }
}

static void timerWriteCounterUnsafe(uint8_t idx, uint16_t value) {
    switch (idx) {
        case 0: TCNT3 = value; break;
        case 1: TCNT4 = value; break;
        default: TCNT5 = value; break;
    }
}

static void timerClearCompareFlagUnsafe(uint8_t idx) {
    switch (idx) {
        case 0: TIFR3 = _BV(OCF3A); break;
        case 1: TIFR4 = _BV(OCF4A); break;
        default: TIFR5 = _BV(OCF5A); break;
    }
}

static void timerEnableCompareInterruptUnsafe(uint8_t idx) {
    switch (idx) {
        case 0: TIMSK3 |= _BV(OCIE3A); break;
        case 1: TIMSK4 |= _BV(OCIE4A); break;
        default: TIMSK5 |= _BV(OCIE5A); break;
    }
}

static void timerWriteCompareUnsafe(uint8_t idx, uint16_t value) {
    switch (idx) {
        case 0: OCR3A = value; break;
        case 1: OCR4A = value; break;
        default: OCR5A = value; break;
    }
}

static void timerScheduleNextChunkUnsafe(uint8_t idx, bool fromPreviousCompare) {
    Channel& c = g_ch[idx];
    uint32_t chunkTicks = c.phaseRemainingTicks;
    if (chunkTicks == 0) {
        timerStopUnsafe(idx);
        return;
    }
    if (chunkTicks > Cfg::TIMER_MAX_CHUNK_TICKS) chunkTicks = Cfg::TIMER_MAX_CHUNK_TICKS;
    c.phaseRemainingTicks -= chunkTicks;

    const uint16_t base = fromPreviousCompare ? timerReadCompareUnsafe(idx) : timerReadCounterUnsafe(idx);
    timerWriteCompareUnsafe(idx, (uint16_t)(base + (uint16_t)chunkTicks));
    timerClearCompareFlagUnsafe(idx);
    timerEnableCompareInterruptUnsafe(idx);
}

static void timerStartUnsafe(uint8_t idx) {
    switch (idx) {
        case 0: TCCR3B = Cfg::TIMER_CS_64_BITS; break;
        case 1: TCCR4B = Cfg::TIMER_CS_64_BITS; break;
        default: TCCR5B = Cfg::TIMER_CS_64_BITS; break;
    }
}

static void queueModeRegisterClearUnsafe(uint8_t idx) {
    g_modeRegisterClearMask |= channelMaskBit(idx);
}

static void resetChannelRuntimeUnsafe(uint8_t idx, bool clearRequestedMode) {
    Channel& c = g_ch[idx];
    timerStopUnsafe(idx);
    c.mode             = Mode::Off;
    c.pulsesLeft       = 0;
    c.phaseDurationTicks  = 0;
    c.phaseRemainingTicks = 0;
    c.inHighPhase      = false;
    if (clearRequestedMode) {
        c.requestedMode    = Mode::Off;
        c.modeApplyPending = false;
    }
}

static void stopChannelUnsafe(uint8_t idx, bool clearRequestedMode) {
    resetChannelRuntimeUnsafe(idx, clearRequestedMode);
    setGateStateUnsafe(idx, false);
}

static void stopChannel(uint8_t idx, bool clearRequestedMode) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stopChannelUnsafe(idx, clearRequestedMode);
    }
}

static void triggerEnableToggle(uint8_t idx) {
    g_ch[idx].enaToggleStartUs = micros();
    g_ch[idx].enaToggleActive  = true;
    digitalWrite(Pins::ENA[idx], HIGH);
}

static void setChannelEnabledState(uint8_t idx, bool enabled) {
    Channel& c = g_ch[idx];
    if (c.enabled == enabled) return;

    c.enabled = enabled;
    triggerEnableToggle(idx);
}

static void tickEnableToggles() {
    const uint32_t nowUs = micros();
    for (uint8_t idx = 0; idx < 3; idx++) {
        Channel& c = g_ch[idx];
        if (!c.enaToggleActive) continue;
        if ((nowUs - c.enaToggleStartUs) >= Cfg::ENA_TOGGLE_US) {
            digitalWrite(Pins::ENA[idx], LOW);
            c.enaToggleActive = false;
        }
    }
}

static uint8_t collectDueTimerMask(uint8_t sourceMask) {
    uint8_t dueMask = sourceMask;
    if ((TIMSK3 & _BV(OCIE3A)) && (TIFR3 & _BV(OCF3A))) dueMask |= channelMaskBit(0);
    if ((TIMSK4 & _BV(OCIE4A)) && (TIFR4 & _BV(OCF4A))) dueMask |= channelMaskBit(1);
    if ((TIMSK5 & _BV(OCIE5A)) && (TIFR5 & _BV(OCF5A))) dueMask |= channelMaskBit(2);
    return dueMask;
}

static void startTimersTogetherUnsafe(uint8_t startMask) {
    if (startMask == 0) return;

    GTCCR = Cfg::TIMER_SYNC_HOLD_BITS;
    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((startMask & channelMaskBit(idx)) == 0) continue;
        timerStopUnsafe(idx);
        timerWriteCounterUnsafe(idx, 0);
        timerScheduleNextChunkUnsafe(idx, false);
        timerStartUnsafe(idx);
    }
    GTCCR = 0;
}

static void armRunningTimersUnsafe(uint8_t timerMask) {
    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((timerMask & channelMaskBit(idx)) == 0) continue;
        timerScheduleNextChunkUnsafe(idx, true);
    }
}

static void handleTimerCompareBatch(uint8_t sourceMask) {
    const uint8_t dueMask = collectDueTimerMask(sourceMask);
    uint8_t gateHighMask = 0;
    uint8_t gateLowMask  = 0;
    uint8_t nextChunkMask = 0;

    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((dueMask & channelMaskBit(idx)) == 0) continue;

        timerClearCompareFlagUnsafe(idx);

        Channel& c = g_ch[idx];
        if (c.mode != Mode::Pulse && c.mode != Mode::PulseTrain) {
            gateLowMask |= channelMaskBit(idx);
            timerStopUnsafe(idx);
            continue;
        }

        if (c.phaseRemainingTicks > 0) {
            nextChunkMask |= channelMaskBit(idx);
            continue;
        }

        if (c.inHighPhase) {
            gateLowMask |= channelMaskBit(idx);
            if (c.pulsesLeft > 0) c.pulsesLeft--;
            if (c.pulsesLeft == 0) {
                c.mode             = Mode::Off;
                c.phaseDurationTicks  = 0;
                c.phaseRemainingTicks = 0;
                c.inHighPhase      = false;
                timerStopUnsafe(idx);
                queueModeRegisterClearUnsafe(idx);
                continue;
            }
            c.inHighPhase = false;
        } else {
            c.inHighPhase = true;
            gateHighMask |= channelMaskBit(idx);
        }

        c.phaseRemainingTicks = c.phaseDurationTicks;
        nextChunkMask |= channelMaskBit(idx);
    }

    if ((gateHighMask | gateLowMask) != 0) {
        writeGateChannelsUnsafe(gateHighMask, gateLowMask);
    }

    armRunningTimersUnsafe(nextChunkMask);
}

ISR(TIMER3_COMPA_vect) {
    handleTimerCompareBatch(channelMaskBit(0));
}

ISR(TIMER4_COMPA_vect) {
    handleTimerCompareBatch(channelMaskBit(1));
}

ISR(TIMER5_COMPA_vect) {
    handleTimerCompareBatch(channelMaskBit(2));
}

static bool applyPendingModes() {
    uint8_t applyMask = 0;
    bool hasNonOffRequest = false;
    for (uint8_t idx = 0; idx < 3; idx++) {
        if (!g_ch[idx].modeApplyPending) continue;
        applyMask |= channelMaskBit(idx);
        if (g_ch[idx].requestedMode != Mode::Off) hasNonOffRequest = true;
    }
    if (applyMask == 0) return true;

    if (hasNonOffRequest && evaluateState() != TopState::Ready) {
        g_lastError = 10;
        mb.Hreg(Reg::LAST_ERROR, g_lastError);
        return false;
    }

    uint8_t pulseMask = 0;
    uint8_t gateHighMask = 0;
    uint8_t gateLowMask = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            if ((applyMask & channelMaskBit(idx)) == 0) continue;
            gateLowMask |= channelMaskBit(idx);
            resetChannelRuntimeUnsafe(idx, false);
        }

        for (uint8_t idx = 0; idx < 3; idx++) {
            if ((applyMask & channelMaskBit(idx)) == 0) continue;

            Channel& c = g_ch[idx];
            c.modeApplyPending = false;

            if (c.requestedMode == Mode::DC) {
                c.mode = Mode::DC;
                gateHighMask |= channelMaskBit(idx);
                continue;
            }

            if (c.requestedMode != Mode::Pulse && c.requestedMode != Mode::PulseTrain) {
                c.mode = Mode::Off;
                continue;
            }

            c.mode             = c.requestedMode;
            c.pulsesLeft       = c.count;
            c.phaseDurationTicks  = pulseDurationTicks(c.pulseMs);
            c.phaseRemainingTicks = c.phaseDurationTicks;
            c.inHighPhase      = true;
            gateHighMask |= channelMaskBit(idx);
            pulseMask |= channelMaskBit(idx);
        }

        if ((gateHighMask | gateLowMask) != 0) {
            writeGateChannelsUnsafe(gateHighMask, gateLowMask);
        }

        startTimersTogetherUnsafe(pulseMask);
    }

    return true;
}

static void handleSafetyTrip() {
    uint8_t clearMask = 0;
    const StopReason reason = stopReasonForTopState(evaluateState());

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t idx = 0; idx < 3; idx++) {
            Channel& c = g_ch[idx];
            const bool clearRequestedMode =
                c.modeApplyPending || c.mode == Mode::DC || c.mode == Mode::Pulse || c.mode == Mode::PulseTrain;
            const bool wasActive =
                c.modeApplyPending || c.mode == Mode::DC || c.mode == Mode::Pulse || c.mode == Mode::PulseTrain;

            if (clearRequestedMode) {
                stopChannelUnsafe(idx, true);
                clearMask |= channelMaskBit(idx);
            } else {
                timerStopUnsafe(idx);
            }

            if (wasActive) {
                supervisorMarkChannelAborted(idx, reason);
            }
        }
    }

    if (clearMask != 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            g_modeRegisterClearMask |= clearMask;
        }
    }
}

static void syncDeferredRegisters() {
    if (g_commandRegisterClearPending) {
        mb.cbDisable();
        mb.Hreg(Reg::COMMAND, 0);
        mb.cbEnable(true);
        g_commandRegisterClearPending = false;
    }

    uint8_t clearMask = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        clearMask = g_modeRegisterClearMask;
        g_modeRegisterClearMask = 0;
    }

    if (clearMask == 0) return;

    mb.cbDisable();
    for (uint8_t idx = 0; idx < 3; idx++) {
        if ((clearMask & channelMaskBit(idx)) == 0) continue;
        g_ch[idx].requestedMode = Mode::Off;
        g_ch[idx].modeApplyPending = false;
        mb.Hreg(Reg::CH_MODE(idx + 1), 0);
        if (!g_supCh[idx].abortLatched) {
            supervisorMarkChannelComplete(idx);
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
            setGateState(i, true);
        }
    }
}

// ===========================================================================
// Mode write helper (used by onSet callbacks)
// ===========================================================================
static void handleModeWrite(uint8_t idx, uint16_t val) {
    Mode newMode = (Mode)(val & 0x03);

    if (newMode == Mode::Pulse && g_ch[idx].count > 1) newMode = Mode::PulseTrain;

    g_ch[idx].requestedMode = newMode;
    g_ch[idx].modeApplyPending = true;
    supervisorMarkChannelStaged(idx);
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
    supervisorStageCommand(val);
    // Drain immediately so staged host writes followed by COMMAND=4 keep the
    // legacy 'apply now' behavior expected by pulser_test_gui.py.
    supervisorProcessCommandMailbox();
    return val;
}

// Return g_ch[idx].requestedMode (may differ from val due to Pulse→PulseTrain elevation);
// the library stores the callback return value — this is the ONLY safe way to
// update the control register because mb.Hreg(CH_MODE, …) inside a callback
// would re-trigger this same callback (infinite recursion).
uint16_t cbSetCH1Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(0, val); return (uint16_t)g_ch[0].requestedMode; }
uint16_t cbSetCH2Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(1, val); return (uint16_t)g_ch[1].requestedMode; }
uint16_t cbSetCH3Mode(TRegister* reg, uint16_t val) { feedWatchdog(); handleModeWrite(2, val); return (uint16_t)g_ch[2].requestedMode; }

uint16_t cbSetCH1PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { g_ch[0].pulseMs = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[0].pulseMs;
}
uint16_t cbSetCH2PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { g_ch[1].pulseMs = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[1].pulseMs;
}
uint16_t cbSetCH3PulseMs(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::PULSE_MS_MIN && val <= Cfg::PULSE_MS_MAX) { g_ch[2].pulseMs = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[2].pulseMs;
}

uint16_t cbSetCH1Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { g_ch[0].count = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[0].count;
}
uint16_t cbSetCH2Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { g_ch[1].count = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[1].count;
}
uint16_t cbSetCH3Count(TRegister* reg, uint16_t val) {
    feedWatchdog();
    if (val >= Cfg::COUNT_MIN && val <= Cfg::COUNT_MAX) { g_ch[2].count = val; return val; }
    g_lastError = 3; mb.Hreg(Reg::LAST_ERROR, g_lastError); return (uint16_t)g_ch[2].count;
}

static uint16_t handleEnableStateWrite(uint8_t idx, uint16_t val) {
    feedWatchdog();
    if (val <= 1) {
        setChannelEnabledState(idx, val != 0);
        return g_ch[idx].enabled ? 1 : 0;
    }

    g_lastError = 3;
    mb.Hreg(Reg::LAST_ERROR, g_lastError);
    return g_ch[idx].enabled ? 1 : 0;
}

uint16_t cbSetCH1EnableState(TRegister* reg, uint16_t val) { return handleEnableStateWrite(0, val); }
uint16_t cbSetCH2EnableState(TRegister* reg, uint16_t val) { return handleEnableStateWrite(1, val); }
uint16_t cbSetCH3EnableState(TRegister* reg, uint16_t val) { return handleEnableStateWrite(2, val); }

// ===========================================================================
// Modbus onGet callbacks (status registers -- live values)
// ===========================================================================
// Any read of SYS_STATE counts as a keep-alive: the GUI polls this register
// every cycle, so it naturally feeds the SW watchdog via reads (FC03) as well
// as via writes, matching the README intent ("any valid Modbus frame").
uint16_t cbGetSysState    (TRegister* reg, uint16_t val) { feedWatchdog(); return (uint16_t)g_state; }
uint16_t cbGetSysReason   (TRegister* reg, uint16_t val) { return (uint16_t)g_state; }
uint16_t cbGetFaultLatched(TRegister* reg, uint16_t val) { return 0; }
uint16_t cbGetInterlockOk (TRegister* reg, uint16_t val) { return interlockOk()  ? 1 : 0; }
uint16_t cbGetWatchdogOk  (TRegister* reg, uint16_t val) { return watchdogOk()   ? 1 : 0; }
uint16_t cbGetSupervisorState(TRegister* reg, uint16_t val) { return (uint16_t)currentSupervisorState(); }
uint16_t cbGetCommandQueueDepth(TRegister* reg, uint16_t val) { return (uint16_t)g_cmdMailbox.count; }
uint16_t cbGetLastCommandCode(TRegister* reg, uint16_t val) { return g_lastCommandCode; }
uint16_t cbGetLastCommandResult(TRegister* reg, uint16_t val) { return (uint16_t)g_lastCommandResult; }
uint16_t cbGetLastError   (TRegister* reg, uint16_t val) {
    uint16_t e = g_lastError; g_lastError = 0; mb.Hreg(Reg::LAST_ERROR, 0); return e;
}
uint16_t cbGetLastRejectReason(TRegister* reg, uint16_t val) { return (uint16_t)g_lastRejectReason; }
uint16_t cbGetLastCommandSeq(TRegister* reg, uint16_t val) { return g_lastCommandSeq; }

static uint16_t liveChField(uint8_t idx, uint8_t field) {
    Mode activeMode = Mode::Off;
    uint32_t pulsesLeft = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        activeMode = g_ch[idx].mode;
        pulsesLeft = g_ch[idx].pulsesLeft;
    }

    switch (field) {
        case 0: return (uint16_t)activeMode;
        case 1: return (uint16_t)g_ch[idx].pulseMs;
        case 2: return (uint16_t)g_ch[idx].count;
        case 3: return (uint16_t)pulsesLeft;
        case 4: return g_ch[idx].enabled ? 1 : 0;
        case 5: case 6: case 7: return 0;  // not wired in HW rev 2026-03-17
        case 8: return (digitalRead(Pins::GATE[idx]) == HIGH) ? 1 : 0;
        default: return 0;
    }
}

static uint16_t supervisorChField(uint8_t idx, uint8_t field) {
    switch (field) {
        case 0: return (uint16_t)g_supCh[idx].runState;
        case 1: return (uint16_t)g_supCh[idx].stopReason;
        case 2: return g_supCh[idx].completionLatched ? 1 : 0;
        case 3: return g_supCh[idx].abortLatched ? 1 : 0;
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

#define MAKE_CH_SUP_CALLBACKS(IDX)                                                   \
uint16_t cbGetCH##IDX##SUP0(TRegister* r, uint16_t v){ return supervisorChField(IDX-1,0);}\
uint16_t cbGetCH##IDX##SUP1(TRegister* r, uint16_t v){ return supervisorChField(IDX-1,1);}\
uint16_t cbGetCH##IDX##SUP2(TRegister* r, uint16_t v){ return supervisorChField(IDX-1,2);}\
uint16_t cbGetCH##IDX##SUP3(TRegister* r, uint16_t v){ return supervisorChField(IDX-1,3);}

MAKE_CH_SUP_CALLBACKS(1)
MAKE_CH_SUP_CALLBACKS(2)
MAKE_CH_SUP_CALLBACKS(3)

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
        mb.addHreg(Reg::CH_ENABLE_STATE(ch), 0);
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
    mb.onSetHreg(Reg::CH_ENABLE_STATE(1), cbSetCH1EnableState);
    mb.onSetHreg(Reg::CH_ENABLE_STATE(2), cbSetCH2EnableState);
    mb.onSetHreg(Reg::CH_ENABLE_STATE(3), cbSetCH3EnableState);

    // System status
    mb.addHreg(Reg::SYS_STATE,     0);
    mb.addHreg(Reg::SYS_REASON,    0);
    mb.addHreg(Reg::FAULT_LATCHED, 0);
    mb.addHreg(Reg::INTERLOCK_OK,  0);
    mb.addHreg(Reg::WATCHDOG_OK,   0);
    mb.addHreg(Reg::LAST_ERROR,    0);
    mb.addHreg(Reg::SUP_STATE,     0);
    mb.addHreg(Reg::CMD_QUEUE_DEPTH, 0);
    mb.addHreg(Reg::LAST_CMD_CODE, 0);
    mb.addHreg(Reg::LAST_CMD_RESULT, 0);
    mb.onGetHreg(Reg::SYS_STATE,       cbGetSysState);
    mb.onGetHreg(Reg::SYS_REASON,      cbGetSysReason);
    mb.onGetHreg(Reg::FAULT_LATCHED,   cbGetFaultLatched);
    mb.onGetHreg(Reg::INTERLOCK_OK,    cbGetInterlockOk);
    mb.onGetHreg(Reg::WATCHDOG_OK,     cbGetWatchdogOk);
    mb.onGetHreg(Reg::LAST_ERROR,      cbGetLastError);
    mb.onGetHreg(Reg::SUP_STATE,       cbGetSupervisorState);
    mb.onGetHreg(Reg::CMD_QUEUE_DEPTH, cbGetCommandQueueDepth);
    mb.onGetHreg(Reg::LAST_CMD_CODE,   cbGetLastCommandCode);
    mb.onGetHreg(Reg::LAST_CMD_RESULT, cbGetLastCommandResult);

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

    // Supervisor extension status
    static const CbFn ch1sup[4] = { cbGetCH1SUP0, cbGetCH1SUP1, cbGetCH1SUP2, cbGetCH1SUP3 };
    static const CbFn ch2sup[4] = { cbGetCH2SUP0, cbGetCH2SUP1, cbGetCH2SUP2, cbGetCH2SUP3 };
    static const CbFn ch3sup[4] = { cbGetCH3SUP0, cbGetCH3SUP1, cbGetCH3SUP2, cbGetCH3SUP3 };

    for (uint8_t f = 0; f < 4; f++) {
        mb.addHreg(Reg::CH_SUP_BASE(1) + f, 0);
        mb.addHreg(Reg::CH_SUP_BASE(2) + f, 0);
        mb.addHreg(Reg::CH_SUP_BASE(3) + f, 0);
        mb.onGetHreg(Reg::CH_SUP_BASE(1) + f, ch1sup[f]);
        mb.onGetHreg(Reg::CH_SUP_BASE(2) + f, ch2sup[f]);
        mb.onGetHreg(Reg::CH_SUP_BASE(3) + f, ch3sup[f]);
    }

    mb.addHreg(Reg::LAST_REJECT_REASON, 0);
    mb.addHreg(Reg::LAST_CMD_SEQ, 0);
    mb.onGetHreg(Reg::LAST_REJECT_REASON, cbGetLastRejectReason);
    mb.onGetHreg(Reg::LAST_CMD_SEQ, cbGetLastCommandSeq);
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

static void lcdUpdate(bool interlockIsOk, bool watchdogIsOk, bool forceRefresh = false) {
    if (!g_lcd) return;
    uint32_t now = millis();
    // Defer if a serial byte arrived recently (I2C / RS-485 noise guard)
    if ((now - g_lastSerialByteMs) < Cfg::LCD_SERIAL_DEFER_MS) return;
    if (!forceRefresh && (now - g_lcdLastRefreshMs) < Cfg::LCD_REFRESH_MS) return;
    g_lcdLastRefreshMs = now;
    g_lcdRefreshPending = false;

    // Build row 0: system status
    snprintf(g_lcdBuf[0], Cfg::LCD_COLS + 1,
             "WDG:%-2s INT:%-2s      ",
             watchdogIsOk  ? "OK" : "NO",
             interlockIsOk ? "OK" : "NO");

    // Rows 1-3: per channel
    for (uint8_t i = 0; i < 3; i++) {
        Mode activeMode = Mode::Off;
        uint32_t pulsesLeft = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            activeMode = g_ch[i].mode;
            pulsesLeft = g_ch[i].pulsesLeft;
        }
        snprintf(g_lcdBuf[i + 1], Cfg::LCD_COLS + 1,
                 "CH %c %s O:%d R:%-5u  ",
                 (char)('A' + i),
                 modeAbbrev(activeMode),
                 (digitalRead(Pins::GATE[i]) == HIGH) ? 1 : 0,
                 (unsigned int)(pulsesLeft & 0xFFFFu));
    }

    // Write only changed rows; disable TWI between writes to reduce I2C noise
    for (uint8_t r = 0; r < Cfg::LCD_ROWS; r++) {
        if (memcmp(g_lcdBuf[r], g_lcdLast[r], Cfg::LCD_COLS) != 0) {
            TWCR = 0;           // disable TWI hardware
            Wire.begin();
            Wire.setClock(400000);
            g_lcd->setCursor(0, r);
            g_lcd->print(g_lcdBuf[r]);
            memcpy(g_lcdLast[r], g_lcdBuf[r], Cfg::LCD_COLS + 1);
            TWCR = 0;           // disable again until next write
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
    timerHardwareInit();

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

    for (uint8_t idx = 0; idx < 3; idx++) {
        g_supCh[idx] = SupervisorChannelState{};
    }

    // SW watchdog first feed
    feedWatchdog();
    g_lastInterlockSample = interlockOk();
    g_lastWatchdogSample = watchdogOk();
    supervisorRefreshAllChannelStates();

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
    supervisorProcessCommandMailbox();

    const bool interlockIsOk = interlockOk();
    const bool watchdogIsOk = watchdogOk();
    if ((interlockIsOk != g_lastInterlockSample) ||
        (watchdogIsOk != g_lastWatchdogSample)) {
        g_lcdRefreshPending = true;
    }

    g_lastInterlockSample = interlockIsOk;
    g_lastWatchdogSample = watchdogIsOk;

    const TopState nextState = evaluateState(interlockIsOk, watchdogIsOk);
    if (g_state == TopState::Ready && nextState != TopState::Ready) {
        handleSafetyTrip();
    }
    g_state = nextState;

    if (g_state != TopState::Ready) {
        forceAllOutputsLow();
    } else {
        applyDC();
    }

    tickEnableToggles();
    syncDeferredRegisters();
    supervisorRefreshAllChannelStates();

#if BCON_ENABLE_LCD
    lcdUpdate(interlockIsOk, watchdogIsOk, g_lcdRefreshPending);
#endif
}
