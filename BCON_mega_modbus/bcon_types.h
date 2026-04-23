#pragma once
// bcon_types.h
// Shared enums and structs for BCON_mega_modbus.ino
// Must be included AFTER Arduino.h (provides uint8_t, uint32_t, bool).

// System-level safety state
enum class TopState : uint8_t {
    Ready          = 0,
    SafeInterlock  = 1,
    SafeWatchdog   = 2,
};

// Channel output mode
enum class Mode : uint8_t {
    Off        = 0,
    DC         = 1,
    Pulse      = 2,
    PulseTrain = 3,
};

// Per-channel runtime state
struct Channel {
    volatile Mode mode             = Mode::Off;
    Mode          requestedMode    = Mode::Off;
    uint32_t      pulseMs          = 10;
    uint32_t      count            = 1;

    volatile uint32_t pulsesLeft       = 0;
    volatile uint32_t phaseDurationTicks  = 0;
    volatile uint32_t phaseRemainingTicks = 0;
    volatile bool     inHighPhase      = false;
    bool              modeApplyPending = false;
    bool              enabled          = false;

    uint32_t enaToggleStartUs = 0;
    bool     enaToggleActive  = false;
};


// Supervisor-visible semantic channel state
enum class ChannelRunState : uint8_t {
    Off = 0,
    Staged = 1,
    RunningDC = 2,
    RunningPulse = 3,
    RunningTrain = 4,
    Complete = 5,
    Aborted = 6,
};

// Supervisor stop / abort reasons
enum class StopReason : uint8_t {
    None = 0,
    NormalComplete = 1,
    AllOffCommand = 2,
    SafeInterlock = 3,
    SafeWatchdog = 4,
};

// Overall supervisor summary state
enum class SupervisorState : uint8_t {
    Idle = 0,
    Active = 1,
    CommandQueued = 2,
    SafeInterlockHold = 3,
    SafeWatchdogHold = 4,
};

// Last-command result reporting
enum class CommandResultCode : uint8_t {
    None = 0,
    Queued = 1,
    Executed = 2,
    Rejected = 3,
};

// Last-command reject reporting
enum class RejectReason : uint8_t {
    None = 0,
    InvalidCommand = 1,
    QueueFull = 2,
    UnsafeInterlock = 3,
    UnsafeWatchdog = 4,
};

// Normalized supervisor command types derived from Modbus COMMAND writes
enum class SupervisorCommandType : uint8_t {
    None = 0,
    AllOff = 1,
    Apply = 4,
};

// Ordered command-queue element
struct QueuedCommand {
    uint16_t seq = 0;
    uint16_t rawCode = 0;
    SupervisorCommandType type = SupervisorCommandType::None;
};

// Per-channel semantic state owned by the supervisor
struct SupervisorChannelState {
    ChannelRunState runState = ChannelRunState::Off;
    StopReason stopReason = StopReason::None;
    bool completionLatched = false;
    bool abortLatched = false;
};
