#pragma once
// bcon_types.h
// Shared enums and structs for BCON_mega_modbus.ino
// Must be included AFTER Arduino.h (provides uint8_t, uint32_t, bool).

// System-level safety state
enum class TopState : uint8_t {
    Ready          = 0,
    SafeInterlock  = 1,
    SafeWatchdog   = 2,
    FaultLatched   = 3,
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

    uint32_t enaToggleStartUs = 0;
    bool     enaToggleActive  = false;
};
