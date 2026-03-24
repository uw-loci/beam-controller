# Firmware Gap Analysis

Cross-reference between `test_interfaces/pulser_test_gui.py` expectations and
`BCON_mega_modbus/BCON_mega_modbus.ino` actual implementation.

---

## 🔴 Critical — causes runtime failures today

### 1. Unimplemented register gaps cause `read_block(0, 34)` to always fail

The GUI polls registers 0–33 in one bulk FC 0x03 request.  The firmware only
handles registers 0–2 (system control) and 10–13 / 20–23 / 30–33 (per-channel
control).  The following addresses have **no handler** in `readHoldingRegister`:

| Address range | Gap description |
|---|---|
| 3 – 9 | Between REG_COMMAND (2) and CH1 base (10) |
| 14 – 19 | Between CH1 enable-toggle (13) and CH2 base (20) |
| 24 – 29 | Between CH2 enable-toggle (23) and CH3 base (30) |

When `readHoldingRegister` returns `false` for any address in the range,
`processModbusFrame` immediately sends exception 0x02 and aborts — so **every
control-register poll block fails** and the GUI register tree never updates.

**Fix:** Return `0` (and `true`) for any unmapped read address instead of
returning `false`.  Add a default case in `readHoldingRegister`:

```cpp
// At the end of readHoldingRegister, before the final `return false`:
out = 0;
return true;   // return 0 for unmapped/reserved registers
```

---

## 🟠 Major — features expected by GUI but missing from firmware

### 2. `REG_TELEMETRY_MS` (register 1) is accepted but never acts on

The firmware stores `Runtime::telemetryPeriodMs` when written, and returns it
on read.  However, **no autonomous telemetry is ever pushed** — the firmware
only responds to incoming requests.

The GUI writes and reads this register and exposes a "Telemetry (ms)" field.
Currently the value is a no-op: setting it to any number has no observable
effect on firmware behaviour.

**Fix options (choose one):**
- [ ] Remove the register and the GUI field (simplest).
- [ ] Implement a periodic unsolicited Modbus broadcast / FC 0x04 push on the
      `telemetryPeriodMs` interval (requires host-side listener).
- [ ] Repurpose as a read-only status counter (total frames received, uptime, etc.)

---

### 3. No REMOTE_ARM / beam-arm mechanism

The GUI "Safety & Remote Arm" section has a `REMOTE_ARM` checkbox and an
"Arm Beam" button.  The handler `_toggle_remote_arm()` already logs:

> `"REMOTE_ARM register is not implemented in current firmware"`

There is no ARM register in the firmware.  `REG_COMMAND = 2 / 3` clears the
fault latch (not the same as arming).  Currently the "Arm Beam" button path is
also broken at the GUI level (`_arm_beam` only handles the already-armed branch
and `REG_COMMAND = 3` is unreachably buried inside `_set_telemetry`).

**Fix (firmware side):**
- [ ] Define `REG_REMOTE_ARM` (e.g. register 3) — write `1` to arm, `0` to disarm.
- [ ] When REMOTE_ARM = 0, treat the same as watchdog/interlock fault: block
      all channel activation.
- [ ] Include remote-arm state in system status (e.g. a bit in REG_SYS_STATE or
      a new REG_ARM_STATE register at 106).

---

### 4. No atomic SYNC command — channels cannot start simultaneously

The GUI **Synchronous Control** panel now sends per-channel commands in two
phasesto minimise jitter:
  - **Phase 1**: all `duration_ms` + `count` writes for all selected channels
  - **Phase 2**: all `mode` writes for all selected channels (kept adjacent in the serial queue)

This is better than before, but still limited by Modbus RTU serial latency (~1–5 ms
per transaction at 115200 baud × 6 frames = 6–30 ms spread at worst for 3 channels).

For applications requiring sub-millisecond start simultaneity a firmware-native
atomic SYNC register is needed:

**Fix (firmware side):**
- [ ] Add `REG_SYNC_MASK` (e.g. register 4) — bitmask of channels (bits 0/1/2).
- [ ] Add `REG_SYNC_CMD` (e.g. register 5) — write the action code to
      atomically apply it to all masked channels in a single ISR pass.
- [ ] Action codes mirror mode values (0=off, 1=dc, 2=pulse, 3=train).
- [ ] With this, the GUI can write mask+params first, then one single SYNC_CMD
      write to start all channels within the same firmware `loop()` tick.

---

## 🟡 Minor — silent validation failures confuse the host

### 5. Watchdog timeout bounds are enforced silently

`Config::HEARTBEAT_MIN_MS = 50`, `Config::HEARTBEAT_MAX_MS = 60000`.  Writes
outside this range return Modbus exception 0x03 (Illegal Value).  The host has
no way to discover the valid range.

**Fix:**
- [ ] Add read-only registers `REG_WD_MIN_MS` and `REG_WD_MAX_MS` (e.g. 106–107).
- [ ] Or document in REG_LAST_ERROR that `IllegalValue = 3` was returned.

---

### 6. Pulse count and duration limits are undiscoverable

| Limit | Value | Register written |
|---|---|---|
| `PULSE_COUNT_MIN` | 1 | CH_COUNT (field 2) |
| `PULSE_COUNT_MAX` | 10 000 | CH_COUNT (field 2) |
| `PULSE_DURATION_MIN_MS` | 1 | CH_PULSE_MS (field 1) |
| `PULSE_DURATION_MAX_MS` | 60 000 | CH_PULSE_MS (field 1) |

Out-of-range writes fail with exception 0x03 but the host receives no numeric
hint about the allowed range.  The GUI has no client-side validation.

**Fix:**
- [ ] Add client-side validation in the GUI (cheapest option).
- [ ] Or expose limit registers (e.g. at registers 108–109 for duration, 110
      area is used — pick a free block).

---

### 7. `PULSE_TRAIN` mode (code 3) silently rejects count < 2

Writing `CH_MODE = 3` when `CH_COUNT` is still at its default of `1` returns
Modbus exception 0x03.  The GUI "Pulse Train" button and "Apply Params + Mode"
do not enforce count ≥ 2 before sending.

**Fix:**
- [ ] GUI: validate count ≥ 2 before writing mode 3, show error dialog.
- [ ] Firmware (optional): when mode 3 is written with count = 1, auto-promote
      count to 2 instead of rejecting.

---

### 8. Watchdog default mismatch between firmware and GUI

| Source | Default |
|---|---|
| `Config::DEFAULT_WATCHDOG_TIMEOUT_MS` | **1 500 ms** |
| `bcon_simple_gui.py` watchdog_var | **1 000 ms** |

After connect the GUI watchdog field auto-fills from the polled register (1 500
ms), but on first connect before any poll there is a brief window where the GUI
would write 1 000 ms if the user clicked "Set WD" immediately.

**Fix:**
- [ ] Align the GUI default to 1 500 ms, or add a comment explaining the
      discrepancy.

---

## 🔵 Informational — not bugs, but worth noting

### 9. `REG_SYS_REASON` (101) always equals `REG_SYS_STATE` (100)

`topLevelStateToCode` and `topLevelReasonCode` return the same integer for each
state (both cast the same enum, 0–3).  REG_SYS_REASON is therefore redundant.

If the intent is to separate "current state" from "why we are in that state"
(e.g. distinguish fault-source), the reason register should encode independent
information (last fault trigger, OC channel index, etc.).

---

### 10. Enable-toggle (field 3) read always returns 0

`readHoldingRegister` for `field == 3` returns `0` ("write-only").  This is
correct behaviour, but the GUI's `read_block(0, 34)` will always see 0 at
registers 13, 23, 33, which is fine as long as the host never uses that value.
No action required — just documenting expected behaviour.

---

### 11. Hardware WDT (8 s) and software WDT are independent

The firmware has two watchdog layers:

| Layer | Timeout | Effect on fail |
|---|---|---|
| Software WDT (`watchdogTimeoutMs`) | 1 500 ms (default) | Forces all outputs LOW, enters `SafeWatchdog` state |
| Hardware WDT (AVR) | 8 s | MCU hard-reset |

The GUI only controls the software WDT via `REG_WATCHDOG_MS`.  There is no
register to read or write the hardware WDT timeout.  If the Modbus task stalls
(e.g. stuck in `delay()`) for > 8 s, a silent MCU reset occurs.  Worth noting
for diagnostics.

---

## Summary table

| # | Severity | Gap | Status | Action |
|---|---|---|---|---|
| 1 | 🔴 Critical | Unmapped regs 3-9/14-19/24-29 → read_block(0,34) always aborts | Open | Return 0 for unmapped reads |
| 2 | 🟠 Major | `REG_TELEMETRY_MS` stored but never acts on | Open | Remove, repurpose, or implement push |
| 3 | 🟠 Major | No REMOTE_ARM register or beam-arm mechanism | Open | Add REG_REMOTE_ARM + gating logic |
| 4 | 🟠 Major | No atomic SYNC register; GUI now uses 2-phase writes | Partially mitigated | Add REG_SYNC_MASK + REG_SYNC_CMD for μs-level sync |
| 5 | 🟡 Minor | Watchdog bounds undiscoverable by host | Open | Add limit registers or GUI validation |
| 6 | 🟡 Minor | Pulse count/duration limits undiscoverable | Open | Add limit registers or GUI validation |
| 7 | 🟡 Minor | Mode 3 silently fails when count < 2 | Mitigated in GUI | GUI validates count ≥ 2 before sending; firmware still rejects silently |
| 8 | 🟡 Minor | Watchdog default mismatch (1500 ms in FW vs 1000 ms in GUI) | Open | Align defaults |
| 9 | 🔵 Info | REG_SYS_REASON always equals REG_SYS_STATE | Open | Differentiate or remove |
| 10 | 🔵 Info | Enable-toggle read always 0 (write-only) | By design | Document only |
| 11 | 🔵 Info | HW WDT (8 s) unobservable from host | Open | Consider reset-reason register |

---

## GUI features implemented (host-side workarounds)

| Feature | Location | Notes |
|---|---|---|
| Per-channel independent config table | Synchronous Control section | Each channel has its own duration/count/mode/enable; 2-phase write |
| Write Params / ▶ Start Selected / ⏹ Stop All | Sync buttons | Validates PULSE_TRAIN count ≥ 2 before sending |
| CSV pulse sequence player | Sequence Player section | Load, template save, run, stop with dwell timing |
| Consecutive-error tolerance | ModbusManager | 4 failures before auto-disconnect |
| Post-connect settle delay | connect() | 2.5 s after DTR-reset before first Modbus frame |
| Split channel status reads | poll loop | 3 × 9-register reads instead of one 29-register read (avoids fw exception 0x02) |

---

## CSV Sequence File Format

Files are stored in `sequences/` (auto-created). Use **Save Template** in the
GUI to generate a commented starter file.  Format:

```csv
# Lines starting with # are ignored
# step: integer — rows with the same step number start together (2-phase write)
# ch: 1, 2, 3, or ALL
# mode: OFF | DC | PULSE | PULSE_TRAIN
# duration_ms: pulse width (PULSE / PULSE_TRAIN only; leave blank for OFF/DC)
# count: pulse count (PULSE_TRAIN must be >= 2; leave blank for OFF/DC)
# dwell_ms: wait after this step completes (last row per step wins)
step,ch,mode,duration_ms,count,dwell_ms
1,1,PULSE,100,5,0
1,2,PULSE,200,1,0
1,3,DC,,,500
2,1,PULSE_TRAIN,50,10,0
2,2,OFF,,,0
2,3,OFF,,,1000
3,ALL,OFF,,,500
```

The sequence player:
1. Validates the file on load (mode names, PULSE_TRAIN count ≥ 2)
2. On **▶ Run**: for each step — write params (phase 1) → write modes (phase 2) → sleep dwell
3. Stop event is checked every 50 ms during dwell so **■ Stop** is responsive
4. Progress is shown in the GUI status bar and written to the session CSV log
