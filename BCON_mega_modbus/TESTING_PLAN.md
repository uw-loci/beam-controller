# BCON Mega Modbus - Current Firmware Test Plan

This plan matches the current source in `BCON_mega_modbus.ino` and `bcon_types.h`.
It replaces the older plan that targeted the retired custom Modbus implementation.

## Scope

Current firmware assumptions:

- Target: Arduino Mega 2560
- Default transport for bench work: `BCON_USE_USB_SERIAL = 1`
- Optional production transport: `BCON_USE_USB_SERIAL = 0` on `Serial1` with `RS485_DE_RE = 17`
- LCD enabled in current bench build: `BCON_ENABLE_LCD = 1`
- Slave ID: `1`
- Baud rate: `115200`
- Watchdog default: `1500 ms`
- Watchdog grace period: `8000 ms`

Current contract details that matter for testing:

- Register `2` (`COMMAND`) is `0=NOP`, `1=AllOff`, `2/3=ClearFault`
- `SYS_STATE` is the watchdog keepalive read path
- `LAST_ERROR` auto-clears on read
- Channel status offsets `4-7` are intentionally not wired and should read `0`
- `Pulse` is auto-promoted to `PulseTrain` when `COUNT > 1`
- `g_faultLatched` exists, but the current source has no hardware fault input path that asserts it during normal bench testing

## Recommended Bench Setup

Required equipment:

1. Oscilloscope for gate and enable-toggle timing checks
2. Multimeter for steady-state DC output checks
3. Bench interlock source so pin `A0` can be driven HIGH and LOW on demand
4. Modbus master tool or the updated GUI in `test_interfaces/pulser_test_gui.py`

Recommended hookup notes:

- Gate outputs: `A11`, `A13`, `A9`
- Gate indicator LEDs: `3`, `5`, `7`
- Enable-toggle outputs: `12`, `11`, `10`
- Interlock input: `A0`, active HIGH, no pull-up enabled in firmware
- LCD I2C: `SDA=20`, `SCL=21`

Host-side note:

- Opening the USB serial port resets the Mega via DTR. Wait about `2.5 s` after connect before issuing the first Modbus command.

## Quick Checklist

- [ ] INIT-001 Boot safe outputs low
- [ ] INIT-002 Bench transport selection verified
- [ ] INIT-003 Watchdog grace behavior with interlock HIGH and LOW
- [ ] REG-001 GUI-compatible register block reads
- [ ] REG-002 Illegal-address read across channel-status gaps
- [ ] REG-003 `LAST_ERROR` auto-clear behavior
- [ ] CMD-001 `COMMAND=0` NOP
- [ ] CMD-002 `COMMAND=1` AllOff
- [ ] CMD-003 `COMMAND=3` ClearFault with interlock LOW and HIGH
- [ ] CH-001 DC mode output
- [ ] CH-002 Single pulse timing
- [ ] CH-003 Pulse auto-promotes to PulseTrain when count > 1
- [ ] CH-004 Enable-toggle 100 ms pulse
- [ ] SAFE-001 Interlock safe state
- [ ] SAFE-002 Watchdog expiry and keepalive path
- [ ] SAFE-003 Non-Ready mode-write rejection
- [ ] LCD-001 LCD detect and quiet-window update
- [ ] GUI-001 Updated GUI interoperability smoke test

---

## 1. Initialization and Build Tests

### INIT-001 - Boot Sequence and Safe Outputs

Setup:

- Fresh power-on or reset
- Observe gate outputs and enable-toggle outputs at boot

Verify:

- `captureAndDisableWdt()` runs early enough that a watchdog-caused reboot does not loop forever
- All gate outputs start LOW
- All gate LED outputs start LOW
- All enable-toggle outputs start LOW

Pass criteria:

- No output pin glitches HIGH during boot
- Device reaches a stable Modbus-ready state after boot

### INIT-002 - Transport Configuration

Bench build test:

- With `BCON_USE_USB_SERIAL = 1`, verify Modbus works on USB serial

Optional production build test:

- With `BCON_USE_USB_SERIAL = 0`, verify Modbus works on `Serial1`
- Verify DE/RE pin `17` controls RS-485 direction correctly

### INIT-003 - Watchdog Grace Period

Setup:

- Power cycle the board
- Do not send host traffic for the first 8 seconds

Verify:

- If interlock is HIGH, `SYS_STATE` can report `Ready` during the grace window
- If interlock is LOW, `SYS_STATE` reports `SafeInterlock` even during the grace window
- After the grace window, lack of keepalive transitions the unit to `SafeWatchdog`

### INIT-004 - LCD Presence and Absence

Verify both cases:

- LCD attached at `0x27`
- LCD attached at `0x3F`
- No LCD attached at all

Pass criteria:

- The firmware continues to run correctly in all three cases
- No-LCD operation does not block Modbus communications

---

## 2. Register Map and Communication Tests

### 2.1 Read Tests

| Test | Read | Expected Result |
|------|------|-----------------|
| REG-001A | `0-33` | Succeeds; includes control registers and current values used by the GUI |
| REG-001B | `100-105` | Succeeds; returns system state, reason, fault, interlock, watchdog, last error |
| REG-001C | `110-118` | Succeeds; CH1 live status |
| REG-001D | `120-128` | Succeeds; CH2 live status |
| REG-001E | `130-138` | Succeeds; CH3 live status |
| REG-002 | `110-138` as one contiguous block | Expected to fail because `119` and `129` are unimplemented gaps |

Additional checks:

- Verify channel status offsets `4-7` return `0`
- Verify `output_level` at offset `8` follows the actual gate pin level

### 2.2 Write Tests

| Test | Register | Input | Expected Result |
|------|----------|-------|-----------------|
| REG-010 | `WATCHDOG_MS` | `50`, `1500`, `60000` | Accepted |
| REG-011 | `WATCHDOG_MS` | `49`, `60001` | Rejected by callback; value stays unchanged; `LAST_ERROR=3` |
| REG-012 | `TELEMETRY_MS` | `0`, `1`, `1500`, `65535` | Accepted |
| REG-013 | `CHx_PULSE_MS` | `1`, `100`, `60000` | Accepted |
| REG-014 | `CHx_PULSE_MS` | `0`, `60001` | Rejected; `LAST_ERROR=3` |
| REG-015 | `CHx_COUNT` | `1`, `2`, `10000` | Accepted |
| REG-016 | `CHx_COUNT` | `0`, `10001` | Rejected; `LAST_ERROR=3` |

### 2.3 Error Register Behavior

#### REG-003 - `LAST_ERROR` Auto-Clears

Procedure:

1. Force a known error, for example write `WATCHDOG_MS=49`
2. Read register `105` once
3. Read register `105` again

Verify:

- First read returns `3`
- Second read returns `0`

Note:

- Any host that polls `105` continuously must surface the value immediately because the read clears it.

---

## 3. Command Register Tests

### CMD-001 - `COMMAND=0` NOP

Verify:

- No mode changes occur
- No outputs change state
- Command register reads back `0`
- Watchdog is fed by the write callback

### CMD-002 - `COMMAND=1` AllOff

Setup:

- Start one or more channels in `DC`, `Pulse`, or `PulseTrain`

Verify:

- All gate outputs go LOW immediately
- All three `CH_MODE` control registers are forced to `0`
- Live status mode also returns `OFF`

### CMD-003 - `COMMAND=2` and `COMMAND=3` ClearFault

Verify current behavior:

- Both values behave the same in the current source
- With interlock LOW, the command is rejected and `LAST_ERROR=12`
- With interlock HIGH, the command is accepted and clears `g_faultLatched` if it had been set

Important note:

- The current firmware does not expose a true arm/disarm command. Do not treat `COMMAND=3` as arm.

---

## 4. Channel Output and Timing Tests

Use an oscilloscope for all pulse timing checks.

### CH-001 - DC Mode

Procedure:

1. Set interlock HIGH
2. Keep watchdog alive
3. Write `CH_MODE=1`

Verify:

- Gate output goes HIGH and stays HIGH
- LED output mirrors the gate output
- Channel status mode returns `DC`
- `remaining` stays `0`

### CH-002 - Single Pulse

Procedure:

1. Write `PULSE_MS=500`
2. Write `COUNT=1`
3. Write `MODE=2`

Verify:

- Gate goes HIGH immediately
- Gate returns LOW after about `500 ms`
- `remaining` transitions to `0`
- Mode returns to `OFF` after the pulse completes

### CH-003 - Pulse Auto-Promotion

Procedure:

1. Write `PULSE_MS=200`
2. Write `COUNT=5`
3. Write `MODE=2` (`Pulse`)

Verify:

- The firmware stores the effective mode as `PulseTrain`
- The output pattern is `HIGH 200 ms` then `LOW 200 ms`, repeated five times total
- The channel returns to `OFF` at the end

### CH-004 - Explicit PulseTrain

Procedure:

1. Write `MODE=3`
2. Test with `COUNT=5`

Verify:

- Behavior matches the pulse-train timing above

Characterization check:

- Also test `MODE=3` with `COUNT=1` and document the observed result for regression tracking

### CH-005 - Enable Toggle Output

Procedure:

1. Write `CH_ENA_TOGGLE=1`

Verify:

- Corresponding enable-toggle output goes HIGH for about `100 ms`
- Output then returns LOW automatically
- This path works even when the top-level state is not `Ready`

### CH-006 - Status Register Mapping

Verify:

- Offset `0`: active mode
- Offset `1`: configured pulse duration
- Offset `2`: configured count
- Offset `3`: pulses remaining in current run
- Offsets `4-7`: always `0`
- Offset `8`: gate output level

---

## 5. Safety and State-Machine Tests

### SAFE-001 - Interlock Safe State

Procedure:

1. Start a channel in `DC`
2. Pull interlock LOW

Verify:

- `SYS_STATE` becomes `SafeInterlock`
- All gate outputs are forced LOW
- LED outputs are forced LOW

### SAFE-002 - Watchdog Expiry

Procedure:

1. Keep interlock HIGH
2. Stop all writes and stop reading register `100`
3. Wait longer than the watchdog timeout after the grace window

Verify:

- `SYS_STATE` becomes `SafeWatchdog`
- All gate outputs are forced LOW

### SAFE-003 - Keepalive Path

Verify both paths:

- Reading `SYS_STATE` (`100`) keeps the watchdog alive
- Writing any register with a callback also keeps the watchdog alive

Important note:

- Do not assume that reading arbitrary holding registers feeds the watchdog. The explicit read keepalive path is `SYS_STATE`.

### SAFE-004 - State Priority

Confirm priority order in `evaluateState()`:

1. `SafeInterlock`
2. `SafeWatchdog`
3. `FaultLatched`
4. `Ready`

### SAFE-005 - Non-Ready Mode Write Rejection

Procedure:

1. Make the controller non-ready by opening the interlock or letting the watchdog expire
2. Attempt to write `MODE=1`, `2`, or `3`

Verify:

- The mode write is rejected
- `LAST_ERROR=10`
- The existing channel mode remains unchanged

### SAFE-006 - Recovery Behavior

Verify current behavior after returning to `Ready`:

- DC channels resume HIGH because the mode remains `DC`
- Pulse and pulse-train channels should be characterized and documented because their timing state is not explicitly paused during non-ready intervals

---

## 6. Fault-Handling Tests

### FLT-001 - Current Bench Limitation

Current source limitation:

- `g_faultLatched` is present in the state machine, but there is no hardware input path in the current firmware that asserts it during normal bench use

Pass criteria for current bench build:

- `FAULT_LATCHED` stays `0` unless an instrumented test build is used

### FLT-002 - ClearFault with Interlock LOW

Procedure:

1. Force interlock LOW
2. Write `COMMAND=3`

Verify:

- `LAST_ERROR=12`
- No unsafe state transition occurs

### FLT-003 - ClearFault with Interlock HIGH

Procedure:

1. Force interlock HIGH
2. Write `COMMAND=3`

Verify:

- Command is accepted
- If no fault is latched, the operation is effectively a no-op

### FLT-004 - Instrumented Fault-Latch Test

Optional debug-build test:

- Add a temporary test hook that sets `g_faultLatched = true`
- Verify `SYS_STATE=FaultLatched`
- Verify `COMMAND=3` clears the latched fault when interlock is HIGH

---

## 7. LCD Tests

### LCD-001 - Address Detect and Graceful Absence

Verify:

- LCD is found at `0x27` when present
- LCD falls back to `0x3F` when that is the only valid address
- If neither address responds, the firmware continues running without LCD updates

### LCD-002 - Quiet-Window Refresh

Verify:

- LCD refresh is deferred if a serial byte arrived in the last `50 ms`
- LCD refresh cadence is about `1 s`
- TWI remains enabled after setup; LCD updates do not toggle `TWCR` or re-run `Wire.begin()` per row

### LCD-003 - Content Format

Expected pattern:

- Row 0: watchdog, interlock, fault summary
- Rows 1-3: `CHn`, mode abbreviation, output level, remaining count

Verify abbreviations:

- `OFF`
- `DC `
- `PUL`
- `TRN`

---

## 8. GUI Interoperability Smoke Tests

These are bench tests for the updated GUI in `test_interfaces/pulser_test_gui.py`.

### GUI-001 - Connect and Poll

Verify:

- GUI connects at `115200`, unit ID `1`
- GUI waits through the Mega reset window after opening the port
- GUI can read `0-33`, `100-105`, and the three 9-register channel status blocks

### GUI-002 - Clear Fault Button

Verify:

- The GUI sends `COMMAND=3`
- The GUI no longer presents an arm/disarm workflow that the firmware does not support

### GUI-003 - Stop Paths

Verify:

- Sync Stop uses `COMMAND=1`
- Stopping a running CSV sequence forces `AllOff`

### GUI-004 - Error Visibility

Verify:

- Trigger `LAST_ERROR=3` or `12`
- Confirm the GUI surfaces it immediately before the register is cleared by the read

---

## 9. Configuration Matrix

Run at least one smoke test for each supported compile-time mode:

| Config | Expected Result |
|--------|-----------------|
| `BCON_USE_USB_SERIAL=1`, `BCON_ENABLE_LCD=1` | Current bench baseline |
| `BCON_USE_USB_SERIAL=1`, `BCON_ENABLE_LCD=0` | No LCD, Modbus still works |
| `BCON_USE_USB_SERIAL=0`, `BCON_ENABLE_LCD=1` | RS-485 transport works with DE/RE pin `17` |

---

## 10. Removed or Not-Applicable Legacy Tests

The following items from the old plan do not match the current firmware and should not be used as acceptance criteria:

- Bit-bang I2C bus recovery before LCD init
- Custom Modbus RX buffer overflow and CRC parser tests owned by in-repo protocol code
- Bench tests for enable-status, power-status, overcurrent-status, and gated-status input pins
- Any remote-arm or arm-beam register workflow
- Assumptions that reading every holding register range is legal; the current register map has real gaps

---

## Execution Priority

| Phase | Focus | Pass Criteria |
|-------|-------|---------------|
| P0 | Interlock, watchdog, AllOff, non-ready rejection | Outputs always fail safe |
| P1 | Register map, read/write validation, `LAST_ERROR` behavior | Host contract matches source |
| P2 | DC, pulse, pulse-train, enable-toggle timing | Measured outputs match configured timing |
| P3 | LCD and GUI interoperability | Diagnostics stay accurate under live polling |
| P4 | Optional instrumented fault-latch tests and RS-485 build | Extended coverage complete |
