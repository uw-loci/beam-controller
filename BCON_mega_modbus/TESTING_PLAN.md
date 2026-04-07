# BCON Mega Modbus - Comprehensive Testing Plan

## Test Overview

| Component | Priority | Notes |
|-----------|----------|-------|
| System Init | High | Boot behavior, reset flags, WDT setup |
| Modbus Comms | High | All function codes, error handling |
| Output Control | Critical | DC mode validation critical for safety |
| Watchdog/Interlock | High | Safety features must work correctly |
| Fault Management | High | Latching behavior must be reliable |
| LCD Display | Medium | Diagnostic display, not safety-critical |

### Quick Checkboxes
- [ ] INIT-001 Boot Sequence
- [ ] INIT-002 I2C Bus Recovery
- [ ] INIT-003 Watchdog Grace Period
- [ ] RC-001 Read single register
- [ ] RC-002 Read multi-register batch
- [ ] RC-003 Read gap addresses handling
- [ ] WC-001 Watchdog bounds validation
- [ ] WM-001 Write multiple registers success
- [ ] BC-001 Broadcast write behavior
- [ ] DC mode and timing tests
- [ ] Pulse and pulse-train tests
- [ ] Watchdog and interlock safety tests
- [ ] Fault management and clear sequence
- [ ] LCD activity and content verification
- [ ] Edge cases (overflow, CRC, timing)

---

## 1. System Initialization Tests

### Test Case: INIT-001 - Boot Sequence Verification
**Setup:** Fresh firmware upload to Arduino Mega
**Verify:** 
- Watchdog disabled during .init3 phase (MCUSR captured)
- RS485 port configured correctly (Serial vs Serial1 based on BCON_MODBUS_USE_USB_SERIAL)
- All output pins initialized LOW as specified in `setup()`

### Test Case: INIT-002 - I2C Bus Recovery
**Setup:** Simulate stuck SDA line by holding it low externally before power-on
**Verify:** Firmware performs bit-bang recovery (9 clock pulses on pin 21) and successfully initializes LCD if present

### Test Case: INIT-003 - Watchdog Grace Period
**Verify:** `Runtime::watchdogGraceDeadlineMs = millis() + WATCHDOG_BOOT_GRACE_MS` (8000ms)
**Verify:** System stays in READY state during grace period even without host communication

---

## 2. Modbus Communication Tests

### 2.1 Read Holding Registers (Function Code 0x03)

| Test | Register Range | Expected Behavior |
|------|----------------|-------------------|
| RC-001 | Single register read | Returns correct value per `readHoldingRegister()` |
| RC-002 | Multi-register batch (max 125) | Returns all values without exceptions |
| RC-003 | Read gap addresses (3-9, 14-19, etc.) | Returns **0** instead of exception - prevents bulk read failures |
| RC-004 | System status registers | Correct TOP_LEVEL_STATE and reason codes returned |

### 2.2 Write Holding Registers (Function Code 0x06)

| Test | Register | Value Range | Expected Behavior |
|------|----------|-------------|-------------------|
| WC-001 | REG_WATCHDOG_MS | <50ms or >60s | **Returns exception 0x03** (IllegalValue) |
| WC-002 | REG_TELEMETRY_MS | Any valid uint16 | Accepts, updates Runtime::telemetryPeriodMs |
| WC-003 | REG_COMMAND=1 | Value=1 | All channels OFF immediately |
| WC-004 | REG_COMMAND=2/3 with fault active | Fault still asserted | **Returns exception 0x04** (FaultStillActive) - prevents unsafe clearing |
| WC-005 | REG_COMMAND=4 | Pending staged CHx mode writes | Commits all staged non-OFF channel starts together |

### 2.3 Write Multiple Registers (Function Code 0x10)

| Test | Scenario | Expected Behavior |
|------|----------|-------------------|
| WM-001 | Valid multi-register write | All registers updated successfully |
| WM-002 | Invalid byte count mismatch | Returns exception 0x03 before any writes |
| WM-003 | Write to invalid address mid-batch | Stops at error, returns exception (partial update) |

### 2.4 Broadcast Writes (Address=0)

| Test | Scenario | Verify |
|------|----------|--------|
| BC-001 | Broadcast write with valid data | No acknowledgment frame sent back |
| BC-002 | Broadcast to invalid address | No response expected |

---

## 3. Output Control Tests

**Critical: Use oscilloscope for pulse timing validation**

### 3.1 DC Mode (Mode=1)

```cpp
// Test: stage channel mode to DC, then write REG_COMMAND=4 and verify continuous HIGH
```

| Check | Expected |
|-------|----------|
| Gate pin state | Continuous HIGH after command accepted |
| Channel timer | Cleared (no countdown running) |
| Enable toggle | Pulses if enable_status input is LOW |

### 3.2 Pulse Mode (Mode=2, count=1)

```cpp
// Test: set pulse duration to 500ms, stage mode=2, then write REG_COMMAND=4
```

| Check | Expected |
|-------|----------|
| Initial state | Gate HIGH immediately |
| After pulse duration | Gate returns LOW automatically |
| pulsesRemaining | Decrements from 1→0, then mode becomes OFF |
| Timer behavior | Corresponding TIMER3/4/5 compare ISR is armed for durationMs |

### 3.3 Pulse Train Mode (Mode=3)

```cpp
// Test: set pulse train with duration=200ms, count=5, then write REG_COMMAND=4
```

| Check | Expected |
|-------|----------|
| Pattern observed on oscilloscope | HIGH(200ms) → LOW(200ms gap) × 4 → HIGH(200ms) → LOW (stop) |
| pulsesRemaining | Counts down correctly through interrupts |
| State transitions | MODE→PULSE_TRAIN→OFF as designed |

### 3.4 Output Priority Tests

```cpp
// Test: Command DC mode, then immediately send watchdog timeout scenario
```

| Check | Expected |
|-------|----------|
| When watchdog expires | **All outputs forced LOW** (applyOutputs() checks TopLevelState) |
| After watchdog restored via heartbeat | Outputs resume per current channel commands |

---

## 4. Watchdog & Interlock Tests

### 4.1 Software Watchdog Behavior

| Test Case | Scenario | Verify |
|-----------|----------|--------|
| WD-001 | No host communication for >WATCHDOG_TIMEOUT_MS (default 1500ms) | All outputs forced LOW, TopLevelState→SafeWatchdog |
| WD-002 | Heartbeat write within timeout window | System stays in READY state |
| WD-003 | Timeout + Interlock failure simultaneously | **Interlock priority** - SafeInterlock reported before SafeWatchdog |

### 4.2 Interlock Testing

```cpp
// Pin: Config::KB_INTERLOCK_PIN = 22
// Active HIGH with no pullup (Config::INTERLOCK_ACTIVE_HIGH=true, INTERLOCK_USE_PULLUP=false)
```

| Test Case | Input State | Expected Behavior |
|-----------|-------------|-------------------|
| IL-001 | Interlock pin LOW | TopLevelState→SafeInterlock, all outputs forced OFF |
| IL-002 | Interlock pin HIGH with DC command | Outputs drive per channel configuration (READY state) |

### 4.3 Combined Safety Tests

```cpp
// Test: Simulate both watchdog timeout AND interlock failure
```

Expected priority order in `evaluateTopLevelState()`:
1. **SafeInterlock** - highest priority
2. SafeWatchdog
3. FaultLatched
4. Ready

---

## 5. Fault Management Tests

### 5.1 Overcurrent Detection

| Test Case | Status Pin State | Expected Behavior |
|-----------|------------------|-------------------|
| FC-001 | OVERCURRENT_STATUS_PIN LOW (active) on any channel | Runtime::faultLatched = true immediately in loop() |
| FC-002 | Clear command (REG_COMMAND=3) with fault active | **Exception 0x04** - cannot clear while fault asserted |

### 5.2 Fault Latch Reset Sequence

```cpp
// Valid reset sequence: remove overcurrent condition → write REG_COMMAND=2 or 3
```

| Check | Expected |
|-------|----------|
| After command accepted | Runtime::faultLatched = false, TopLevelState returns to READY (if no other issues) |
| Fault status register | Returns 0 after clear |

---

## 6. LCD Display Tests

### 6.1 LCD During Modbus Activity

**CRITICAL: This is a race-condition sensitive area - verify thoroughly**

Current strategy: LCD refreshes in the inter-poll quiet window (~250 ms gap between
dashboard polls). A refresh is skipped — and deferred 50 ms — if any Modbus byte
arrived in the last 50 ms (`framePending`). TWI is disabled (`TWCR = 0`) between
refreshes to prevent electrical noise from corrupting the PCF8574 output register.
I2C clock runs at 400 kHz, keeping the full 4-row refresh to ~20 ms.

| Test Case | Verify |
|-----------|--------|
| LC-001 | LCD **does update** ~once per second even while host is connected (quiet-window refresh) |
| LC-002 | TWI peripheral disabled (`TWCR = 0`) between refreshes and re-enabled only for the ~20 ms render window |
| LC-003 | No LCD update occurs within 50 ms of the last Modbus byte (defer path fires, retries in 50 ms) |
| LC-004 | First frame after boot (before any valid command): `framePending` alone blocks I2C — no race with first incoming frame |

### 6.2 LCD Content Verification

During idle (no Modbus activity):

```
Row 0: "WDG:OK INT:OK FLT:0" or similar status summary
Row 1-3: Per-channel info - e.g., "CH1 DC O:1 R:0"
```

| Check | Expected Format |
|-------|-----------------|
| Mode abbreviations | OFF, DC , PUL, TRN (modeToShortString) |
| Output level | O:1 when HIGH, O:0 when LOW |
| Pulse remaining | R:<count> for pulse/pulse-train modes |

---

## 7. Edge Cases & Stress Tests

### 7.1 Buffer Overflow Protection

```cpp
// Modbus receive buffer is MODBUS_RX_BUFFER_SIZE = 256 bytes
```

| Test Case | Setup | Verify |
|-----------|-------|--------|
| EO-001 | Send frame larger than 254 bytes (before CRC) | Runtime::modbusRxIndex resets, error set to BufferOverflow |

### 7.2 Frame Timing Tests

```cpp
// MODBUS_FRAME_GAP_MS = 4ms - inter-frame silence required for new frame detection
```

| Test Case | Expected Behavior |
|-----------|-------------------|
| EO-002 | Rapid consecutive frames <4ms apart | Second frame appended to same buffer (single frame) |
| EO-003 | Valid frame followed immediately by garbage | First frame processed, garbage accumulates in next receive cycle |

### 7.3 CRC Error Handling

```cpp
// ModbusCrc16 uses standard polynomial 0xA001 (reflected)
```

| Test Case | Expected Behavior |
|-----------|-------------------|
| EO-004 | Frame with incorrect CRC | **Silently discarded** - no exception sent, frame ignored |
| EO-005 | Valid CRC but bad data in payload | Processed normally (CRC protects integrity only) |

### 7.4 Timer Overflow Tests

```cpp
// uint32_t for countdowns → overflows after ~49 days at 1ms ticks
```

| Test Case | Verify |
|-----------|--------|
| EO-006 | Long-duration pulse trains (>49 days) | Countdown wraps correctly due to unsigned arithmetic |

---

## 8. Additional Considerations

### 8.1 Configuration Testing

Verify each compile-time constant works as intended:

```cpp
// Toggle these defines and verify behavior changes:
#define BCON_MODBUS_USE_USB_SERIAL   // Serial vs Serial1 port selection
#define BCON_ENABLE_I2C_LCD          // LCD subsystem enabled/disabled
```

### 8.2 Power Cycle Tests

| Test Case | Verify |
|-----------|--------|
| PC-001 | Cold boot - all outputs start LOW, watchdog grace active |
| PC-002 | WDT reset scenario (stall loop >8s) - MCUSR captures cause |
| PC-003 | Partial power cycle (LCD disconnected mid-operation) - graceful degradation |

### 8.3 Pin Consistency Verification

Create a pin mapping spreadsheet to verify all Config pins match hardware schematics:

```
Channel Control:     Gate(5,6,7), EnableToggle(8,9,10)
Status Inputs:       Enable(23,27,31), Power(24,28,32), OC(25,29,33), Gated(26,30,34)
Interlock:           KB_INTERLOCK_PIN = 22
RS485 DE/RE:         RS485_DE_RE_PIN = 2
LCD I2C:             SDA(20), SCL(21) - used in bit-bang recovery only
```

---

## Test Execution Priority Matrix

| Phase | Tests | Duration Estimate | Pass Criteria |
|-------|-------|-------------------|---------------|
| P0 - Safety Critical | INIT-*, WD-*, IL-*, FC-* | 2 hours | All outputs OFF when fault conditions detected |
| P1 - Core Functionality | RC-*, WC-*, Output Control tests | 4 hours | All basic operations work correctly |
| P2 - Communication | Modbus function code tests, error handling | 3 hours | Protocol compliance verified |
| P3 - Edge Cases | Buffer overflow, CRC errors, timing | 1 hour | Graceful degradation, no crashes |
| P4 - LCD/Diagnostics | Display content, activity detection | 1 hour | Accurate status reporting |

---

## Recommended Test Equipment

1. **Oscilloscope** (essential) - Verify pulse widths and timing accuracy
2. **Multimeter** - Verify DC mode voltage levels
3. **Logic analyzer** - Optional, for I2C bus monitoring during LCD tests
4. **Modbus master tool** - Python script or commercial tool for automated testing

---

## Notes on Firmware Design Issues Found During Review

1. **LCD/TWI Quiet-Window Refresh**: TWI is disabled between refreshes to block noise from USB serial traffic. The LCD updates ~once per second in the inter-poll gap, so the display stays live while the host is connected. Any refresh attempt within 50 ms of a Modbus byte is deferred; the gate keys off `framePending` (recent serial activity) alone — not on whether a full valid frame has completed — to close the first-frame race at boot or after >5 s idle.

2. **Silent CRC Errors**: Invalid CRC frames are silently discarded without logging or status update. May make debugging communication issues harder.

3. **Watchdog Grace Period**: 8-second grace window may be too long for some applications - consider making it configurable.

4. **Unsupported Function Codes**: Only FC 0x03, 0x06, and 0x10 are implemented. Any other function code (e.g., FC 0x01, 0x02) returns Modbus exception 0x01 (Illegal Function) — not silently ignored.
