# BCON Mega Modbus — Pulser Firmware

**Target hardware:** Arduino Mega 2560 (ATmega2560)  
**Source file:** `BCON_mega_modbus.ino`

---

## Overview

This firmware turns an Arduino Mega 2560 into a **3-channel beam-gate controller** for the Beam Controller (BCON) system. It is responsible for **pulse generation and beam on/off control** only. Beam deflection (BOP Amp waveform generation) is handled by a separate subsystem and is outside the scope of this firmware.

Each channel corresponds to one electron beam cathode. The firmware controls a digital **pulse-gate output pin** per channel (the signal that enables or suppresses the beam), and monitors a set of **status input pins** from the external hardware (enable status, power status, overcurrent, gated).

The host computer communicates with the firmware using **Modbus RTU** over a USB-CDC serial port (default) or RS-485 (build-time switchable). All configuration, commanding, and status polling is performed through the Modbus holding register map described below.

---

## Firmware Responsibilities (Scope)

| Responsibility | Status |
|---|---|
| Per-channel DC beam on/off | ✅ Implemented |
| Per-channel single pulse with configurable duration | ✅ Implemented |
| Per-channel pulse train (N pulses, configurable duration & count) | ✅ Implemented |
| Software watchdog — forces all beams off if host heartbeat stops | ✅ Implemented |
| Hardware WDT (8 s) — MCU hard-reset on firmware lock-up | ✅ Implemented |
| Knob Box interlock input — SafeInterlock state | ✅ Implemented |
| Overcurrent fault latch — latches and forces outputs off | ✅ Implemented |
| Enable-toggle output per channel (100 ms pulse to external latch) | ✅ Implemented |
| Modbus RTU status reporting (system state, interlock, WD, OC) | ✅ Implemented |
| 20×4 I2C LCD local status display | ✅ Implemented |
| Beam deflection / BOP Amp waveform generation | ❌ Out of scope |
| Dashboard GUI / Beam position visualization | ❌ Out of scope |

---

## Safety Architecture

### Top-Level State Machine

The firmware continuously evaluates a **top-level system state** that gates all output activity. Outputs are only driven when the state is `Ready`. Any other state forces all pulse-gate outputs to `LOW` immediately.

```
┌──────────────┐
│    Ready     │  All outputs operate normally
└──────┬───────┘
       │ Interlock pin deasserted
       ▼
┌──────────────────┐
│  SafeInterlock   │  All outputs forced LOW
└──────┬───────────┘
       │ Watchdog timeout exceeded
       ▼
┌──────────────────┐
│  SafeWatchdog    │  All outputs forced LOW
└──────┬───────────┘
       │ Overcurrent latched
       ▼
┌──────────────────┐
│  FaultLatched    │  All outputs forced LOW
└──────────────────┘
```

| State | Code | Description |
|---|---|---|
| `Ready` | 0 | System healthy, outputs follow channel mode commands |
| `SafeInterlock` | 1 | Knob Box interlock pin is not asserted; all outputs forced OFF |
| `SafeWatchdog` | 2 | No valid Modbus frame received within `WATCHDOG_MS` timeout; all outputs forced OFF |
| `FaultLatched` | 3 | Overcurrent was detected; fault must be explicitly cleared by host before outputs can resume |

### Software Watchdog

Any valid Modbus frame (any function code) resets the watchdog timer. The host must send at least one frame every `WATCHDOG_MS` milliseconds (register 0, default 1500 ms, range 50–60 000 ms) to keep the system in `Ready` state. If the host disconnects or crashes, outputs are forced off automatically.

An **8-second boot grace window** prevents the watchdog from firing during startup before the host has had time to connect.

### Hardware Watchdog (ATmega WDT)

A hardware watchdog timer is enabled at the end of `setup()` with an 8-second timeout. `loop()` resets it every iteration. A firmware lock-up causes a full MCU reset, at which point the startup `.init3` hook immediately re-disables the WDT and clears `MCUSR` to avoid an infinite reset loop.

### Interlock Input — Hardware and Software Layers

The Knob Box "Arm Beams" signal enters BCON via an RJ45 panel port and is **optically isolated** by a SparkFun opto-isolator board (BCON-200, ILD205T-based, non-inverting output). The isolated output drives two paths simultaneously:

1. **Hardware AND gate layer (BCON-210, SN74HCT08N):** The isolated Arm Beams signal is wired to two inputs of the 4-circuit AND gate (pins 1A, 3B, 4B). The AND gate also receives the Mega's gate-drive pulse outputs. The TC4427 gate drivers can only be triggered when **both** the firmware signal and the interlock signal are simultaneously HIGH. This enforcement is hardware-fast (nanoseconds), independent of firmware.

2. **Software interlock (Mega pin A0):** The same isolated signal is also read by the Mega on analog pin A0 used in digital input mode (`HIGH` = interlock satisfied). When A0 reads LOW, the firmware enters `SafeInterlock` state and forces all outputs LOW in software — preventing new commands from being accepted and stopping any in-progress pulse train. This provides the Modbus status reporting layer and the watchdog-level enforcement.

The two layers together ensure that even if firmware is delayed or misbehaves, no gate signal can reach a pulser unless the Knob Box interlock is asserted.

Pull-down resistors (10 kΩ) on the opto-isolator output and on each AND gate interlock input ensure all lines default LOW if the cable is disconnected.

### Overcurrent Fault Latch

Each channel has an active-low `OVERCURRENT_STATUS` input. If any overcurrent input is asserted, `faultLatched` is set in `loop()` and the system enters `FaultLatched`. To clear the fault the host must write `2` or `3` to the `COMMAND` register (reg 2). The firmware checks that the overcurrent condition is no longer active and the interlock is satisfied before allowing the fault to be cleared.

---

## Output Modes

Each channel independently supports four output modes:

| Mode | Code | Behavior |
|---|---|---|
| `Off` | 0 | Gate output held LOW |
| `DC` | 1 | Gate output held HIGH continuously |
| `Pulse` | 2 | Gate output HIGH for `pulse_duration_ms`, then OFF (single shot). If `pulse_count > 1` when mode 2 is written, the firmware automatically runs a pulse train. |
| `PulseTrain` | 3 | Gate output alternates HIGH/LOW for `pulse_count` pulses, each HIGH phase lasting `pulse_duration_ms`; gap between pulses equals pulse duration (50 % duty cycle). |

Switching to `DC`, `Pulse`, or `PulseTrain` mode while the channel's `ENABLE_STATUS` input reads as not-enabled automatically triggers a 100 ms **enable-toggle pulse** on the corresponding `PULSER_ENABLE_TOGGLE_OUTPUT_PIN` to assert the external enable latch.

Pulse timing is driven by dedicated **16-bit compare timers** (`TIMER3`, `TIMER4`, `TIMER5`), one per channel. Each channel schedules its next edge from an ISR using microsecond-based compare chunks, so pulse widths stay stable even while Modbus traffic and LCD work continue in `loop()`.

---

## Hardware Signal Chain (Gate Drive Path)

The following chain describes how a firmware pulse-gate output reaches the PVX-4140 pulser:

```
Mega Gate Drive Output (A9/A11/A13)
  │
  ├── 100 Ω series resistor (BCON-340, ring/spike protection)
  │
  └──► AND Gate input A or B (SN74HCT08N, BCON-210)
           │
           ├── other input: Arm Beams isolated signal (from BCON-200)
           │                  10 kΩ pulldown to GND if cable disconnected
           │
           └──► AND Gate Output ──► TC4427CPA MOSFET Gate Driver (BCON-220)
                                        │   Vdd = 9 V (from AC-DC supply)
                                        │   10 kΩ pulldown on input
                                        │   100 nF + 1 µF ceramic decoupling on Vdd
                                        │   30 Ω series output resistance
                                        │
                                        └──► Panel-mount 50 Ω BNC (BCON-260)
                                                 │
                                                 └──► 50 Ω coax cable
                                                          │
                                                          └──► PVX-4140 Gate BNC
```

Key design details:
- The AND gate ensures the TC4427 can only fire when **both** the Mega asserts its output AND the Knob Box interlock (Arm Beams) is satisfied — no firmware or software path can bypass this.
- TC4427 is powered from the 9 V rail with a 30 Ω source series resistance, resulting in ~5 V at the 50 Ω PVX gate input (voltage divider: 9 V × 50/(50+40) ≈ 5 V), satisfying the PVX gate trigger threshold.
- The Mega's output (A9/A11/A13) and the interlock feedback to A0 are both referenced to the same Arduino GND; the 9 V gate-driver domain is isolated by the TC4427, which has its own 9 V GND referenced to the AC-DC supply GND.
- TVS diodes (BCON-160, BCON-161) clamp transients on the 9 V supply and return lines.

### Enable Toggle Path (per channel)

```
Mega Enable Gate Pin (D10/D11/D12)
  │
  └──► 2N7000 NMOS gate (BCON-350)
           │   Drain ─── PVX DA-15 Pin 9 (Enable)
           │   Source ── GND
           │   (pulls Enable line low on rising Mega output edge)
           │
           └── protects Mega from the PVX's 24 V pullup on Pin 9
```

### Status Input Path (per channel)

```
PVX-4140 DA-15 Remote Interface (Pins 6, 8, 14, 15)
  │  (active-low, open-drain, ≤24 V)
  │
  └──► ISO7740 Quad Digital Isolator (inputs A/B/C/D)
           │
           └──► Mega digital input pins (D23–D34 per firmware assignment)
                    (active-low, internal pull-up enabled)
```

---

## Pin Assignments

> **Note:** The pin assignments below reflect the hardware design per the BCON Hardware Development Specification (Rev 2026-03-17). The current firmware `Config` constants were originally assigned to different pins and **must be updated** to match this table before programming hardware-built units. See [Build Configuration](#build-configuration) for the relevant macros.

### Digital Outputs

| Arduino Pin | HW Spec Label | Function | Notes |
|---|---|---|---|
| **A9** | Pulser C Gate Drive Output | CH-C pulse-gate digital output | Through AND gate → TC4427 gate driver → 50 Ω BNC |
| **A11** | Pulser A Gate Drive Output | CH-A pulse-gate digital output | Through AND gate → TC4427 gate driver → 50 Ω BNC |
| **A13** | Pulser B Gate Drive Output | CH-B pulse-gate digital output | Through AND gate → TC4427 gate driver → 50 Ω BNC |
| **D3** | Pulser A Gate Status LED | CH-A gate state indicator | Blue panel LED; driven HIGH when CH-A gate output is HIGH |
| **D5** | Pulser B Gate Status LED | CH-B gate state indicator | Blue panel LED; driven HIGH when CH-B gate output is HIGH |
| **D7** | Pulser C Gate Status LED | CH-C gate state indicator | Blue panel LED; driven HIGH when CH-C gate output is HIGH |
| **D10** | Pulser C Enable Gate | CH-C pulser enable toggle | Drives 2N7000 NMOS gate; transistor pulls PVX DB15 pin 9 low to toggle enable |
| **D11** | Pulser B Enable Gate | CH-B pulser enable toggle | Same as above |
| **D12** | Pulser A Enable Gate | CH-A pulser enable toggle | Same as above |
| **D17** | UART RE Select | RS-485 transceiver DE/RE | Direction control for MAX485-compatible transceiver module |
| **D18 (TX1)** | UART TX | Serial1 RS-485 transmit | Connected to DI on RS-485 transceiver (BCON-335) |
| **D20 (SDA)** | SDA | I2C data | LCD (Wire library) |
| **D21 (SCL)** | SCL | I2C clock | LCD (Wire library) |

### Digital Inputs

| Arduino Pin | HW Spec Label | Function | Notes |
|---|---|---|---|
| **A0** | Arm Beams Read | KB interlock status read | Active-HIGH; driven by SparkFun opto-isolator (BCON-200) OUT1 |
| **D19 (RX1)** | UART RX | Serial1 RS-485 receive | Connected to RO on RS-485 transceiver (BCON-335) |

### Reserved

| Arduino Pin | Notes |
|---|---|
| **D0, D1** | USB-CDC; do not assign to other functions |

---

## Modbus RTU Interface

**Production settings:** 115 200 baud, 8N1, Slave ID 1, RS-485 via `Serial1` (TX1 = D18, RX1 = D19) with DE/RE direction control on **D17**. The RS-485 transceiver (MAX485-compatible, BCON-335) connects to an optical isolator (BCON-330) and exits the chassis via an isolated RJ45 panel port over Cat5E cable to the dashboard's RS-485/USB converter (BCON-ASM-630).

USB-CDC (`Serial` on D0/D1) is available for firmware development and bench testing only by setting `BCON_MODBUS_USE_USB_SERIAL 1` at build time.

**Supported function codes:**

| FC | Description |
|---|---|
| `0x03` | Read Holding Registers (1–125 registers per request) |
| `0x06` | Write Single Register |
| `0x10` | Write Multiple Registers |

Broadcast address (slave ID 0) is accepted for write commands; no response is sent for broadcasts.

---

## Modbus Register Map

> All values are 16-bit unsigned integers. Registers marked **R** are read-only; **R/W** are readable and writable.

### System Control Registers

| Address | Name | R/W | Range / Values | Description |
|---|---|---|---|---|
| 0 | `WATCHDOG_MS` | R/W | 50–60 000 | Software watchdog timeout in milliseconds. Any valid Modbus frame resets the timer. Default: 1500 ms. |
| 1 | `TELEMETRY_MS` | R/W | any | Informational: intended telemetry poll period for the host. Not used by firmware to generate autonomous frames. |
| 2 | `COMMAND` | W | see below | System-level command register. |

**COMMAND values (register 2):**

| Value | Action |
|---|---|
| `0` | NOP (also serves as a watchdog heartbeat write) |
| `1` | All channels OFF (sets all three channels to `Off` mode immediately) |
| `2` or `3` | Clear fault latch (only succeeds if no overcurrent is active and interlock is satisfied) |
| `4` | Apply all staged channel mode writes together |

---

### Channel Control Registers

One set of four registers per channel. Address formula: `10 × (ch)` where `ch` = 1, 2, 3.

| Address | CH | Name | R/W | Range / Values | Description |
|---|---|---|---|---|---|
| 10 | 1 | `CH1_MODE` | R/W | 0–3 | Staged requested mode (see mode table above). Writes are committed when `COMMAND=4` is written. |
| 11 | 1 | `CH1_PULSE_MS` | R/W | 1–60 000 | Pulse duration in milliseconds. |
| 12 | 1 | `CH1_COUNT` | R/W | 1–10 000 | Number of pulses in a train. Also used to disambiguate Pulse vs. PulseTrain when mode 2 is written. |
| 13 | 1 | `CH1_ENABLE_TOGGLE` | W | 1 | Write `1` to generate a 100 ms active-HIGH pulse on the CH1 enable-toggle output. |
| 20 | 2 | `CH2_MODE` | R/W | 0–3 | Same as CH1. |
| 21 | 2 | `CH2_PULSE_MS` | R/W | 1–60 000 | Same as CH1. |
| 22 | 2 | `CH2_COUNT` | R/W | 1–10 000 | Same as CH1. |
| 23 | 2 | `CH2_ENABLE_TOGGLE` | W | 1 | Same as CH1. |
| 30 | 3 | `CH3_MODE` | R/W | 0–3 | Same as CH1. |
| 31 | 3 | `CH3_PULSE_MS` | R/W | 1–60 000 | Same as CH1. |
| 32 | 3 | `CH3_COUNT` | R/W | 1–10 000 | Same as CH1. |
| 33 | 3 | `CH3_ENABLE_TOGGLE` | W | 1 | Same as CH1. |

> **Note on mode 2 (PULSE):** Writing `2` to a mode register uses the stored `PULSE_MS` and `COUNT` values. If `COUNT = 1`, a single pulse is fired. If `COUNT > 1`, the firmware automatically runs a pulse train identical to mode `3`.

> **Apply semantics:** Writing `0`, `1`, `2`, or `3` to `CHx_MODE` stages that request in the control register. Writing `4` to `COMMAND` commits all staged mode changes together, so selected channels can start or stop on the same firmware apply boundary. `COMMAND=1` remains the immediate all-off action.

---

### System Status Registers

| Address | Name | R/W | Values | Description |
|---|---|---|---|---|
| 100 | `SYS_STATE` | R | 0–3 | Current top-level state (0=Ready, 1=SafeInterlock, 2=SafeWatchdog, 3=FaultLatched). |
| 101 | `SYS_REASON` | R | 0–3 | Reason code matching the current state (mirrors `SYS_STATE`). |
| 102 | `FAULT_LATCHED` | R | 0 or 1 | `1` if an overcurrent fault has been latched and not yet cleared. |
| 103 | `INTERLOCK_OK` | R | 0 or 1 | `1` if the Knob Box interlock pin is asserted (system may operate). |
| 104 | `WATCHDOG_OK` | R | 0 or 1 | `1` if the software watchdog has not timed out (host is responding). |
| 105 | `LAST_ERROR` | R | see below | Last Modbus-layer error code. Cleared to `0` on next successful operation. |

**LAST_ERROR codes:**

| Code | Name | Meaning |
|---|---|---|
| 0 | `None` | No error |
| 1 | `IllegalFunction` | Unsupported function code received |
| 2 | `IllegalAddress` | Write to a read-only or undefined register |
| 3 | `IllegalValue` | Value out of allowed range |
| 4 | `DeviceFailure` | Internal device failure |
| 10 | `NotReady` | Command rejected because system is not in Ready state |
| 11 | `FaultStillActive` | Clear-fault rejected because overcurrent condition is still active |
| 12 | `InterlockNotReady` | Clear-fault rejected because interlock is not satisfied |
| 13 | `BufferOverflow` | Receive buffer overflow (frame too long) |

---

### Per-Channel Status Registers

Nine status fields per channel. Address formula: `110 + 10 × (ch − 1) + field`.

| Field | Offset | Name | Description |
|---|---|---|---|
| 0 | +0 | `mode` | Current active output mode (0=Off, 1=DC, 2=Pulse, 3=PulseTrain) |
| 1 | +1 | `pulse_ms` | Configured pulse duration in milliseconds |
| 2 | +2 | `count` | Configured pulse count |
| 3 | +3 | `remaining` | Pulses remaining in the current train (0 when idle) |
| 4 | +4 | `enable_status` | `1` if ENABLE_STATUS input is asserted (active-low pin reads HIGH through logic inversion) |
| 5 | +5 | `power_status` | `1` if POWER_STATUS input is asserted |
| 6 | +6 | `overcurrent` | `1` if OVERCURRENT_STATUS input is asserted |
| 7 | +7 | `gated` | `1` if GATED_STATUS input is asserted |
| 8 | +8 | `output_level` | `1` if the gate output pin is currently driven HIGH |

**Channel status base addresses:**

| Channel | Base Address | Fields |
|---|---|---|
| CH1 | 110 | 110–118 |
| CH2 | 120 | 120–128 |
| CH3 | 130 | 130–138 |

> **Important for batch reads:** Each channel status block occupies a stride of 10 registers, but only offsets 0–8 are implemented (no register at offset 9 of each block). Reading 29 contiguous registers starting at 110 would cross unimplemented addresses (119, 129) which return `0` without error. Each channel block should each be read as 9 registers (e.g. `read_holding_registers(110, 9)`, `read_holding_registers(120, 9)`, `read_holding_registers(130, 9)`).

---

## Typical Host Workflow

```
1. Open serial port at 115200 8N1
   → DTR asserts, Arduino resets; wait ≥ 2.5 s for firmware boot

2. Set watchdog timeout (optional):
   WRITE reg 0  ← desired timeout in ms (e.g. 1000)

3. Configure a channel (e.g. CH1, 200 ms pulse):
   WRITE reg 11 ← 200          (pulse duration)
   WRITE reg 12 ← 1            (pulse count)
   WRITE reg 10 ← 2            (mode = Pulse → fires immediately)

4. Send periodic heartbeat to keep watchdog alive:
   WRITE reg 2  ← 0  (NOP command, any reg write also works)
   → repeat within WATCHDOG_MS interval

5. Poll status:
   READ  regs 100–105           (6 system status registers)
   READ  regs 110–118           (CH1 status: 9 registers)
   READ  regs 120–128           (CH2 status)
   READ  regs 130–138           (CH3 status)

6. Turn all channels off before disconnecting:
   WRITE reg 2  ← 1            (COMMAND = all-off)
```

---

## LCD Display

The 20×4 I2C LCD (auto-detected at address `0x27` or `0x3F`) shows live system status when `BCON_ENABLE_I2C_LCD = 1`:

```
Row 0: WDG:OK INT:OK FLT:0
Row 1: CH1 <mode> O:<output> R:<remaining>
Row 2: CH2 <mode> O:<output> R:<remaining>
Row 3: CH3 <mode> O:<output> R:<remaining>
```

Mode abbreviations: `OFF`, `DC `, `PUL` (Pulse), `TRN` (PulseTrain).

The LCD shares the I2C bus. To avoid conflicts with high-speed USB serial traffic, the firmware:
- Runs I2C at 400 kHz to minimise refresh time (~20 ms per full refresh)
- Disables the TWI peripheral (`TWCR = 0`) between refresh cycles
- Defers any LCD update if a Modbus byte arrived within the last 50 ms
- Refreshes at most once per second (`LCD_REFRESH_MS = 1000`)
- Only redraws rows whose content changed (reduces I2C transactions)

---

## Build Configuration

Edit the `#define` directives near the top of the `.ino` file to change build-time options:

| Macro | Hardware Default | Description |
|---|---|---|
| `BCON_MODBUS_USE_USB_SERIAL` | **`0`** (RS-485) | `0` = RS-485 (`Serial1`, DE/RE on D17) — production hardware; `1` = USB-CDC (`Serial`) — bench testing / firmware development only |
| `BCON_ENABLE_I2C_LCD` | `1` | `1` = Enable I2C LCD; `0` = Disable (LCD code excluded at compile time) |

> **Firmware pin constants:** The `Config` namespace constants in the `.ino` file currently reflect an earlier bench-test wiring. Before programming hardware-built units, update them to match the BCON Hardware Development Specification pinout (see [Pin Assignments](#pin-assignments)): `KB_INTERLOCK_PIN` → A0; `PULSER_GATE_OUTPUT_PINS` → {A11, A13, A9}; `PULSER_ENABLE_TOGGLE_OUTPUT_PINS` → {D12, D11, D10}; `RS485_DE_RE_PIN` → D17.

---

## Dependencies

| Library | Purpose |
|---|---|
| `LiquidCrystal_I2C` | HD44780 LCD over PCF8574 I2C backpack |
| `Wire` | I2C (included in Arduino core) |
| `avr/wdt.h` | Hardware watchdog timer (included in AVR toolchain) |

The `modbus-esp8266` library in the `libs/` folder is **not used** by this firmware. Modbus RTU is implemented directly without an external library to keep the footprint small and avoid dynamic memory allocation on the ATmega2560.

---

## Test Interfaces (`test_interfaces/`)

Three Python scripts are provided for development and bench testing:

### `bcon_simple_modbus.py`
Low-level Modbus driver class (`BconModbus`). Wraps `pymodbus` with compatibility shims for different pymodbus API versions. Key methods:
- `connect(port, settle_s=2.5)` — opens port and waits for Arduino DTR-reset to complete
- `kick_watchdog()` — writes NOP to keep watchdog alive
- `set_watchdog_ms(ms)` — configure watchdog timeout
- `set_dc(channel)` / `set_off(channel)` — DC on/off
- `fire_pulse(channel, duration_ms)` — single pulse
- `read_status()` — returns `BconStatus` with all system and channel fields

### `bcon_simple_gui.py`
Minimal Tkinter GUI for quick bench testing. Connects to a single port, selects a channel (1–3), sets pulse duration and watchdog timeout, fires DC/Pulse/Off commands, and polls status every 300 ms.

### `pulser_test_gui.py`
Full-featured test dashboard (CustomTkinter, dark-mode). Features include:
- Per-channel parameter entry (duration, count, mode) with Apply button
- Synchronous start/stop across selected channels using staged `CH_MODE` writes plus `COMMAND=4`
- CSV pulse sequence player (load a `.csv` file with step definitions)
- Preset save/load (JSON)
- Raw register viewer/editor
- Session event logging to CSV
- Auto-scan of available serial ports

**Install dependencies:**
```bash
pip install -r test_interfaces/requirements.txt
# requires: pymodbus>=3.6,<4.0  customtkinter  pyserial
```

**Run the full GUI:**
```bash
cd test_interfaces
python pulser_test_gui.py
```

---

## Relationship to BCON User Story

This firmware implements the **beam on/off and pulsing** portion of the BCON User Story:

> *"The dashboard needs to tell the BCON hardware which beam should be on or off, what type of waveform and how long of a pulse if pulsed."*
> — BCON User Story, 2026-02-15

> *"The BCON subpanel should allow the user to set type of pulsing behavior, so setting it to either DC or pulsed. If pulsed, you need to be able to set a numeric value in milliseconds for how long the pulse should be. This should be per beam, because the hardware already supports setting this data per beam."*
> — BCON User Story, 2026-02-15

It also satisfies the safety requirements:

> *"BCON hardware should confirm it gets a pulse from the dashboard every now and then otherwise it will turn the beams off without a recent signal."* — Implemented as software watchdog (reg 0, default 1.5 s).

> *"A local onboard watchdog on the BCON hardware should watch for local lockups."* — Implemented as AVR 8-second hardware WDT.

> *"The Knob Box subpanel will indicate if it is blocking beams on either due to an interlock reason."* — Implemented via `KB_INTERLOCK_PIN` and `SafeInterlock` state.

**Beam deflection** (BOP Amp, waveform generation, solenoid B-field control, X-Y scan position visualization) is explicitly out of scope for this firmware and is addressed by a separate subsystem.
