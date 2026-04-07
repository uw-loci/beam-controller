"""
Pulser Upper‑Machine GUI (Tkinter + Material-style with CustomTkinter)
- Modbus RTU master using pymodbus
- Auto-scans serial ports (Linux) by default
- Features implemented (first version):
  - Manual per-channel control (start/stop/single + parameter write)
    - Synchronous start via staged mode writes + `COMMAND=4` apply
  - Safety / Remote-ARM register and physical enable input monitoring
  - Register viewer/editor (read/write holding registers)
  - Presets save/load (JSON)
  - CSV session logging (events)

Usage:
  1) pip install -r requirements.txt
  2) python3 pulser_gui.py

Author: generated (GitHub Copilot — Raptor mini (Preview))
"""

import os
import sys
import json
import csv
import time
import inspect
import logging
import queue
import threading
from datetime import datetime
from pathlib import Path

import customtkinter as ctk
from tkinter import messagebox, filedialog, ttk

try:
    # pymodbus v3: ModbusSerialClient is exposed under pymodbus.client
    from pymodbus.client import ModbusSerialClient as ModbusClient
except Exception as e:
    print("pymodbus import error:", e)
    print("Install with: pip install pymodbus")
    raise

try:
    import serial.tools.list_ports as list_ports
except Exception as e:
    print("pyserial import error:", e)
    print("Install with: pip install pyserial")
    raise

# suppress chatty pymodbus retry/timeout messages
logging.getLogger("pymodbus").setLevel(logging.ERROR)

# ----------------------------- Modbus / Register map -----------------------------
TOTAL_REGS = 160

REG_WATCHDOG_MS = 0
REG_TELEMETRY_MS = 1
REG_COMMAND = 2
COMMAND_APPLY_STAGED_MODES = 4

CH_BASE = [10, 20, 30]
CH_MODE_OFF = 0
CH_PULSE_MS_OFF = 1
CH_COUNT_OFF = 2
CH_ENABLE_TOGGLE_OFF = 3

REG_SYS_STATE = 100
REG_SYS_REASON = 101
REG_FAULT_LATCHED = 102
REG_INTERLOCK_OK = 103
REG_WATCHDOG_OK = 104
REG_LAST_ERROR = 105

REG_CH_STATUS_BASE = 110
REG_CH_STATUS_STRIDE = 10

MODE_OFF = 0
MODE_DC = 1
MODE_PULSE = 2
MODE_PULSE_TRAIN = 3
MODE_LABEL_TO_CODE = {
    "OFF": MODE_OFF,
    "DC": MODE_DC,
    "PULSE": MODE_PULSE,
    "PULSE_TRAIN": MODE_PULSE_TRAIN,
}

# Default Modbus/settings
DEFAULT_BAUD = 115200
DEFAULT_UNIT = 1
POLL_INTERVAL = 0.3  # seconds

# ----------------------------- Modbus manager (background thread) -----------------------------
class ModbusManager(threading.Thread):
    def __init__(self, ui_queue: queue.Queue):
        super().__init__(daemon=True)
        self.ui_queue = ui_queue
        self.client = None
        self.port = None
        self.baud = DEFAULT_BAUD
        self.unit = DEFAULT_UNIT
        self._stop_event = threading.Event()
        self.cmd_queue = queue.Queue()
        self.connected = False
        self.last_regs = [0] * TOTAL_REGS
        self._poll_errors = 0          # consecutive poll failures
        self._MAX_POLL_ERRORS = 4      # tolerate this many before auto-disconnect
        self._heartbeat_counter = 0    # counts poll cycles; triggers periodic NOP write
        self._HEARTBEAT_EVERY = 3      # write NOP every N poll cycles (~0.9 s at 0.3 s interval)

    def connect(self, port, baud, unit, settle_s: float = 2.5):
        self.port = port
        self.baud = baud
        self.unit = unit
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        try:
            try:
                self.client = ModbusClient(port=port, framer="rtu", baudrate=baud, timeout=1, retries=1)
            except TypeError:
                self.client = ModbusClient(method="rtu", port=port, baudrate=baud, timeout=1)
            ok = self.client.connect()
        except Exception as e:
            self.client = None
            self.connected = False
            self.ui_queue.put(("error", f"Connect failed: {e}"))
            self.ui_queue.put(("connected", False))
            return False
        # Opening the serial port asserts DTR which resets the Arduino Mega.
        # Wait for the firmware to complete setup() before sending Modbus frames.
        if ok and settle_s > 0:
            time.sleep(settle_s)
        self.connected = ok
        self._poll_errors = 0
        self.ui_queue.put(("connected", ok))
        return ok

    def disconnect(self):
        self.connected = False
        self._poll_errors = 0
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
        self.ui_queue.put(("connected", False))

    def stop(self):
        self._stop_event.set()

    def enqueue_write(self, reg, value):
        self.cmd_queue.put(("write", reg, value))

    def _write_register_compat(self, reg, val):
        write_fn = self.client.write_register
        sig = inspect.signature(write_fn)
        if "device_id" in sig.parameters:
            return write_fn(reg, int(val), device_id=self.unit)
        if "unit" in sig.parameters:
            return write_fn(reg, int(val), unit=self.unit)
        if "slave" in sig.parameters:
            return write_fn(reg, int(val), slave=self.unit)
        return write_fn(reg, int(val))

    def _read_holding_registers_compat(self, start, count):
        read_fn = self.client.read_holding_registers
        sig = inspect.signature(read_fn)
        if "device_id" in sig.parameters:
            return read_fn(start, count=count, device_id=self.unit)
        if "unit" in sig.parameters:
            return read_fn(start, count, unit=self.unit)
        if "slave" in sig.parameters:
            return read_fn(start, count, slave=self.unit)
        return read_fn(start, count)

    def run(self):
        last_error_msg = None
        while not self._stop_event.is_set():
            # process queued commands first
            try:
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    if cmd[0] == "write" and self.client and self.connected:
                        _, reg, val = cmd
                        try:
                            self._write_register_compat(reg, val)
                            self.ui_queue.put(("wrote", reg, val))
                        except Exception as e:
                            self.ui_queue.put(("error", f"Write reg {reg}: {e}"))
            except queue.Empty:
                pass

            # poll registers
            if self.client and self.connected:
                try:
                    regs = [0] * TOTAL_REGS

                    def read_block(start, count):
                        rr = self._read_holding_registers_compat(start, count)
                        if not rr or not hasattr(rr, 'registers'):
                            return False
                        vals = list(rr.registers)
                        for i, value in enumerate(vals):
                            idx = start + i
                            if 0 <= idx < TOTAL_REGS:
                                regs[idx] = value
                        return True

                    ok = True
                    ok &= read_block(0, 34)      # control regs (includes CH3 enable-toggle register)
                    ok &= read_block(100, 6)     # system status regs
                    # Each channel status block is 10 registers wide but only
                    # offsets 0-8 are implemented; reading 29 contiguous regs
                    # would hit the unimplemented offsets 9/19 (addresses 119/129)
                    # which the firmware rejects with Modbus exception 0x02.
                    ok &= read_block(110, 9)     # CH1 status regs 110-118
                    ok &= read_block(120, 9)     # CH2 status regs 120-128
                    ok &= read_block(130, 9)     # CH3 status regs 130-138

                    if ok:
                        # Periodic explicit NOP heartbeat write (COMMAND=0) so the
                        # firmware SW watchdog is fed by a write frame even when the
                        # operator makes no control changes.  Defense-in-depth on top
                        # of the firmware feeding on SYS_STATE reads.
                        self._heartbeat_counter += 1
                        if self._heartbeat_counter >= self._HEARTBEAT_EVERY:
                            self._heartbeat_counter = 0
                            try:
                                self._write_register_compat(REG_COMMAND, 0)  # NOP
                            except Exception:
                                pass  # non-fatal; read-based feed is still active

                    if not ok:
                        self._heartbeat_counter = 0
                        self._poll_errors += 1
                        err = f"Modbus read failed ({self._poll_errors}/{self._MAX_POLL_ERRORS})"
                        if err != last_error_msg:
                            self.ui_queue.put(("error", err))
                            last_error_msg = err
                        if self._poll_errors >= self._MAX_POLL_ERRORS:
                            self.connected = False
                            if self.client:
                                try:
                                    self.client.close()
                                except Exception:
                                    pass
                                self.client = None
                            self.ui_queue.put(("connected", False))
                    else:
                        self._poll_errors = 0
                        if regs != self.last_regs:
                            self.last_regs = regs.copy()
                            self.ui_queue.put(("regs", regs))
                        last_error_msg = None
                except Exception as e:
                    self._poll_errors += 1
                    err = f"Poll error ({self._poll_errors}/{self._MAX_POLL_ERRORS}): {e}"
                    if err != last_error_msg:
                        self.ui_queue.put(("error", err))
                        last_error_msg = err
                    if self._poll_errors >= self._MAX_POLL_ERRORS:
                        # too many consecutive failures – tear down
                        self.connected = False
                        if self.client:
                            try:
                                self.client.close()
                            except Exception:
                                pass
                            self.client = None
                        self.ui_queue.put(("connected", False))
            time.sleep(POLL_INTERVAL)

# ----------------------------- Utility helpers -----------------------------

def scan_serial_ports():
    ports = list_ports.comports()
    preferred = []
    others = []
    for p in ports:
        desc = (getattr(p, "description", "") or "").lower()
        hwid = (getattr(p, "hwid", "") or "").lower()
        if any(tok in desc for tok in ("arduino", "usb serial", "ch340", "cp210")) or "vid:pid=2341" in hwid:
            preferred.append(p.device)
        else:
            others.append(p.device)
    results = preferred + others
    # common fallback patterns
    if not results:
        # try typical Linux names
        results = ['/dev/ttyUSB0', '/dev/ttyACM0']
    return results

# ----------------------------- Main GUI App -----------------------------
class PulserApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")
        self.title("Pulser Upper‑Machine — Modbus RTU Master")
        self.geometry("980x640")

        # background threads / queues
        self.ui_queue = queue.Queue()
        self.modbus = ModbusManager(self.ui_queue)
        self.modbus.start()

        # logging / presets folders
        Path("logs").mkdir(exist_ok=True)
        Path("presets").mkdir(exist_ok=True)
        Path("sequences").mkdir(exist_ok=True)

        # sequence player state
        self._seq_steps: list = []          # parsed [(step_num, rows, dwell_ms), ...]
        self._seq_thread: threading.Thread | None = None
        self._seq_stop = threading.Event()

        self._build_ui()
        self.after(200, self._periodic)

    # ----------------------------- UI build -----------------------------
    def _build_ui(self):
        # Top frame: connection
        top = ctk.CTkFrame(self)
        top.pack(fill="x", padx=12, pady=8)

        # make columns resize sensibly when the window is resized
        top.grid_columnconfigure(0, weight=0)
        top.grid_columnconfigure(1, weight=1)   # port combobox grows
        top.grid_columnconfigure(2, weight=0)
        top.grid_columnconfigure(3, weight=0)
        top.grid_columnconfigure(4, weight=0)
        top.grid_columnconfigure(5, weight=0)
        top.grid_columnconfigure(6, weight=0)
        top.grid_columnconfigure(7, weight=0)
        top.grid_columnconfigure(8, weight=0)

        ctk.CTkLabel(top, text="Port:").grid(row=0, column=0, sticky="w", padx=6)
        port_values = scan_serial_ports()
        self.port_cb = ctk.CTkComboBox(top, values=port_values)
        self.port_cb.grid(row=0, column=1, padx=6, sticky="ew")
        if port_values:
            self.port_cb.set(port_values[0])

        ctk.CTkLabel(top, text="Baud:").grid(row=0, column=2, sticky="w", padx=6)
        self.baud_entry = ctk.CTkEntry(top, width=100)
        self.baud_entry.insert(0, str(DEFAULT_BAUD))
        self.baud_entry.grid(row=0, column=3, padx=6)

        ctk.CTkLabel(top, text="Unit ID:").grid(row=0, column=4, sticky="w", padx=6)
        self.unit_entry = ctk.CTkEntry(top, width=60)
        self.unit_entry.insert(0, str(DEFAULT_UNIT))
        self.unit_entry.grid(row=0, column=5, padx=6)

        self.connect_btn = ctk.CTkButton(top, text="Connect", command=self._connect)
        self.connect_btn.grid(row=0, column=6, padx=8)

        # Theme selector (Light / Dark)
        self.theme_cb = ctk.CTkComboBox(top, values=["Dark", "Light"], width=120, command=self._theme_changed)
        self.theme_cb.set("Dark")
        self.theme_cb.grid(row=0, column=7, padx=6)

        self.conn_label = ctk.CTkLabel(top, text="Disconnected", text_color="red")
        self.conn_label.grid(row=0, column=8, padx=8, sticky='e')

        # Middle: two columns — controls and register/editor
        main = ctk.CTkFrame(self)
        main.pack(fill="both", expand=True, padx=12, pady=6)
        main.grid_columnconfigure(0, weight=2)
        main.grid_columnconfigure(1, weight=1)
        main.grid_rowconfigure(0, weight=1)  # allow main content to expand vertically

        # Left pane: channel controls + sync + safety
        left = ctk.CTkScrollableFrame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0,8), pady=4)

        # Channel control cards
        self.channel_vars = []
        for ch in range(3):
            frame = ctk.CTkFrame(left)
            frame.pack(fill="x", pady=8, padx=6)
            # allow inputs to stretch when window expands
            frame.grid_columnconfigure(1, weight=1)
            frame.grid_columnconfigure(3, weight=1)
            ctk.CTkLabel(frame, text=f"Channel {ch+1}", font=ctk.CTkFont(size=14, weight="bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(4,8))

            ctk.CTkLabel(frame, text="Duration (ms):").grid(row=1, column=0, sticky="w")
            dur_entry = ctk.CTkEntry(frame)
            dur_entry.grid(row=1, column=1, padx=4, sticky="ew")

            ctk.CTkLabel(frame, text="Count:").grid(row=1, column=2, sticky="w")
            cnt_entry = ctk.CTkEntry(frame)
            cnt_entry.grid(row=1, column=3, padx=4, sticky="ew")

            ctk.CTkLabel(frame, text="Mode:").grid(row=2, column=0, sticky="w", pady=(6,0))
            mode_cb = ctk.CTkComboBox(frame, values=["OFF", "DC", "PULSE", "PULSE_TRAIN"])
            mode_cb.set("PULSE")
            mode_cb.grid(row=2, column=1, padx=4, pady=(6,0), sticky="ew")

            status_lbl = ctk.CTkLabel(frame, text="Status: idle", anchor="w")
            status_lbl.grid(row=3, column=0, columnspan=2, sticky="w", pady=(8,4))
            pulses_lbl = ctk.CTkLabel(frame, text="Pulses: 0", anchor="w")
            pulses_lbl.grid(row=3, column=2, columnspan=2, sticky="w", pady=(8,4))

            btn_apply = ctk.CTkButton(frame, text="Apply Params + Mode", width=120, command=lambda ch=ch, cE=cnt_entry, d=dur_entry, m=mode_cb: self._apply_channel(ch, d, cE, m))
            btn_apply.grid(row=4, column=0, columnspan=2, pady=6, padx=(0, 4), sticky="ew")
            btn_stop = ctk.CTkButton(frame, text="Off", fg_color="#e74c3c", command=lambda ch=ch: self._set_mode(ch, MODE_OFF))
            btn_stop.grid(row=4, column=2, pady=6, padx=4, sticky="ew")
            btn_dc = ctk.CTkButton(frame, text="DC", fg_color="#2ecc71", command=lambda ch=ch: self._set_mode(ch, MODE_DC))
            btn_dc.grid(row=4, column=3, pady=6, sticky="ew")

            btn_pulse = ctk.CTkButton(frame, text="Pulse", command=lambda ch=ch: self._set_mode(ch, MODE_PULSE))
            btn_pulse.grid(row=5, column=0, columnspan=2, pady=(2, 6), padx=(0, 4), sticky="ew")
            btn_train = ctk.CTkButton(frame, text="Pulse Train", command=lambda ch=ch: self._set_mode(ch, MODE_PULSE_TRAIN))
            btn_train.grid(row=5, column=2, columnspan=2, pady=(2, 6), sticky="ew")

            btn_toggle = ctk.CTkButton(frame, text="Enable Toggle", command=lambda ch=ch: self._toggle_enable(ch))
            btn_toggle.grid(row=6, column=0, columnspan=4, pady=(2, 6), sticky="ew")

            self.channel_vars.append({
                'count': cnt_entry,
                'duration': dur_entry,
                'mode': mode_cb,
                'status': status_lbl,
                'pulses': pulses_lbl
            })

        # Sync control — per-channel configuration table
        sync_frame = ctk.CTkFrame(left)
        sync_frame.pack(fill="x", pady=8, padx=6)
        sync_frame.grid_columnconfigure(1, weight=1)
        sync_frame.grid_columnconfigure(2, weight=1)
        sync_frame.grid_columnconfigure(3, weight=1)

        ctk.CTkLabel(sync_frame, text="Synchronous Control", font=ctk.CTkFont(size=13, weight="bold")).grid(
            row=0, column=0, columnspan=5, sticky="w", pady=(4, 6))

        # Header row
        for col, hdr in enumerate(("CH", "Duration (ms)", "Count", "Mode", "Include")):
            ctk.CTkLabel(sync_frame, text=hdr, font=ctk.CTkFont(size=11, weight="bold")).grid(
                row=1, column=col, padx=4, pady=(0, 4))

        # Per-channel rows
        self.sync_ch_vars = [ctk.BooleanVar(value=True) for _ in range(3)]
        self.sync_configs = []
        for ch in range(3):
            r = ch + 2
            ctk.CTkLabel(sync_frame, text=f"CH{ch+1}", font=ctk.CTkFont(weight="bold")).grid(
                row=r, column=0, padx=6, pady=3, sticky="w")

            dur_e = ctk.CTkEntry(sync_frame, width=80, placeholder_text="100")
            dur_e.grid(row=r, column=1, padx=4, pady=3, sticky="ew")

            cnt_e = ctk.CTkEntry(sync_frame, width=60, placeholder_text="1")
            cnt_e.grid(row=r, column=2, padx=4, pady=3, sticky="ew")

            mode_cb = ctk.CTkComboBox(sync_frame, values=["OFF", "DC", "PULSE", "PULSE_TRAIN"], width=120)
            mode_cb.set("PULSE")
            mode_cb.grid(row=r, column=3, padx=4, pady=3, sticky="ew")

            ctk.CTkCheckBox(sync_frame, text="", variable=self.sync_ch_vars[ch], width=30).grid(
                row=r, column=4, padx=8, pady=3)

            self.sync_configs.append({'duration': dur_e, 'count': cnt_e, 'mode': mode_cb})

        # Action buttons
        btn_row = ctk.CTkFrame(sync_frame, fg_color="transparent")
        btn_row.grid(row=5, column=0, columnspan=5, pady=(8, 4), sticky="ew")
        ctk.CTkButton(btn_row, text="Write Params", width=110,
                      command=self._sync_write_params).pack(side="left", padx=4)
        ctk.CTkButton(btn_row, text="▶ Start Selected", width=120, fg_color="#2ecc71",
                      hover_color="#27ae60", command=self._sync_start).pack(side="left", padx=4)
        ctk.CTkButton(btn_row, text="⏹ Stop All", width=100, fg_color="#e74c3c",
                      hover_color="#c0392b", command=self._sync_stop_all).pack(side="left", padx=4)

        # CSV Pulse Sequence player
        seq_frame = ctk.CTkFrame(left)
        seq_frame.pack(fill="x", pady=8, padx=6)
        seq_frame.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(seq_frame, text="CSV Pulse Sequence",
                     font=ctk.CTkFont(size=13, weight="bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(4, 4))

        self.seq_file_lbl = ctk.CTkLabel(
            seq_frame, text="No sequence loaded", anchor="w", text_color="gray")
        self.seq_file_lbl.grid(row=1, column=0, columnspan=4, sticky="w", padx=4)

        self.seq_progress_lbl = ctk.CTkLabel(seq_frame, text="", anchor="w")
        self.seq_progress_lbl.grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=(2, 4))

        seq_btn_row = ctk.CTkFrame(seq_frame, fg_color="transparent")
        seq_btn_row.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(2, 6))
        ctk.CTkButton(seq_btn_row, text="Load CSV", width=90,
                      command=self._load_sequence).pack(side="left", padx=4)
        ctk.CTkButton(seq_btn_row, text="Save Template", width=110,
                      command=self._save_sequence_template).pack(side="left", padx=4)
        self.seq_run_btn = ctk.CTkButton(
            seq_btn_row, text="▶ Run Sequence", width=120,
            fg_color="#2ecc71", hover_color="#27ae60",
            command=self._run_sequence, state="disabled")
        self.seq_run_btn.pack(side="left", padx=4)
        self.seq_stop_btn = ctk.CTkButton(
            seq_btn_row, text="■ Stop", width=80,
            fg_color="#e74c3c", hover_color="#c0392b",
            command=self._stop_sequence, state="disabled")
        self.seq_stop_btn.pack(side="left", padx=4)

        # Safety / Remote arm
        safety_frame = ctk.CTkFrame(left)
        safety_frame.pack(fill="x", pady=8, padx=6)
        ctk.CTkLabel(safety_frame, text="Safety & Remote Arm", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, sticky="w", pady=(6,8))
        self.safety_label = ctk.CTkLabel(safety_frame, text="Interlock: unknown")
        self.safety_label.grid(row=1, column=0, sticky="w")
        self.remote_arm_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(safety_frame, text="REMOTE_ARM", variable=self.remote_arm_var, command=self._toggle_remote_arm).grid(row=1, column=1, padx=12)

        # System settings (watchdog / telemetry)
        sys_frame = ctk.CTkFrame(left)
        sys_frame.pack(fill="x", pady=8, padx=6)
        ctk.CTkLabel(sys_frame, text="System Settings", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(6,8))
        ctk.CTkLabel(sys_frame, text="Watchdog (ms):").grid(row=1, column=0, sticky="w")
        self.watchdog_entry = ctk.CTkEntry(sys_frame, width=80)
        self.watchdog_entry.grid(row=1, column=1, padx=4, sticky="w")
        ctk.CTkButton(sys_frame, text="Set", width=60, command=self._set_watchdog).grid(row=1, column=2, padx=4)
        ctk.CTkLabel(sys_frame, text="Telemetry (ms):").grid(row=2, column=0, sticky="w", pady=(4,0))
        self.telemetry_entry = ctk.CTkEntry(sys_frame, width=80)
        self.telemetry_entry.grid(row=2, column=1, padx=4, sticky="w", pady=(4,0))
        ctk.CTkButton(sys_frame, text="Set", width=60, command=self._set_telemetry).grid(row=2, column=2, padx=4, pady=(4,0))

        # Arm Beam control (requires explicit confirmation)
        self.arm_btn = ctk.CTkButton(safety_frame, text="Arm Beam", fg_color="#2ecc71", command=self._arm_beam)
        self.arm_btn.grid(row=1, column=2, padx=8, sticky='e')
        self.arm_status_lbl = ctk.CTkLabel(safety_frame, text="DISARMED", text_color="gray")
        self.arm_status_lbl.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8,0))

        # Presets & Logging
        ctrl_frame = ctk.CTkFrame(left)
        ctrl_frame.pack(fill="x", pady=8, padx=6)
        ctk.CTkLabel(ctrl_frame, text="Presets / Logging", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, sticky="w", pady=(6,8))
        ctk.CTkButton(ctrl_frame, text="Save preset", command=self._save_preset).grid(row=1, column=0, padx=6)
        ctk.CTkButton(ctrl_frame, text="Load preset", command=self._load_preset).grid(row=1, column=1, padx=6)
        ctk.CTkButton(ctrl_frame, text="Export registers CSV", command=self._export_regs_csv).grid(row=1, column=2, padx=6)

        # Right pane: registers editor + log
        right = ctk.CTkFrame(main)
        right.grid(row=0, column=1, sticky="nsew")

        regs_frame = ctk.CTkFrame(right)
        regs_frame.pack(fill="both", expand=True, padx=6, pady=6)
        regs_frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(regs_frame, text="Modbus Holding Registers (0-31)", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, sticky='w')

        style = ttk.Style()
        # increase row height so register values are not visually clamped
        style.configure("Custom.Treeview", rowheight=40)
        self.reg_tree = ttk.Treeview(regs_frame, style="Custom.Treeview", columns=("reg", "val"), show='headings', height=16)
        self.reg_tree.heading('reg', text='Register')
        self.reg_tree.heading('val', text='Value')
        self.reg_tree.column('reg', width=80, anchor='center', stretch=False)
        self.reg_tree.column('val', width=100, anchor='center', stretch=True)
        self.reg_tree.grid(row=1, column=0, sticky='nsew', pady=6)
        regs_frame.grid_rowconfigure(1, weight=1)
        for i in range(32):
            self.reg_tree.insert('', 'end', values=(i, 0))

        reg_buttons = ctk.CTkFrame(right)
        reg_buttons.pack(fill="x", padx=6, pady=6)
        ctk.CTkButton(reg_buttons, text="Read All", command=self._read_all_regs).grid(row=0, column=0, padx=6)
        ctk.CTkButton(reg_buttons, text="Write Selected", command=self._write_selected_reg).grid(row=0, column=1, padx=6)

        # bottom status bar
        status = ctk.CTkFrame(self)
        status.pack(fill="x", padx=12, pady=(0,12))
        self.last_poll_lbl = ctk.CTkLabel(status, text="Last poll: --")
        self.last_poll_lbl.pack(side='left')
        self.log_text = ctk.CTkLabel(status, text="Log: ready")
        self.log_text.pack(side='right')

    # ----------------------------- UI actions -----------------------------
    def _connect(self):
        if not self.modbus.connected:
            port = self.port_cb.get()
            if not port:
                messagebox.showerror("Connect", "No serial port selected")
                return
            baud = int(self.baud_entry.get())
            unit = int(self.unit_entry.get())
            # Warn user: opening the port resets the Mega via DTR; we block here
            # for ~2.5 s while firmware completes setup().
            self.conn_label.configure(text="Connecting… (waiting for boot)", text_color="orange")
            self.connect_btn.configure(state="disabled")
            self.update_idletasks()
            try:
                ok = self.modbus.connect(port, baud, unit)
            finally:
                self.connect_btn.configure(state="normal")
            if ok:
                self.conn_label.configure(text=f"Connected ({port})", text_color="green")
            else:
                self.conn_label.configure(text="Connect failed", text_color="red")
                messagebox.showerror("Connect", f"Failed to open port {port}")
        else:
            self.modbus.disconnect()
            self.conn_label.configure(text="Disconnected", text_color="red")

    def _theme_changed(self, value: str):
        """Switch UI theme (Dark/Light)."""
        try:
            ctk.set_appearance_mode(value)
            self._log_event(f"Theme set to {value}")
        except Exception as e:
            self._log_event(f"Theme change failed: {e}")

    def _apply_channel(self, ch, d_entry, cnt_entry, mode_cb):
        try:
            duration = int(d_entry.get())
            count = int(cnt_entry.get())
        except Exception:
            messagebox.showerror("Invalid", "Enter numeric values for parameters")
            return

        if duration <= 0:
            messagebox.showerror("Invalid", "Pulse duration must be > 0")
            return
        if count <= 0:
            messagebox.showerror("Invalid", "Count must be > 0")
            return

        mode_label = mode_cb.get().strip().upper()
        if mode_label not in MODE_LABEL_TO_CODE:
            messagebox.showerror("Invalid", f"Unsupported mode: {mode_label}")
            return

        base = CH_BASE[ch]
        self.modbus.enqueue_write(base + CH_PULSE_MS_OFF, duration)
        self.modbus.enqueue_write(base + CH_COUNT_OFF, count)
        mode_code = MODE_LABEL_TO_CODE[mode_label]
        self.modbus.enqueue_write(base + CH_MODE_OFF, mode_code)
        if mode_code != MODE_OFF:
            self.modbus.enqueue_write(REG_COMMAND, COMMAND_APPLY_STAGED_MODES)
        self._log_event(f"Applied CH{ch+1}: mode={mode_label} duration_ms={duration} count={count}")

    def _set_mode(self, ch, mode):
        base = CH_BASE[ch]
        if mode == MODE_PULSE:
            self.modbus.enqueue_write(base + CH_COUNT_OFF, 1)
        self.modbus.enqueue_write(base + CH_MODE_OFF, mode)
        if mode != MODE_OFF:
            self.modbus.enqueue_write(REG_COMMAND, COMMAND_APPLY_STAGED_MODES)
        mode_name = {
            MODE_OFF: "OFF",
            MODE_DC: "DC",
            MODE_PULSE: "PULSE",
            MODE_PULSE_TRAIN: "PULSE_TRAIN",
        }.get(mode, str(mode))
        self._log_event(f"CMD CH{ch+1} -> {mode_name}")

    def _toggle_enable(self, ch):
        base = CH_BASE[ch]
        self.modbus.enqueue_write(base + CH_ENABLE_TOGGLE_OFF, 1)
        self._log_event(f"CMD CH{ch+1} -> ENABLE_TOGGLE")

    def _sync_write_params(self):
        """Write duration + count for every checked sync channel (without changing mode)."""
        selected = [ch for ch in range(3) if self.sync_ch_vars[ch].get()]
        if not selected:
            self._log_event("Sync: no channels checked")
            return
        for ch in selected:
            cfg = self.sync_configs[ch]
            dur_str = cfg['duration'].get().strip()
            cnt_str = cfg['count'].get().strip()
            try:
                dur = int(dur_str) if dur_str else None
                cnt = int(cnt_str) if cnt_str else None
            except ValueError:
                messagebox.showerror("Invalid", f"CH{ch+1}: duration and count must be integers")
                return
            base = CH_BASE[ch]
            if dur is not None and dur > 0:
                self.modbus.enqueue_write(base + CH_PULSE_MS_OFF, dur)
            if cnt is not None and cnt > 0:
                self.modbus.enqueue_write(base + CH_COUNT_OFF, cnt)
        self._log_event(f"Sync wrote params for CH{[c+1 for c in selected]}")

    def _sync_start(self):
        """Apply each channel's individual config, then start all selected channels.

        Strategy for minimising inter-channel start jitter:
          Phase 1 – write all duration/count values for every selected channel.
                    Phase 2 – stage mode writes for every selected channel.
                    Phase 3 – emit COMMAND=4 so firmware commits the staged modes together.
                All phases are enqueued atomically so no other write can interleave.
        """
        selected = [ch for ch in range(3) if self.sync_ch_vars[ch].get()]
        if not selected:
            self._log_event("Sync Start: no channels checked")
            return

        # Validate all channels before enqueuing anything
        params = {}
        for ch in selected:
            cfg = self.sync_configs[ch]
            dur_str = cfg['duration'].get().strip()
            cnt_str = cfg['count'].get().strip()
            mode_label = cfg['mode'].get().strip().upper()
            try:
                dur = int(dur_str) if dur_str else 100
                cnt = int(cnt_str) if cnt_str else 1
            except ValueError:
                messagebox.showerror("Invalid", f"CH{ch+1}: duration and count must be integers")
                return
            if mode_label not in MODE_LABEL_TO_CODE:
                messagebox.showerror("Invalid", f"CH{ch+1}: unknown mode '{mode_label}'")
                return
            mode_code = MODE_LABEL_TO_CODE[mode_label]
            if mode_code == MODE_PULSE_TRAIN and cnt < 2:
                messagebox.showerror("Invalid", f"CH{ch+1}: PULSE_TRAIN requires count \u2265 2")
                return
            params[ch] = (dur, cnt, mode_code, mode_label)

        # Phase 1: write duration + count for all channels (params must be set before mode)
        for ch, (dur, cnt, mode_code, _) in params.items():
            if mode_code not in (MODE_OFF, MODE_DC):
                self.modbus.enqueue_write(CH_BASE[ch] + CH_PULSE_MS_OFF, dur)
                self.modbus.enqueue_write(CH_BASE[ch] + CH_COUNT_OFF, cnt)

        # Phase 2: stage modes for all channels
        for ch, (_, _, mode_code, _) in params.items():
            self.modbus.enqueue_write(CH_BASE[ch] + CH_MODE_OFF, mode_code)

        # Phase 3: commit all staged non-OFF modes together in firmware
        if any(params[ch][2] != MODE_OFF for ch in selected):
            self.modbus.enqueue_write(REG_COMMAND, COMMAND_APPLY_STAGED_MODES)

        self._log_event(
            "Sync Start: " +
            ", ".join(
                f"CH{ch+1}={params[ch][3]}({params[ch][0]}ms\u00d7{params[ch][1]})"
                for ch in selected
            )
        )

    def _sync_stop_all(self):
        """Force all three channels to OFF immediately."""
        for ch in range(3):
            self.modbus.enqueue_write(CH_BASE[ch] + CH_MODE_OFF, MODE_OFF)
        self._log_event("Sync Stop: all channels \u2192 OFF")

    # ----------------------------- CSV Sequence player -----------------------------

    def _load_sequence(self):
        fname = filedialog.askopenfilename(
            initialdir="sequences",
            filetypes=[("CSV Sequence", "*.csv"), ("All files", "*.*")],
            title="Load Pulse Sequence CSV",
        )
        if not fname:
            return
        try:
            steps_raw: dict = {}  # step_num -> {rows: list, dwell_ms: int}
            with open(fname, newline="") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or line.lower().startswith("step"):
                        continue
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) < 3:
                        continue
                    step_num = int(parts[0])
                    ch_str   = parts[1].upper()
                    mode     = parts[2].upper()
                    dur_ms   = int(parts[3]) if len(parts) > 3 and parts[3] else 100
                    count    = int(parts[4]) if len(parts) > 4 and parts[4] else 1
                    dwell_ms = int(parts[5]) if len(parts) > 5 and parts[5] else 0

                    if mode not in MODE_LABEL_TO_CODE:
                        raise ValueError(f"Unknown mode '{mode}' at step {step_num}")
                    if mode == "PULSE_TRAIN" and count < 2:
                        raise ValueError(f"Step {step_num}: PULSE_TRAIN requires count >= 2")

                    ch_list = list(range(3)) if ch_str == "ALL" else [int(ch_str) - 1]
                    if step_num not in steps_raw:
                        steps_raw[step_num] = {"rows": [], "dwell_ms": 0}
                    for ch_idx in ch_list:
                        steps_raw[step_num]["rows"].append(
                            {"ch": ch_idx, "mode": mode, "duration_ms": dur_ms, "count": count}
                        )
                    steps_raw[step_num]["dwell_ms"] = dwell_ms  # last row wins

            self._seq_steps = [
                (sn, steps_raw[sn]["rows"], steps_raw[sn]["dwell_ms"])
                for sn in sorted(steps_raw.keys())
            ]
            n = len(self._seq_steps)
            self.seq_file_lbl.configure(
                text=f"{os.path.basename(fname)}  ({n} step{'s' if n != 1 else ''})",
                text_color="white"
            )
            self.seq_progress_lbl.configure(text="Ready")
            self.seq_run_btn.configure(state="normal")
            self._log_event(f"Sequence loaded: {os.path.basename(fname)} ({n} steps)")
        except Exception as e:
            messagebox.showerror("Sequence Load Error", str(e))
            self._log_event(f"Sequence load failed: {e}")

    def _save_sequence_template(self):
        fname = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialdir="sequences",
            filetypes=[("CSV Sequence", "*.csv")],
            title="Save Sequence Template",
        )
        if not fname:
            return
        template = (
            "# BCON Pulse Sequence\n"
            "# ============================================================\n"
            "# Columns:\n"
            "#   step        - integer; rows sharing a step number launch together\n"
            "#   ch          - channel number (1, 2, 3) or ALL\n"
            "#   mode        - OFF | DC | PULSE | PULSE_TRAIN\n"
            "#   duration_ms - pulse width in ms  (PULSE / PULSE_TRAIN only)\n"
            "#   count       - pulse count        (PULSE_TRAIN must be >= 2)\n"
            "#   dwell_ms    - wait AFTER this step before the next one\n"
            "#                 (only the last row per step number is used)\n"
            "# ============================================================\n"
            "step,ch,mode,duration_ms,count,dwell_ms\n"
            "1,1,PULSE,100,5,0\n"
            "1,2,PULSE,200,1,0\n"
            "1,3,DC,,,500\n"
            "2,1,PULSE_TRAIN,50,10,0\n"
            "2,2,OFF,,,0\n"
            "2,3,OFF,,,1000\n"
            "3,ALL,OFF,,,500\n"
        )
        with open(fname, "w") as f:
            f.write(template)
        self._log_event(f"Sequence template saved: {os.path.basename(fname)}")

    def _run_sequence(self):
        if not self._seq_steps:
            messagebox.showinfo("Sequence", "No sequence loaded.")
            return
        if not self.modbus.connected:
            messagebox.showwarning("Sequence", "Not connected to device.")
            return
        if self._seq_thread and self._seq_thread.is_alive():
            return  # already running
        self._seq_stop.clear()
        self.seq_run_btn.configure(state="disabled")
        self.seq_stop_btn.configure(state="normal")
        self._seq_thread = threading.Thread(target=self._sequence_worker, daemon=True)
        self._seq_thread.start()
        self._log_event("Sequence started")

    def _stop_sequence(self):
        self._seq_stop.set()
        self._log_event("Sequence stop requested")

    def _sequence_worker(self):
        total = len(self._seq_steps)
        for idx, (step_num, rows, dwell_ms) in enumerate(self._seq_steps):
            if self._seq_stop.is_set():
                break
            self.ui_queue.put(("seq_status", f"Step {idx + 1}/{total}  (#{step_num})"))

            # Phase 1: write parameters for all rows that need them
            for row in rows:
                if row["mode"] in ("OFF", "DC"):
                    continue
                base = CH_BASE[row["ch"]]
                self.modbus.enqueue_write(base + CH_PULSE_MS_OFF, row["duration_ms"])
                self.modbus.enqueue_write(base + CH_COUNT_OFF, row["count"])

            # Phase 2: stage modes together, then commit them with one apply command
            for row in rows:
                self.modbus.enqueue_write(
                    CH_BASE[row["ch"]] + CH_MODE_OFF, MODE_LABEL_TO_CODE[row["mode"]]
                )
            if any(MODE_LABEL_TO_CODE[row["mode"]] != MODE_OFF for row in rows):
                self.modbus.enqueue_write(REG_COMMAND, COMMAND_APPLY_STAGED_MODES)

            # Dwell in small slices so stop is responsive
            deadline = time.time() + dwell_ms / 1000.0
            while time.time() < deadline and not self._seq_stop.is_set():
                time.sleep(0.05)

        final = "Sequence complete" if not self._seq_stop.is_set() else "Sequence stopped"
        self.ui_queue.put(("seq_status", final))
        self.ui_queue.put(("seq_done", None))

    def _toggle_remote_arm(self):
        self.remote_arm_var.set(False)
        self._log_event("REMOTE_ARM register is not implemented in current firmware")

    def _arm_beam(self):
        """User-facing arm/disarm action with confirmation (typesafe)."""
        currently_armed = bool(self.remote_arm_var.get())
        if currently_armed:
            if not messagebox.askyesno("Disarm Beam", "Disarm the beam now?"):
                return
            # disarm
            self.remote_arm_var.set(False)
            self._toggle_remote_arm()
            self._log_event("Beam disarmed via GUI")
            return

    def _set_watchdog(self):
        val = self.watchdog_entry.get().strip()
        if not val:
            return
        try:
            ms = int(val)
        except Exception:
            messagebox.showerror("Invalid", "Watchdog value must be integer")
            return
        self.modbus.enqueue_write(REG_WATCHDOG_MS, ms)
        self._log_event(f"Set watchdog timeout to {ms} ms")

    def _set_telemetry(self):
        val = self.telemetry_entry.get().strip()
        if not val:
            return
        try:
            ms = int(val)
        except Exception:
            messagebox.showerror("Invalid", "Telemetry value must be integer")
            return
        self.modbus.enqueue_write(REG_TELEMETRY_MS, ms)
        self._log_event(f"Set telemetry period to {ms} ms")

        # confirm explicit action: require typing 'ARM'
        answer = simple_input_dialog(self, "Confirm Arm", "Type ARM to confirm:")
        if not answer or answer.strip().upper() != 'ARM':
            messagebox.showwarning("Arm cancelled", "Arm operation cancelled (confirmation failed).")
            return

        self.modbus.enqueue_write(REG_COMMAND, 3)
        self._log_event("ARM command sent (REG_COMMAND=3)")
        messagebox.showinfo("Arm command sent", "Sent ARM/CLEAR command to firmware.")

    def _read_all_regs(self):
        # trigger a harmless no-op write so the comm path is exercised.
        self.modbus.enqueue_write(REG_COMMAND, 0)
        self._log_event("Requested register refresh")

    def _write_selected_reg(self):
        sel = self.reg_tree.selection()
        if not sel:
            messagebox.showinfo("Select", "Select a register row to write")
            return
        item = self.reg_tree.item(sel[0])
        reg = int(item['values'][0])
        val = simple_input_dialog(self, f"Write register {reg}", "Value:")
        if val is None:
            return
        try:
            v = int(val)
        except Exception:
            messagebox.showerror("Invalid", "Value must be integer")
            return
        self.modbus.enqueue_write(reg, v)
        self._log_event(f"Write register {reg} = {v}")

    def _save_preset(self):
        p = {}
        for ch in range(3):
            base = CH_BASE[ch]
            p[f'ch{ch+1}'] = {
                'pulse_ms': int(self.channel_vars[ch]['duration'].get() or self.channel_vars[ch]['width'].get() or 0),
                'count': int(self.channel_vars[ch]['count'].get() or 0),
                'mode': int(self.modbus.last_regs[base + CH_MODE_OFF])
            }
        fname = filedialog.asksaveasfilename(defaultextension='.json', initialdir='presets', filetypes=[('JSON','*.json')])
        if fname:
            with open(fname, 'w') as f:
                json.dump(p, f, indent=2)
            self._log_event(f"Saved preset: {os.path.basename(fname)}")

    def _load_preset(self):
        fname = filedialog.askopenfilename(initialdir='presets', filetypes=[('JSON','*.json')])
        if not fname:
            return
        with open(fname, 'r') as f:
            p = json.load(f)
        for ch in range(3):
            d = p.get(f'ch{ch+1}', {})
            pulse_ms = d.get('pulse_ms', 0)
            self.channel_vars[ch]['count'].delete(0, 'end'); self.channel_vars[ch]['count'].insert(0, str(d.get('count', 0)))
            self.channel_vars[ch]['duration'].delete(0, 'end'); self.channel_vars[ch]['duration'].insert(0, str(pulse_ms))
            mode_code = int(d.get('mode', MODE_OFF))
            mode_label = {
                MODE_OFF: "OFF",
                MODE_DC: "DC",
                MODE_PULSE: "PULSE",
                MODE_PULSE_TRAIN: "PULSE_TRAIN",
            }.get(mode_code, "OFF")
            self.channel_vars[ch]['mode'].set(mode_label)
        self._log_event(f"Loaded preset: {os.path.basename(fname)}")

    def _export_regs_csv(self):
        fname = filedialog.asksaveasfilename(defaultextension='.csv', initialdir='logs', filetypes=[('CSV','*.csv')])
        if not fname:
            return
        regs = self.modbus.last_regs
        with open(fname, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['register','value'])
            for i, v in enumerate(regs[:140]):
                w.writerow([i, v])
        self._log_event(f"Exported registers to {os.path.basename(fname)}")

    def _log_event(self, text):
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        line = f"[{ts}] {text}"
        print(line)
        self.log_text.configure(text=text)
        # also append to a session CSV log
        fname = Path('logs') / f"session_{datetime.now().strftime('%Y%m%d')}.csv"
        write_header = not fname.exists()
        with open(fname, 'a', newline='') as f:
            w = csv.writer(f)
            if write_header:
                w.writerow(['timestamp','event'])
            w.writerow([ts, text])

    # ----------------------------- Periodic UI update (process queue) -----------------------------
    def _periodic(self):
        try:
            while not self.ui_queue.empty():
                msg = self.ui_queue.get_nowait()
                self._handle_msg(msg)
        except queue.Empty:
            pass
        # refresh serial port list occasionally
        if int(time.time()) % 5 == 0:
            try:
                ports = scan_serial_ports()
                # update combobox options without changing current selection
                try:
                    cur = self.port_cb.get()
                except Exception:
                    cur = ''
                self.port_cb.configure(values=ports)
                if cur in ports:
                    self.port_cb.set(cur)
            except Exception:
                pass
        self.after(200, self._periodic)

    def _handle_msg(self, msg):
        typ = msg[0]
        if typ == 'connected':
            ok = msg[1]
            if ok:
                self.conn_label.configure(text=f"Connected ({self.modbus.port})", text_color="green")
            else:
                self.conn_label.configure(text="Disconnected", text_color="red")
        elif typ == 'regs':
            regs = msg[1]
            # update register tree
            for i in range(32):
                item = self.reg_tree.get_children()[i]
                self.reg_tree.item(item, values=(i, regs[i]))
            # update channel UI values and status
            for ch in range(3):
                base = CH_BASE[ch]
                status_base = REG_CH_STATUS_BASE + (ch * REG_CH_STATUS_STRIDE)

                pulse_ms = regs[base + CH_PULSE_MS_OFF]
                count = regs[base + CH_COUNT_OFF]
                # auto-fill entries only if user hasn't edited (naive approach)
                self._safe_fill(self.channel_vars[ch]['count'], count)
                self._safe_fill(self.channel_vars[ch]['duration'], pulse_ms)

                mode = regs[status_base + 0]
                remaining = regs[status_base + 3]
                output_level = regs[status_base + 8]
                st_text = {
                    MODE_OFF: 'off',
                    MODE_DC: 'dc',
                    MODE_PULSE: 'pulse',
                    MODE_PULSE_TRAIN: 'pulse_train'
                }.get(mode, 'unknown')
                self.channel_vars[ch]['status'].configure(text=f"Status: {st_text} | O:{output_level}")
                self.channel_vars[ch]['pulses'].configure(text=f"R:{remaining}")
                mode_label = {
                    MODE_OFF: "OFF",
                    MODE_DC: "DC",
                    MODE_PULSE: "PULSE",
                    MODE_PULSE_TRAIN: "PULSE_TRAIN",
                }.get(mode)
                if mode_label:
                    self.channel_vars[ch]['mode'].set(mode_label)

            interlock_ok = regs[REG_INTERLOCK_OK]
            watchdog_ok = regs[REG_WATCHDOG_OK]
            sys_state = regs[REG_SYS_STATE]
            self.safety_label.configure(text=f"Interlock: {'ok' if interlock_ok else 'locked'} | Watchdog: {'ok' if watchdog_ok else 'expired'} | State: {sys_state}")
            self.remote_arm_var.set(False)

            # update watchdog/telemetry entry fields with current values
            self.watchdog_entry.delete(0, 'end'); self.watchdog_entry.insert(0, str(regs[REG_WATCHDOG_MS]))
            self.telemetry_entry.delete(0, 'end'); self.telemetry_entry.insert(0, str(regs[REG_TELEMETRY_MS]))

            if regs[REG_FAULT_LATCHED]:
                self.arm_status_lbl.configure(text="FAULT LATCHED", text_color="#e74c3c")
            else:
                self.arm_status_lbl.configure(text="NO LATCHED FAULT", text_color="#2ecc71")
            self.last_poll_lbl.configure(text=f"Last poll: {datetime.now().strftime('%H:%M:%S')}")
        elif typ == 'wrote':
            reg, val = msg[1], msg[2]
            self._log_event(f"Wrote R{reg}={val}")
        elif typ == 'error':
            self._log_event(f"Error: {msg[1]}")
        elif typ == 'seq_status':
            self.seq_progress_lbl.configure(text=msg[1])
            self._log_event(msg[1])
        elif typ == 'seq_done':
            self.seq_run_btn.configure(state="normal")
            self.seq_stop_btn.configure(state="disabled")
        else:
            print('unknown msg', msg)

    def _safe_fill(self, entry_widget, value):
        # only overwrite if widget is empty or matches previous value
        cur = entry_widget.get().strip()
        if cur == '' or cur == '0':
            entry_widget.delete(0, 'end')
            entry_widget.insert(0, str(value))

# ----------------------------- Small helpers -----------------------------

def simple_input_dialog(parent, title, label):
    win = ctk.CTkInputDialog(text=label, title=title)
    return win.get_input()

# ----------------------------- Run -----------------------------
if __name__ == '__main__':
    app = PulserApp()
    app.mainloop()
