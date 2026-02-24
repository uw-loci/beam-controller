"""
Pulser Upper‑Machine GUI (Tkinter + Material-style with CustomTkinter)
- Modbus RTU master using pymodbus
- Auto-scans serial ports (Linux) by default
- Features implemented (first version):
  - Manual per-channel control (start/stop/single + parameter write)
  - Synchronous start/stop via `SYNC_CMD` (mask + action)
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

# ----------------------------- Modbus / Register map -----------------------------
TOTAL_REGS = 160

REG_WATCHDOG_MS = 0
REG_TELEMETRY_MS = 1
REG_COMMAND = 2

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

    def connect(self, port, baud, unit):
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
        self.connected = ok
        self.ui_queue.put(("connected", ok))
        return ok

    def disconnect(self):
        self.connected = False
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
                    ok &= read_block(110, 29)    # channel status regs 110..138

                    if not ok:
                        # generic failure, but not an exception; log if different
                        err = "Modbus read failed"
                        if err != last_error_msg:
                            self.ui_queue.put(("error", err))
                            last_error_msg = err
                    elif regs != self.last_regs:
                        self.last_regs = regs.copy()
                        self.ui_queue.put(("regs", regs))
                        last_error_msg = None
                except Exception as e:
                    err = f"Poll error: {e}"
                    if err != last_error_msg:
                        self.ui_queue.put(("error", err))
                        last_error_msg = err
                    # mark disconnected and tear down client to avoid further attempts
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

        # Sync control
        sync_frame = ctk.CTkFrame(left)
        sync_frame.pack(fill="x", pady=8, padx=6)
        ctk.CTkLabel(sync_frame, text="Synchronous Control", font=ctk.CTkFont(size=13, weight="bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(4,8))
        self.sync_ch_vars = [ctk.BooleanVar(value=False) for _ in range(3)]
        for ch in range(3):
            ctk.CTkCheckBox(sync_frame, text=f"CH{ch+1}", variable=self.sync_ch_vars[ch]).grid(row=1, column=ch)
        self.sync_action = ctk.CTkComboBox(sync_frame, values=["dc", "pulse", "train", "stop", "toggle"], width=140)
        self.sync_action.set("pulse")
        self.sync_action.grid(row=1, column=3, padx=8)
        ctk.CTkButton(sync_frame, text="Send SYNC", command=self._send_sync).grid(row=2, column=0, columnspan=4, pady=8)

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
            ok = self.modbus.connect(port, baud, unit)
            if ok:
                self.conn_label.configure(text=f"Connected ({port})", text_color="green")
            else:
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
        self._log_event(f"Applied CH{ch+1}: mode={mode_label} duration_ms={duration} count={count}")

    def _set_mode(self, ch, mode):
        base = CH_BASE[ch]
        if mode == MODE_PULSE:
            self.modbus.enqueue_write(base + CH_COUNT_OFF, 1)
        self.modbus.enqueue_write(base + CH_MODE_OFF, mode)
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

    def _send_sync(self):
        selected = [ch for ch in range(3) if self.sync_ch_vars[ch].get()]
        if not selected:
            self._log_event("SYNC ignored: no channels selected")
            return

        action = self.sync_action.get()
        for ch in selected:
            base = CH_BASE[ch]
            if action == 'dc':
                self.modbus.enqueue_write(base + CH_MODE_OFF, MODE_DC)
            elif action == 'pulse':
                count = self.modbus.last_regs[base + CH_COUNT_OFF]
                if count <= 0:
                    self.modbus.enqueue_write(base + CH_COUNT_OFF, 1)
                self.modbus.enqueue_write(base + CH_MODE_OFF, MODE_PULSE)
            elif action == 'train':
                count = self.modbus.last_regs[base + CH_COUNT_OFF]
                if count < 2:
                    self.modbus.enqueue_write(base + CH_COUNT_OFF, 2)
                self.modbus.enqueue_write(base + CH_MODE_OFF, MODE_PULSE_TRAIN)
            elif action == 'stop':
                self.modbus.enqueue_write(base + CH_MODE_OFF, MODE_OFF)
            else:  # toggle
                status_base = REG_CH_STATUS_BASE + (ch * REG_CH_STATUS_STRIDE)
                cur_mode = self.modbus.last_regs[status_base]
                if cur_mode == MODE_OFF:
                    count = self.modbus.last_regs[base + CH_COUNT_OFF]
                    mode = MODE_PULSE if count <= 1 else MODE_PULSE_TRAIN
                    self.modbus.enqueue_write(base + CH_MODE_OFF, mode)
                else:
                    self.modbus.enqueue_write(base + CH_MODE_OFF, MODE_OFF)

        self._log_event(f"SYNC emulated: {action} channels={[c+1 for c in selected]}")

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
