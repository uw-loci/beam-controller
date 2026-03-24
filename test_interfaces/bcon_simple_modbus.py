from __future__ import annotations

from dataclasses import dataclass
import inspect
from typing import Dict, Any

import logging
import time
from pymodbus.client import ModbusSerialClient

# suppress chatty pymodbus internals
logging.getLogger("pymodbus").setLevel(logging.ERROR)


REG_WATCHDOG_MS = 0
REG_COMMAND = 2
REG_CH_MODE = {1: 10, 2: 20, 3: 30}
REG_CH_PULSE_MS = {1: 11, 2: 21, 3: 31}
REG_SYS_STATE_BASE = 100
REG_CH_STATUS_BASE = 110
REG_CH_STATUS_STRIDE = 10


@dataclass
class BconStatus:
    sys_state: int
    sys_reason: int
    fault_latched: int
    interlock_ok: int
    watchdog_ok: int
    last_error: int
    channels: Dict[int, Dict[str, int]]


class BconModbus:
    def __init__(
        self,
        port: str,
        slave_id: int = 1,
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        self.slave_id = slave_id
        try:
            self.client = ModbusSerialClient(
                port=port,
                framer="rtu",
                baudrate=baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=timeout,
                retries=1,
            )
        except TypeError:
            self.client = ModbusSerialClient(
                method="rtu",
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=timeout,
            )

    def connect(self, settle_s: float = 2.5) -> bool:
        """Open the port and wait for the Mega to finish its boot sequence.
        
        Opening the serial port asserts DTR which resets the Arduino.  We must
        wait for the firmware to complete setup() (LCD init + WDT enable) before
        sending any Modbus frames, otherwise pymodbus times out immediately.
        """
        ok = bool(self.client.connect())
        if ok and settle_s > 0:
            time.sleep(settle_s)
        return ok

    def close(self) -> None:
        # close underlying serial connection and forget the client object
        try:
            self.client.close()
        except Exception:
            pass
        self.client = None

    @staticmethod
    def _is_ok(response: Any) -> bool:
        return response is not None and not response.isError()

    def _write_register_compat(self, address: int, value: int) -> Any:
        write_fn = self.client.write_register
        sig = inspect.signature(write_fn)
        if "device_id" in sig.parameters:
            return write_fn(address=address, value=value, device_id=self.slave_id)
        if "unit" in sig.parameters:
            return write_fn(address=address, value=value, unit=self.slave_id)
        if "slave" in sig.parameters:
            return write_fn(address=address, value=value, slave=self.slave_id)
        return write_fn(address=address, value=value)

    def _read_holding_registers_compat(self, address: int, count: int) -> Any:
        read_fn = self.client.read_holding_registers
        sig = inspect.signature(read_fn)
        if "device_id" in sig.parameters:
            return read_fn(address=address, count=count, device_id=self.slave_id)
        if "unit" in sig.parameters:
            return read_fn(address=address, count=count, unit=self.slave_id)
        if "slave" in sig.parameters:
            return read_fn(address=address, count=count, slave=self.slave_id)
        return read_fn(address=address, count=count)

    def _write_reg(self, address: int, value: int) -> None:
        response = self._write_register_compat(address=address, value=value)
        if not self._is_ok(response):
            raise RuntimeError(f"Modbus write failed at {address}: {response}")

    def _read_regs(self, address: int, count: int) -> list[int]:
        response = self._read_holding_registers_compat(address=address, count=count)
        if not self._is_ok(response):
            raise RuntimeError(f"Modbus read failed at {address}: {response}")
        return list(response.registers)

    def kick_watchdog(self) -> None:
        self._write_reg(REG_COMMAND, 0)

    def set_watchdog_ms(self, value_ms: int) -> None:
        self._write_reg(REG_WATCHDOG_MS, int(value_ms))

    def set_dc(self, channel: int) -> None:
        self._write_reg(REG_CH_MODE[channel], 1)

    def set_off(self, channel: int) -> None:
        self._write_reg(REG_CH_MODE[channel], 0)

    def fire_pulse(self, channel: int, duration_ms: int) -> None:
        self._write_reg(REG_CH_PULSE_MS[channel], int(duration_ms))
        self._write_reg(REG_CH_MODE[channel], 2)

    def read_status(self) -> BconStatus:
        # One sys read + three individual channel reads (9 regs each).
        # NOTE: each channel block occupies a stride of 10 but only offsets 0-8
        # are implemented; reading 29 contiguous registers would cross the gap
        # at offsets 9/19 which the firmware rejects with exception 0x02.
        sys_regs = self._read_regs(REG_SYS_STATE_BASE, 6)

        channels: Dict[int, Dict[str, int]] = {}
        for ch in (1, 2, 3):
            base = REG_CH_STATUS_BASE + (ch - 1) * REG_CH_STATUS_STRIDE
            regs = self._read_regs(base, 9)
            channels[ch] = {
                "mode":        regs[0],
                "pulse_ms":    regs[1],
                "count":       regs[2],
                "remaining":   regs[3],
                "enable":      regs[4],
                "power":       regs[5],
                "overcurrent": regs[6],
                "gated":       regs[7],
                "gate_out":    regs[8],
            }

        return BconStatus(
            sys_state=sys_regs[0],
            sys_reason=sys_regs[1],
            fault_latched=sys_regs[2],
            interlock_ok=sys_regs[3],
            watchdog_ok=sys_regs[4],
            last_error=sys_regs[5],
            channels=channels,
        )
