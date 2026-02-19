import tkinter as tk
from tkinter import ttk

from bcon_simple_modbus import BconModbus


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("BCON Simple Modbus GUI")
        self.driver: BconModbus | None = None

        self.port_var = tk.StringVar(value="COM3")
        self.channel_var = tk.IntVar(value=1)
        self.duration_var = tk.StringVar(value="100")
        self.watchdog_var = tk.StringVar(value="1000")
        self.status_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self._tick()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.port_var, width=10).grid(row=0, column=1, sticky="w")
        ttk.Button(frame, text="Connect", command=self.connect).grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="Disconnect", command=self.disconnect).grid(row=0, column=3)

        ttk.Label(frame, text="Channel").grid(row=1, column=0, sticky="w")
        ttk.Combobox(frame, textvariable=self.channel_var, values=[1, 2, 3], width=7, state="readonly").grid(row=1, column=1, sticky="w")

        ttk.Label(frame, text="Pulse ms").grid(row=2, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.duration_var, width=10).grid(row=2, column=1, sticky="w")

        ttk.Label(frame, text="Watchdog ms").grid(row=3, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.watchdog_var, width=10).grid(row=3, column=1, sticky="w")
        ttk.Button(frame, text="Set WD", command=self.set_watchdog).grid(row=3, column=2, padx=4)

        ttk.Button(frame, text="Set DC", command=self.set_dc).grid(row=4, column=0, pady=6, sticky="ew")
        ttk.Button(frame, text="Pulse", command=self.fire_pulse).grid(row=4, column=1, pady=6, sticky="ew")
        ttk.Button(frame, text="Off", command=self.set_off).grid(row=4, column=2, pady=6, sticky="ew")

        ttk.Label(frame, text="Status").grid(row=5, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.status_var, width=70, anchor="w").grid(row=5, column=1, columnspan=3, sticky="w")

    def connect(self) -> None:
        self.disconnect()
        self.driver = BconModbus(port=self.port_var.get())
        if not self.driver.connect():
            self.status_var.set("Connect failed")
            self.driver = None
            return
        self.status_var.set("Connected")

    def disconnect(self) -> None:
        if self.driver is not None:
            self.driver.close()
            self.driver = None
        self.status_var.set("Disconnected")

    def set_watchdog(self) -> None:
        if self.driver is None:
            return
        try:
            self.driver.set_watchdog_ms(int(self.watchdog_var.get()))
            self.status_var.set("Watchdog updated")
        except Exception as exc:
            self.status_var.set(f"WD error: {exc}")

    def set_dc(self) -> None:
        if self.driver is None:
            return
        try:
            self.driver.set_dc(int(self.channel_var.get()))
            self.status_var.set("DC set")
        except Exception as exc:
            self.status_var.set(f"DC error: {exc}")

    def fire_pulse(self) -> None:
        if self.driver is None:
            return
        try:
            self.driver.fire_pulse(int(self.channel_var.get()), int(self.duration_var.get()))
            self.status_var.set("Pulse fired")
        except Exception as exc:
            self.status_var.set(f"Pulse error: {exc}")

    def set_off(self) -> None:
        if self.driver is None:
            return
        try:
            self.driver.set_off(int(self.channel_var.get()))
            self.status_var.set("Channel off")
        except Exception as exc:
            self.status_var.set(f"Off error: {exc}")

    def _tick(self) -> None:
        if self.driver is not None:
            try:
                self.driver.kick_watchdog()
                status = self.driver.read_status()
                ch = status.channels[int(self.channel_var.get())]
                self.status_var.set(
                    f"SYS={status.sys_state} REASON={status.sys_reason} "
                    f"FLT={status.fault_latched} INT={status.interlock_ok} WD={status.watchdog_ok} "
                    f"ERR={status.last_error} | CH mode={ch['mode']} pulse_ms={ch['pulse_ms']} "
                    f"remain={ch['remaining']} en={ch['enable']} pwr={ch['power']} oc={ch['overcurrent']}"
                )
            except Exception as exc:
                self.status_var.set(f"Comm error: {exc}")

        self.root.after(300, self._tick)


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
