"""Links between the simulator and the flight controller.

SilLink    : the controller compiled for the PC (build/sil/libari_sil.so), called through ctypes.
SerialLink : the real board over USART1 (PIL). Lock-step: one frame out, wait for the reply with
             the same seq; on a timeout the same frame is resent and the firmware answers from
             its reply cache without stepping the controller twice.
Both exchange the exact same bytes, so the SIL run also checks the protocol code.
"""

from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from . import protocol
from .protocol import FcData, SimData

FIRMWARE_DIR = Path(__file__).resolve().parents[2]       # ari/
SIL_LIB = FIRMWARE_DIR / "build" / "sil" / "libari_sil.so"
CPU_HZ = 100e6


class LinkError(Exception):
    pass


class SilLibrary:
    """A private copy of build/sil/libari_sil.so: fresh controller and estimator state."""

    def __init__(self, build: bool = True):
        if build:
            res = subprocess.run(["make", "-s", "sil"], cwd=FIRMWARE_DIR, capture_output=True, text=True)
            if res.returncode != 0:
                raise LinkError(f"make sil failed:\n{res.stdout}{res.stderr}")
        if not SIL_LIB.exists():
            raise LinkError(f"{SIL_LIB} not found; run `make sil` in {FIRMWARE_DIR}")
        # The estimators are C++ globals without a reset: load a private copy for a fresh state
        fd, self._path = tempfile.mkstemp(prefix="ari_sil_", suffix=".so")
        os.close(fd)
        shutil.copyfile(SIL_LIB, self._path)
        self.lib = ctypes.CDLL(self._path)
        self.lib.sil_exchange.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        self.lib.sil_exchange.restype = ctypes.c_int
        self.lib.sil_crc_errors.restype = ctypes.c_uint32
        self._out = ctypes.create_string_buffer(512)

        sim_size, fc_size = ctypes.c_int(), ctypes.c_int()
        self.lib.sil_struct_sizes(ctypes.byref(sim_size), ctypes.byref(fc_size))
        if (sim_size.value, fc_size.value) != (protocol.SIM_FORMAT.size, protocol.FC_FORMAT.size):
            raise LinkError(f"struct size mismatch: C sim/fc = {sim_size.value}/{fc_size.value}, "
                            f"Python = {protocol.SIM_FORMAT.size}/{protocol.FC_FORMAT.size}")

    def exchange(self, data: bytes) -> bytes:
        """Feed raw bytes as the board's UART would receive them; returns the reply bytes."""
        n = self.lib.sil_exchange(data, len(data), self._out, len(self._out))
        if n < 0:
            raise LinkError("SIL reply buffer too small")
        return self._out.raw[:n]

    def crc_errors(self) -> int:
        return self.lib.sil_crc_errors()

    def close(self):
        try:
            os.unlink(self._path)
        except OSError:
            pass


class SilLink:
    name = "sil"

    def __init__(self, build: bool = True):
        self.lib = SilLibrary(build)
        self._decoder = protocol.Decoder()
        self.stats = {"exchanges": 0}

    def step(self, sim: SimData) -> FcData:
        reply = self.lib.exchange(sim.frame())
        if not reply:
            raise LinkError(f"SIL returned no reply for seq {sim.seq} (C CRC errors: {self.lib.crc_errors()})")
        frames = self._decoder.feed(reply)
        if len(frames) != 1 or frames[0][0] != protocol.TYPE_FC_DATA:
            raise LinkError("SIL reply did not decode to one FC frame")
        fc = FcData.unpack(frames[0][1])
        if fc.seq != sim.seq:
            raise LinkError(f"SIL replied to seq {fc.seq}, expected {sim.seq}")
        self.stats["exchanges"] += 1
        return fc

    def close(self):
        self.lib.close()


class SerialLink:
    name = "pil"

    def __init__(self, port: str, baud: int = 2_000_000, timeout_s: float = 0.01, retries: int = 50,
                 reset: bool = True, boot_wait_s: float = 1.0):
        import serial  # pyserial

        self.port = port
        self._warn_latency(port)
        self.ser = serial.Serial(port, baud, timeout=0)
        self.timeout_s = timeout_s
        self.retries = retries
        self._decoder = protocol.Decoder()
        self.stats = {"exchanges": 0, "retries": 0, "rtt_ms": [], "stalls": []}
        if reset:
            # Reboot the board so the estimators and integrators start fresh
            self.ser.write(protocol.encode_frame(protocol.TYPE_RESET, b""))
            self.ser.flush()
            time.sleep(boot_wait_s)
            boot = self.ser.read(self.ser.in_waiting or 0).decode("ascii", errors="replace")
            mismatch = protocol.check_boot_message(boot)
            if mismatch:
                self.ser.close()
                raise LinkError(mismatch)
        self.ser.reset_input_buffer()

    @staticmethod
    def _warn_latency(port: str):
        # FTDI adapters buffer for 16 ms by default, which dominates the lock-step round trip
        dev = os.path.basename(os.path.realpath(port))
        path = Path(f"/sys/bus/usb-serial/devices/{dev}/latency_timer")
        try:
            value = int(path.read_text())
            if value > 1:
                print(f"warning: {path} is {value} ms; set it to 1 for fast lock-step "
                      f"(echo 1 | sudo tee {path})")
        except (OSError, ValueError):
            pass

    def step(self, sim: SimData) -> FcData:
        frame = sim.frame()
        for attempt in range(self.retries + 1):
            t0 = time.perf_counter()
            self.ser.write(frame)
            deadline = t0 + self.timeout_s
            while time.perf_counter() < deadline:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue
                frames = self._decoder.feed(data)
                self._check_reboot()
                for ftype, payload in frames:
                    if ftype != protocol.TYPE_FC_DATA or len(payload) != protocol.FC_FORMAT.size:
                        continue
                    fc = FcData.unpack(payload)
                    if fc.seq == sim.seq:
                        self.stats["exchanges"] += 1
                        self.stats["rtt_ms"].append((time.perf_counter() - t0) * 1e3)
                        return fc
            self.stats["retries"] += 1
        why, fc = self._post_mortem(frame, sim.seq)
        if fc is not None:
            # Lock-step: the physics waited for the board, so the flight can go on
            self.stats["stalls"].append((sim.time_us * 1e-6, why))
            print(f"warning: t={sim.time_us * 1e-6:.2f} s: {why}; continuing")
            self.stats["exchanges"] += 1
            return fc
        raise LinkError(f"no reply from the board for seq {sim.seq} after {self.retries} retries "
                        f"(port {self.port}): {why}")

    def _post_mortem(self, frame: bytes, seq: int, listen_s: float = 1.5, probes: int = 20):
        """Find out why the board stopped answering: it may reboot (the watchdog fires 0.5 s
        after a total hang, just as the retries run out), print a fault, or answer again later."""
        t_lost = time.perf_counter()
        late = b""
        answered_after = None
        reply = None
        end = t_lost + listen_s
        next_probe = t_lost
        while time.perf_counter() < end:
            now = time.perf_counter()
            if answered_after is None and now >= next_probe and probes > 0:
                self.ser.write(frame)
                probes -= 1
                next_probe = now + listen_s / 20
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                time.sleep(0.001)
                continue
            late += data
            for ftype, payload in self._decoder.feed(data):
                if (answered_after is None and ftype == protocol.TYPE_FC_DATA
                        and len(payload) == protocol.FC_FORMAT.size and FcData.unpack(payload).seq == seq):
                    answered_after = time.perf_counter() - t_lost
                    reply = FcData.unpack(payload)
        text = bytes(self._decoder.junk).decode("ascii", errors="replace").strip()
        if "HARDFAULT" in text or "STACK OVERFLOW" in text or "ASSERT" in text or "MALLOC" in text:
            return "the firmware crashed and reported: " + text[text.rfind("ARI ") if "ARI " in text else 0:][:200], None
        i = text.find("Hello from STM32")
        if i >= 0:
            line = text[i:].splitlines()[0].strip()
            if "IWDG" in line:
                return (f"the watchdog reset the board ({line}): the whole MCU, including the LED task, "
                        f"stopped for 0.5 s (interrupts blocked, or power/clock trouble)"), None
            if "BOR" in line or "POR" in line:
                return f"the board lost power and rebooted ({line}): check its supply", None
            return f"the board rebooted ({line})", None
        if answered_after is not None:
            return (f"the board answered again {answered_after + self.retries * self.timeout_s:.2f} s after "
                    f"the last reply: a stall, not a hang (PC or USB-serial pause, or the board stopped "
                    f"reading for a while)"), reply
        if late:
            return f"the board sent unreadable data afterwards ({late[:40]!r}): baud rate or wiring", None
        return ("the board sent nothing for 2 s and ignores frames, without rebooting: its control task "
                "is stuck while the rest of the firmware (which feeds the watchdog) runs. "
                "Is the LED still blinking at 1 Hz? Build with `make HIL_RX_DIAG=1` to see the receive counters"), None

    def _check_reboot(self):
        # The firmware prints its boot message (with the reset cause) only after a reset;
        # mid-run it means the controller restarted from scratch, so the flight is over
        text = bytes(self._decoder.junk).decode("ascii", errors="replace")
        i = text.find("Hello from STM32")
        if i >= 0 and self.stats["exchanges"] > 0:
            line = text[i:].splitlines()[0]
            causes = {"IWDG": "the watchdog reset a hung board", "BOR": "the supply voltage dropped",
                      "POR": "power was lost and came back", "PIN": "the reset button/pin",
                      "SFT": "a software reset"}
            why = ", ".join(v for k, v in causes.items() if k in line) or "unknown cause"
            self._decoder.junk.clear()
            raise LinkError(f"the board rebooted during the run ({line.strip()}: {why})")

    def close(self):
        self.ser.close()


def exec_time_us(fc: FcData) -> float:
    return fc.exec_cycles / CPU_HZ * 1e6
