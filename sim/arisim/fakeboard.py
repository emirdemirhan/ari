"""A virtual flight controller board on a pseudo-terminal, for testing the PIL tooling.

It behaves like the firmware on USART1: prints "Hello from STM32" at boot, answers each sensor
frame through the SIL build of the controller, resends its cached reply for a repeated seq,
and reboots (fresh controller state) on a reset frame. Optional link faults (dropped replies,
corrupted bytes) exercise the lock-step retry path.

    python3 -m arisim fakeboard            # prints the port, then: run ... --port <that port>
"""

from __future__ import annotations

import os
import threading
import termios
import time
import tty

import numpy as np

from . import protocol
from .links import SilLibrary


class FakeBoard:
    def __init__(self, drop_reply_prob: float = 0.0, corrupt_prob: float = 0.0, seed: int = 0, build: bool = True,
                 reboot_after: int = 0, stall_after: int = 0, stall_s: float = 0.0):
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        self._slave = slave
        self.drop_reply_prob = drop_reply_prob
        self.corrupt_prob = corrupt_prob
        self.rng = np.random.default_rng(seed)
        self.build = build
        self.lib = None
        self.boots = 0
        self.replies_dropped = 0
        self.reboot_after = reboot_after      # simulate a watchdog reset after this many replies (0 = never)
        self.replies = 0
        self.stall_after = stall_after        # stop answering once, for stall_s seconds, after this many replies
        self.stall_s = stall_s
        self._decoder = protocol.Decoder()
        self._stop = False
        self._boot("POR BOR")
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _boot(self, cause):
        if self.lib:
            self.lib.close()
        self.lib = SilLibrary(build=self.build and self.boots == 0)
        self.boots += 1
        os.write(self.master, f"Hello from STM32 (reset: {cause}) proto sim={protocol.SIM_FORMAT.size} "
                              f"fc={protocol.FC_FORMAT.size}\r\n".encode())

    def _run(self):
        while not self._stop:
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if self.corrupt_prob > 0:
                data = bytearray(data)
                for i in range(len(data)):
                    if self.rng.uniform() < self.corrupt_prob:
                        data[i] ^= 0x10
                data = bytes(data)
            # Reset frames are handled here; everything goes to the SIL decoder as on the board
            for ftype, _ in self._decoder.feed(data):
                if ftype == protocol.TYPE_RESET:
                    self._boot("SFT")
            reply = self.lib.exchange(data)
            if reply and self.reboot_after and self.replies >= self.reboot_after:
                self.reboot_after = 0
                self._boot("IWDG")
                continue
            if reply and self.stall_after and self.replies >= self.stall_after:
                self.stall_after = 0
                time.sleep(self.stall_s)
                termios.tcflush(self._slave, termios.TCIFLUSH)   # what arrived meanwhile is lost
                continue
            if reply:
                self.replies += 1
                if self.rng.uniform() < self.drop_reply_prob:
                    self.replies_dropped += 1
                    continue
                os.write(self.master, reply)

    def close(self):
        self._stop = True
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if self.lib:
            self.lib.close()
