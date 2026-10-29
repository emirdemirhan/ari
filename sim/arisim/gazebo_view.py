"""Gazebo as the 3D window for arisim.

arisim keeps doing the physics, the sensors and the lock-step exchange with the flight
controller (SIL or PIL); Gazebo only shows the result. This module launches
`gz sim sim/gazebo/arisim_world.sdf` and the relay (sim/gazebo/gz_relay.py, run with the
Python that has the gz bindings), then streams the quad's pose and rotor speeds over UDP at
about 60 Hz of wall time. Terrain boxes from the scenario are created in Gazebo at startup.
"""

from __future__ import annotations

import os
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path

GAZEBO_DIR = Path(__file__).resolve().parents[1] / "gazebo"
WORLD = GAZEBO_DIR / "arisim_world.sdf"
RELAY = GAZEBO_DIR / "gz_relay.py"
POSE = struct.Struct("<7d4f")
BOX = struct.Struct("<5d")


def _find_gz_python() -> str:
    """The gz Python bindings are built for one Python version; find an interpreter that loads them."""
    for exe in ("python3.12", "python3"):
        path = shutil.which(exe)
        if not path:
            continue
        ok = subprocess.run([path, "-c", "import sys; sys.path.insert(0, '/usr/lib/python3/dist-packages'); "
                                        "import gz.transport13, gz.msgs10"], capture_output=True)
        if ok.returncode == 0:
            return path
    raise RuntimeError("no Python interpreter can import gz.transport13 (install python3-gz-transport13)")


def _service_up(name: str) -> bool:
    res = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=10)
    return name in res.stdout


class GazeboView:
    def __init__(self, scenario, port: int = 47800, gui: bool = True, rate_hz: float = 60.0, log_dir=None):
        self.port = port
        self.addr = ("127.0.0.1", port)
        self.period = 1.0 / rate_hz
        self.procs = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_send = 0.0
        self.follow_sent = 0
        self.spin = None
        self.log_path = None
        self._log = None

        if shutil.which("gz") is None:
            raise RuntimeError("gz (Gazebo) is not installed")
        if not _service_up("/world/arisim/set_pose"):
            cmd = ["gz", "sim", "-r", str(WORLD)] if gui else ["gz", "sim", "-s", "-r", str(WORLD)]
            # LIBGL_ALWAYS_SOFTWARE=1 makes Mesa's EGL crash inside Ogre2 when it probes an NVIDIA
            # GPU (segfault in driCreateNewScreen3); Gazebo renders fine on the GPU without it.
            env = {k: v for k, v in os.environ.items() if k != "LIBGL_ALWAYS_SOFTWARE"}
            log_dir = Path(log_dir) if log_dir else Path(tempfile.gettempdir())
            log_dir.mkdir(parents=True, exist_ok=True)
            self.log_path = log_dir / "gazebo.log"
            self._log = open(self.log_path, "w")
            self.procs.append(subprocess.Popen(cmd, start_new_session=True, env=env,
                                               stdout=self._log, stderr=subprocess.STDOUT))
            print(f"starting Gazebo ... (log: {self.log_path})", flush=True)
            deadline = time.time() + 60
            while not _service_up("/world/arisim/set_pose"):
                if time.time() > deadline or self.procs[0].poll() is not None:
                    self.close()
                    raise RuntimeError("Gazebo did not start (try `gz sim sim/gazebo/arisim_world.sdf` by hand)")
                time.sleep(0.5)

        self.procs.append(subprocess.Popen([_find_gz_python(), str(RELAY), "--port", str(port)],
                                           start_new_session=True))
        time.sleep(1.0)
        self._gui = gui
        self._gui_checked = False
        for b in scenario.terrain:
            self.sock.sendto(b"B" + BOX.pack(b.center_m[0], b.center_m[1], b.size_m[0], b.size_m[1], b.height_m),
                             self.addr)

    def __call__(self, sim, row):
        now = time.perf_counter()
        if now - self.last_send < self.period:
            return
        self.last_send = now
        if self.spin is None:
            self.spin = sim.vehicle.spin
        x = sim.vehicle.state
        rotors = [w * s for w, s in zip(x[13:17], self.spin)]
        self.sock.sendto(b"P" + POSE.pack(*x[0:3], *x[6:10], *rotors), self.addr)
        if self._gui and not self._gui_checked and row[0] > 5.0 and self.log_path:
            self._gui_checked = True
            text = self.log_path.read_text(errors="replace")
            if "Segmentation fault" in text or "Stack trace" in text:
                print(f"warning: the Gazebo window crashed; see {self.log_path}. The flight continues.", flush=True)
        # The GUI may come up after the server: ask it to follow the quad a few times
        if self.follow_sent < 5 and int(row[0]) >= self.follow_sent:
            self.sock.sendto(b"F", self.addr)
            self.follow_sent += 1

    def close(self):
        for p in reversed(self.procs):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.procs = []
        if self._log:
            self._log.close()
            self._log = None


class Observers:
    """Calls several per-step observers in turn."""

    def __init__(self, *observers):
        self.observers = [o for o in observers if o is not None]

    def __call__(self, sim, row):
        for o in self.observers:
            o(sim, row)

    def close(self):
        for o in self.observers:
            if hasattr(o, "close"):
                o.close()
