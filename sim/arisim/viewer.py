"""Live 2D view (pygame): top view, side view, attitude, strip charts and telemetry.

Called by the simulator once per control step; draws at about 30 frames per second of wall
time. Closing the window or pressing Esc ends the run (the log is still written).
"""

from __future__ import annotations

import math
import time
from collections import deque

from .simulator import CONTROL_DT, LOG_COLUMNS

IDX = {name: i for i, name in enumerate(LOG_COLUMNS)}
W, H = 1280, 760
BG = (24, 26, 30)
GRID = (48, 52, 58)
FG = (220, 222, 226)
DIM = (140, 145, 152)
TRUE_C = (240, 240, 240)
REF_C = (235, 80, 80)
KF_C = (80, 160, 240)
ESKF_C = (250, 160, 60)
TERRAIN_C = (90, 90, 100)

KEYMAP = {"a": "a", "d": "d", "w": "w", "s": "s", "q": "q", "e": "e", "up": "up", "down": "down"}


class LiveViewer:
    def __init__(self, joystick=None, history_s: float = 10.0, fps: float = 30.0):
        import pygame

        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("ARI simulator")
        self.font = pygame.font.SysFont("monospace", 14)
        self.small = pygame.font.SysFont("monospace", 12)
        self.joystick = joystick
        self.hist = deque(maxlen=int(history_s / CONTROL_DT))
        self.trail = deque(maxlen=3000)
        self.period = 1.0 / fps
        self.last_draw = 0.0
        self.step = 0
        self.wall0 = None
        self.sim0 = 0.0

    # ------------------------------------------------------------------------------------------

    def __call__(self, sim, row):
        self.hist.append(row)
        self.step += 1
        if self.step % 10 == 0:
            self.trail.append((row[IDX["x"]], row[IDX["y"]]))
        now = time.perf_counter()
        if now - self.last_draw < self.period:
            return
        self.last_draw = now
        self._events(sim)
        self.draw(sim, row)

    def _events(self, sim):
        pg = self.pg
        for ev in pg.event.get():
            if ev.type == pg.QUIT or (ev.type == pg.KEYDOWN and ev.key == pg.K_ESCAPE):
                sim.stop_requested = True
            elif ev.type in (pg.KEYDOWN, pg.KEYUP) and self.joystick is not None:
                name = pg.key.name(ev.key)
                if name in KEYMAP:
                    if ev.type == pg.KEYDOWN:
                        self.joystick.keys.add(KEYMAP[name])
                    else:
                        self.joystick.keys.discard(KEYMAP[name])

    # ------------------------------------------------------------------------------------------

    def _text(self, s, x, y, color=FG, font=None):
        self.screen.blit((font or self.font).render(s, True, color), (x, y))

    def draw(self, sim, row):
        pg = self.pg
        r = {name: row[i] for i, name in enumerate(LOG_COLUMNS)}
        self.screen.fill(BG)
        for fn, rect in ((self._top_view, pg.Rect(10, 10, 440, 440)), (self._side_view, pg.Rect(460, 10, 420, 250))):
            self.screen.set_clip(rect)
            fn(sim, r, rect)
            self.screen.set_clip(None)
        self._attitude(r, pg.Rect(890, 10, 380, 250))
        self._telemetry(sim, r, pg.Rect(460, 270, 810, 180))
        charts = [("height (m)", ["agl", "ref_alt", "kf_alt", "eskf_alt"], 1.0),
                  ("roll (deg)", ["roll", "ref_roll", "kf_roll", "eskf_roll"], 180 / math.pi),
                  ("pitch (deg)", ["pitch", "ref_pitch", "kf_pitch", "eskf_pitch"], 180 / math.pi)]
        for i, (title, cols, scale) in enumerate(charts):
            self._chart(pg.Rect(10 + i * 422, 460, 412, 290), title, cols, scale)
        pg.display.flip()

    def _top_view(self, sim, r, rect):
        pg = self.pg
        pg.draw.rect(self.screen, (30, 33, 38), rect)
        scale = 40.0                                   # px per metre
        cx, cy = r["x"], r["y"]

        def to_px(x, y):                               # world x up the screen, y to the left
            return (rect.centerx - (y - cy) * scale, rect.centery - (x - cx) * scale)

        for k in range(-6, 7):
            gx = math.floor(cx) + k
            gy = math.floor(cy) + k
            a, b = to_px(gx, cy - 6), to_px(gx, cy + 6)
            pg.draw.line(self.screen, GRID, a, b)
            a, b = to_px(cx - 6, gy), to_px(cx + 6, gy)
            pg.draw.line(self.screen, GRID, a, b)
        for bx in sim.env.terrain.boxes:
            x0, x1, y0, y1, h = bx
            p0, p1 = to_px(x1, y1), to_px(x0, y0)
            box = pg.Rect(min(p0[0], p1[0]), min(p0[1], p1[1]), abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))
            pg.draw.rect(self.screen, TERRAIN_C, box.clip(rect))
        if len(self.trail) > 1:
            pts = [to_px(x, y) for x, y in self.trail]
            pg.draw.lines(self.screen, (120, 200, 120), False, pts, 1)

        yaw = r["yaw"]
        c, s = math.cos(yaw), math.sin(yaw)
        k = 3.0                                        # vehicle drawn 3x larger than the grid scale
        for i, (px, py, _) in enumerate(sim.vp.rotor.positions_m):
            px, py = k * px, k * py
            wx, wy = cx + c * px - s * py, cy + s * px + c * py
            pg.draw.line(self.screen, DIM, to_px(cx, cy), to_px(wx, wy), 3)
            cmd = r[f"m{i + 1}"] / 100.0
            col = (int(60 + 195 * cmd), int(200 - 120 * cmd), 80)
            pg.draw.circle(self.screen, col, [int(v) for v in to_px(wx, wy)], 9, 0)
            self._text(str(i + 1), *to_px(wx, wy), color=BG, font=self.small)
        nose = to_px(cx + 0.9 * c, cy + 0.9 * s)
        pg.draw.line(self.screen, REF_C, to_px(cx, cy), nose, 2)
        wind = (r["wind_x"], r["wind_y"])
        if math.hypot(*wind) > 0.05:
            a = (rect.x + 50, rect.y + 50)
            b = (a[0] - wind[1] * 12, a[1] - wind[0] * 12)
            pg.draw.line(self.screen, (120, 180, 255), a, b, 3)
            pg.draw.circle(self.screen, (120, 180, 255), [int(v) for v in b], 4)
            self._text(f"wind {math.hypot(*wind):.1f} m/s", rect.x + 8, rect.y + 8, DIM, self.small)
        self._text("top view, 1 m grid (vehicle drawn 3x), x up, y left", rect.x + 8, rect.bottom - 20, DIM, self.small)

    def _side_view(self, sim, r, rect):
        pg = self.pg
        pg.draw.rect(self.screen, (30, 33, 38), rect)
        scale = 50.0
        ground_y = rect.bottom - 30
        x0 = r["x"]

        def to_px(x, z):
            return (rect.centerx + (x - x0) * scale, ground_y - z * scale)

        # Terrain profile along world x through the vehicle
        pts = []
        for k in range(0, rect.width + 1, 4):
            x = x0 + (k - rect.width / 2) / scale
            pts.append((rect.x + k, ground_y - sim.env.terrain.height(x, r["y"]) * scale))
        pg.draw.lines(self.screen, TERRAIN_C, False, pts, 2)
        ref_y = ground_y - (r["ref_alt"] + r["terrain"]) * scale
        if rect.y < ref_y < rect.bottom:
            pg.draw.line(self.screen, REF_C, (rect.x, ref_y), (rect.right, ref_y), 1)
        pos = to_px(r["x"], r["z"])
        pitch = r["pitch"]
        dx, dz = math.cos(pitch) * 0.22 * scale, math.sin(pitch) * 0.22 * scale
        pg.draw.line(self.screen, TRUE_C, (pos[0] - dx, pos[1] - dz), (pos[0] + dx, pos[1] + dz), 4)
        if r["range_valid"] > 0.5:
            pg.draw.line(self.screen, (255, 90, 200), pos, (pos[0], pos[1] + r["range"] * scale), 1)
        self._text(f"side view  z {r['z']:.2f} m  agl {r['agl']:.2f} m  ref {r['ref_alt']:.2f} m",
                   rect.x + 8, rect.y + 8, DIM, self.small)

    def _attitude(self, r, rect):
        pg = self.pg
        pg.draw.rect(self.screen, (30, 33, 38), rect)
        c = (rect.x + 75, rect.centery)
        rad = 65
        pg.draw.circle(self.screen, GRID, c, rad, 1)
        for name, col, w in (("ref", REF_C, 1), ("eskf", ESKF_C, 2), ("kf", KF_C, 2), ("", TRUE_C, 3)):
            roll = r[f"{name}_roll" if name else "roll"]
            pitch = r[f"{name}_pitch" if name else "pitch"]
            off = pitch * 180 / math.pi * 3
            dx, dy = math.cos(-roll) * rad, math.sin(-roll) * rad
            pg.draw.line(self.screen, col, (c[0] - dx, c[1] - dy + off), (c[0] + dx, c[1] + dy + off), w)
        d = 180 / math.pi
        y = rect.y + 10
        for label, key, col in (("true", "", TRUE_C), ("ref ", "ref_", REF_C), ("KF  ", "kf_", KF_C), ("ESKF ", "eskf_", ESKF_C)):
            self._text(f"{label} roll {r[key + 'roll'] * d:+6.1f}  pitch {r[key + 'pitch'] * d:+6.1f}",
                       rect.x + 155, y, col, self.small)
            y += 18
        self._text(f"yaw  true {r['yaw'] * d:+7.1f}", rect.x + 155, y + 8, TRUE_C, self.small)
        self._text(f"     KF   {r['kf_yaw'] * d:+7.1f}", rect.x + 155, y + 26, KF_C, self.small)
        self._text(f"     ESKF  {r['eskf_yaw'] * d:+7.1f}", rect.x + 155, y + 44, ESKF_C, self.small)
        self._text("attitude (deg)", rect.x + 8, rect.bottom - 20, DIM, self.small)

    def _telemetry(self, sim, r, rect):
        pg = self.pg
        pg.draw.rect(self.screen, (30, 33, 38), rect)
        x, y = rect.x + 10, rect.y + 8
        exec_s = "n/a (SIL)" if r["exec_us"] <= 0 else f"{r['exec_us']:.0f} us"
        if self.wall0 is None:
            self.wall0, self.sim0 = time.perf_counter(), r["t"]
        wall = time.perf_counter() - self.wall0
        rtf = (r["t"] - self.sim0) / wall if wall > 0.5 else 0.0
        rtt = sim.link.stats.get("rtt_ms")
        rtt_s = f"   round trip {sum(rtt[-200:]) / len(rtt[-200:]):.2f} ms" if rtt else ""
        lines = [
            f"t {r['t']:7.2f} s   link {sim.link.name.upper()}   real-time factor {rtf:4.2f}{rtt_s}",
            f"estimator to PIDs: {'ESKF' if r['use_eskf'] > 0.5 else 'KF'}",
            f"motors %  {r['m1']:5.1f} {r['m2']:5.1f} {r['m3']:5.1f} {r['m4']:5.1f}   collective {r['total_thrust']:5.1f}",
            f"rpm       {r['rpm1']:5.0f} {r['rpm2']:5.0f} {r['rpm3']:5.0f} {r['rpm4']:5.0f}",
            f"battery   {r['battery_v']:5.2f} V  {r['battery_a']:5.1f} A  {100 * r['soc']:4.1f} %",
            f"speed     {math.hypot(r['vx'], r['vy']):4.1f} m/s horiz  {r['vz']:+5.2f} m/s vert   exec {exec_s}",
        ]
        if self.joystick is not None:
            lines.append(f"pilot: {self.joystick.name}   keys: WASD tilt, Q/E yaw, Up/Down altitude, Esc quit")
        flags = int(r["fc_flags"])
        if flags & 2:
            lines.append("CALIBRATING IMU: keep still (motors off for 2 s)")
        elif flags & 16:
            lines.append("NO HEIGHT SENSOR: holding hover thrust / descending, tilt limited to 10 deg")
        elif flags & 8:
            lines.append(f"above rangefinder reach: height from the barometer ({r['eskf_alt']:.1f} m)")
        elif flags & 4:
            lines.append("ON GROUND, motors idle: raise the altitude (R1 / Up arrow) to take off")
        elif sim.vehicle.in_contact:
            lines.append("touching the ground")
        for s in lines:
            self._text(s, x, y)
            y += 22

    def _chart(self, rect, title, cols, scale):
        pg = self.pg
        pg.draw.rect(self.screen, (30, 33, 38), rect)
        self._text(title, rect.x + 8, rect.y + 6, DIM, self.small)
        if len(self.hist) < 2:
            return
        step = max(1, len(self.hist) // rect.width)
        data = list(self.hist)[::step]
        series = [[row[IDX[c]] * scale for row in data] for c in cols]
        lo = min(min(s) for s in series)
        hi = max(max(s) for s in series)
        if hi - lo < 1e-3:
            hi, lo = hi + 0.5, lo - 0.5
        pad = 0.1 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        inner = rect.inflate(-16, -36).move(0, 10)

        def py(v):
            return inner.bottom - (v - lo) / (hi - lo) * inner.height

        for frac in (0.0, 0.5, 1.0):
            v = lo + frac * (hi - lo)
            pg.draw.line(self.screen, GRID, (inner.x, py(v)), (inner.right, py(v)))
            self._text(f"{v:.2f}", inner.x + 2, py(v) - 14, DIM, self.small)
        for s, col in zip(series, (TRUE_C, REF_C, KF_C, ESKF_C)):
            pts = [(inner.x + i * inner.width / (len(s) - 1), py(v)) for i, v in enumerate(s)]
            pg.draw.lines(self.screen, col, False, pts, 1)
        lx = rect.right - 190
        for name, col in zip(("true", "ref", "KF", "ESKF"), (TRUE_C, REF_C, KF_C, ESKF_C)):
            self._text(name, lx, rect.y + 6, col, self.small)
            lx += 45

    def screenshot(self, path):
        self.pg.image.save(self.screen, str(path))
