"""Pilot input from a gamepad (pygame), with a keyboard fallback, in firmware conventions.

Gamepad (same layout as the old Gazebo bridge):
  left stick x -> roll, left stick y -> pitch, right stick x -> yaw rate, R1 / L1 -> climb / descend
Keyboard (when no gamepad, or to override):
  A / D roll, W / S pitch (W = forward), Q / E yaw (Q = left), Up / Down altitude reference

Sign conventions are converted here, so pushing the stick forward pitches the nose down and
flies forward, and pushing yaw right turns clockwise seen from above (negative yaw rate).
"""

from __future__ import annotations

import math


class JoystickRc:
    def __init__(self, max_yaw_rate: float = 1.5, climb_rate: float = 1.0, deadzone: float = 0.08,
                 invert_pitch: bool = False, invert_yaw: bool = False, max_altitude: float = math.inf):
        import pygame

        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        self.js = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
        if self.js:
            self.js.init()
        self.name = self.js.get_name() if self.js else "keyboard"
        self.max_yaw_rate = max_yaw_rate
        self.climb_rate = climb_rate
        self.deadzone = deadzone
        self.pitch_sign = 1.0 if invert_pitch else -1.0
        self.yaw_sign = 1.0 if invert_yaw else -1.0
        self.max_altitude = max_altitude
        self.altitude = 0.0
        self.last_t = 0.0
        self.keys = set()            # filled by the viewer from keyboard events

    def _dz(self, v: float) -> float:
        if abs(v) < self.deadzone:
            return 0.0
        return (v - self.deadzone * (1 if v > 0 else -1)) / (1 - self.deadzone)

    def get(self, t: float) -> dict:
        dt = max(0.0, t - self.last_t)
        self.last_t = t
        roll = pitch = yaw = 0.0
        climb = 0.0
        if self.js:
            roll = self._dz(self.js.get_axis(0))
            pitch = self.pitch_sign * self._dz(self.js.get_axis(1))
            if self.js.get_numaxes() > 3:
                yaw = self.yaw_sign * self._dz(self.js.get_axis(3)) * self.max_yaw_rate
            if self.js.get_numbuttons() > 5:
                climb = self.js.get_button(5) - self.js.get_button(4)

        k = self.keys
        if "a" in k or "d" in k:
            roll = 0.5 * (("d" in k) - ("a" in k))
        if "w" in k or "s" in k:
            pitch = 0.5 * (("w" in k) - ("s" in k))
        if "q" in k or "e" in k:
            yaw = self.max_yaw_rate * (("q" in k) - ("e" in k))
        if "up" in k or "down" in k:
            climb = ("up" in k) - ("down" in k)

        self.altitude = min(self.max_altitude, max(0.0, self.altitude + climb * self.climb_rate * dt))
        return {"altitude_m": self.altitude, "roll": roll, "pitch": pitch, "yaw_rate": yaw}
