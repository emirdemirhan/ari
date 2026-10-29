"""Pilot command timeline and fault injection for a scenario."""

from __future__ import annotations

import math

from .config import Scenario

CHANNELS = ("altitude_m", "roll", "pitch", "yaw_rate")


class SetpointTimeline:
    """Piecewise-constant (optionally ramped) pilot commands. Unset channels keep their value."""

    def __init__(self, scenario: Scenario):
        self.segments = {ch: [] for ch in CHANNELS}      # per channel: (t, ramp, value)
        for sp in scenario.setpoints:
            for ch in CHANNELS:
                v = getattr(sp, ch)
                if v is not None:
                    self.segments[ch].append((sp.t, sp.ramp_s, float(v)))

    def _value(self, ch: str, t: float) -> float:
        # A ramp starts from the previous setpoint's value, so ramps should not overlap
        value = 0.0
        for t0, ramp, v in self.segments[ch]:
            if t < t0:
                break
            if ramp > 0 and t < t0 + ramp:
                return value + (v - value) * (t - t0) / ramp
            value = v
        return value

    def get(self, t: float) -> dict:
        return {ch: self._value(ch, t) for ch in CHANNELS}


class FaultInjector:
    def __init__(self, scenario: Scenario):
        self.pending = sorted(scenario.faults, key=lambda f: f.t)
        self.log = []

    def apply(self, t: float, vehicle, imu, rangefinder, barometer=None):
        while self.pending and self.pending[0].t <= t + 1e-9:
            f = self.pending.pop(0)
            if f.type == "motor_scale":
                vehicle.efficiency[f.motor - 1] = float(f.value)
            elif f.type == "gyro_bias_step":
                for i in range(3):
                    imu.gyro_fault[i] += float(f.value[i]) * math.pi / 180.0
            elif f.type == "accel_bias_step":
                for i in range(3):
                    imu.accel_fault[i] += float(f.value[i]) * 1e-3 * 9.80665
            elif f.type == "range_blocked":
                rangefinder.blocked_until = f.t + f.duration_s
            elif f.type == "baro_blocked" and barometer is not None:
                barometer.blocked_until = f.t + f.duration_s
            else:
                raise ValueError(f"unknown fault type '{f.type}'")
            self.log.append((t, f.type))


class ReplayRc:
    """Pilot commands recorded in a run's log.csv, replayed step by step (same 2 ms grid)."""

    def __init__(self, log_path, roll_pitch_scale: float = 0.52):
        import numpy as np

        d = np.genfromtxt(log_path, delimiter=",", names=True)
        self.t = d["t"] - 0.002            # log rows are stamped at the end of their step
        self.alt = d["ref_alt"]
        self.roll = d["ref_roll"] / roll_pitch_scale
        self.pitch = d["ref_pitch"] / roll_pitch_scale
        self.yaw = d["ref_yaw_rate"]
        self.duration = float(d["t"][-1])

    def get(self, t: float) -> dict:
        i = min(max(int(round(t / 0.002)), 0), len(self.t) - 1)
        return {"altitude_m": float(self.alt[i]), "roll": float(self.roll[i]),
                "pitch": float(self.pitch[i]), "yaw_rate": float(self.yaw[i])}
