"""World: gravity, air, wind (mean + turbulence + gusts) and terrain (ground plane + boxes)."""

from __future__ import annotations

import math

from .config import Scenario, WindParams


class Wind:
    """Mean wind + first-order Gauss-Markov turbulence per axis + 1-cosine gusts, world frame."""

    def __init__(self, params: WindParams, rng):
        self.p = params
        self.rng = rng
        self.turb = [0.0, 0.0, 0.0]
        # Start the turbulence in its stationary distribution
        for i in range(3):
            self.turb[i] = rng.normal(0.0, params.turbulence_sigma_mps[i]) if params.turbulence_sigma_mps[i] > 0 else 0.0

    def update(self, t: float, dt: float) -> tuple:
        p = self.p
        a = math.exp(-dt / p.turbulence_tau_s) if p.turbulence_tau_s > 0 else 0.0
        for i in range(3):
            s = p.turbulence_sigma_mps[i]
            if s > 0:
                self.turb[i] = a * self.turb[i] + s * math.sqrt(1 - a * a) * self.rng.normal()
        w = [p.mean_mps[i] + self.turb[i] for i in range(3)]
        for g in p.gusts:
            if g.t <= t < g.t + g.duration_s:
                shape = 0.5 * (1 - math.cos(2 * math.pi * (t - g.t) / g.duration_s))
                for i in range(3):
                    w[i] += g.peak_mps[i] * shape
        return (w[0], w[1], w[2])


class Terrain:
    """Flat ground at z = 0 with axis-aligned boxes standing on it (top faces only)."""

    def __init__(self, boxes):
        self.boxes = []
        for b in boxes:
            cx, cy = b.center_m
            sx, sy = b.size_m
            self.boxes.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, b.height_m))

    def height(self, x: float, y: float) -> float:
        h = 0.0
        for x0, x1, y0, y1, bh in self.boxes:
            if x0 <= x <= x1 and y0 <= y <= y1 and bh > h:
                h = bh
        return h

    def raycast(self, origin, direction, max_range: float):
        """Distance along direction to the first surface below, or None."""
        dz = direction[2]
        if dz >= -1e-6:
            return None
        best = None
        for top in [0.0] + [b[4] for b in self.boxes]:
            t = (top - origin[2]) / dz
            if t < 0 or t > max_range:
                continue
            x = origin[0] + direction[0] * t
            y = origin[1] + direction[1] * t
            if self.height(x, y) == top and (best is None or t < best):
                best = t
        return best


class Environment:
    def __init__(self, scenario: Scenario, rng):
        self.gravity = scenario.gravity_mps2
        self.air_density = scenario.air_density_kg_m3
        self.wind = Wind(scenario.wind, rng)
        self.terrain = Terrain(scenario.terrain)
        self.wind_now = scenario.wind.mean_mps
