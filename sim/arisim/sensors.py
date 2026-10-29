"""Sensor models: IMU (gyro + accelerometer), downward rangefinder and barometer.

IMU signal chain, run at the physics substep rate:
  true rate / specific force at the IMU position (lever arm included)
  + motor vibration (rotor-frequency imbalance, blade-pass on z) scaled with rotor speed^2
  -> scale-factor and cross-axis errors (fixed per run)
  + bias (random power-on offset + random walk + injected fault steps)
  + white noise at the datasheet noise density
  -> first-order low-pass at the configured bandwidth
  -> sampled at the output data rate, quantised to the LSB, saturated at the range.
The flight controller receives the latest sample at each control step (sample-and-hold), so
an ODR that does not divide the control rate produces the same jitter as on hardware.
"""

from __future__ import annotations

import math
from collections import deque

from .config import BarometerParams, ImuParams, RangefinderParams
from .mathutil import m_mul_v, q_to_rotmat, v_add, v_cross

G0 = 9.80665
DEG = math.pi / 180.0


def _error_matrix(rng, scale_sigma, misalign_sigma):
    m = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            m[i][j] = (1.0 + rng.normal(0, scale_sigma)) if i == j else rng.normal(0, misalign_sigma)
    return m


def _apply(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


class Imu:
    def __init__(self, p: ImuParams, rng, dt_phys: float, omega_hover: float):
        self.p = p
        self.rng = rng
        self.dt = dt_phys
        self.omega_hover = omega_hover

        self.gyro_bias = [rng.normal(0, p.gyro_bias_sigma_dps * DEG) for _ in range(3)]
        self.accel_bias = [rng.normal(0, p.accel_bias_sigma_mg * 1e-3 * G0) for _ in range(3)]
        self.gyro_fault = [0.0, 0.0, 0.0]
        self.accel_fault = [0.0, 0.0, 0.0]
        self.gyro_m = _error_matrix(rng, p.gyro_scale_sigma, p.gyro_misalign_sigma_rad)
        self.accel_m = _error_matrix(rng, p.accel_scale_sigma, p.accel_misalign_sigma_rad)

        self.alpha_g = 1.0 - math.exp(-2 * math.pi * p.gyro_lpf_hz * dt_phys)
        self.alpha_a = 1.0 - math.exp(-2 * math.pi * p.accel_lpf_hz * dt_phys)

        # White noise per substep, so the low-pass shapes it like the sensor's own filter
        fs_half = 0.5 / dt_phys
        self.noise_g = p.gyro_noise_density_dps_rthz * DEG * math.sqrt(fs_half)
        self.noise_a = p.accel_noise_density_ug_rthz * 1e-6 * G0 * math.sqrt(fs_half)
        self.rw_g = p.gyro_bias_rw_dps_rts * DEG * math.sqrt(dt_phys)
        self.rw_a = p.accel_bias_rw_mg_rts * 1e-3 * G0 * math.sqrt(dt_phys)

        self.range_g = p.gyro_range_dps * DEG
        self.range_a = p.accel_range_g * G0
        self.lsb_g = self.range_g / 32768.0
        self.lsb_a = self.range_a / 32768.0

        self.gyro_f = [0.0, 0.0, 0.0]
        self.accel_f = [0.0, 0.0, G0]
        self.gyro_out = (0.0, 0.0, 0.0)
        self.accel_out = (0.0, 0.0, G0)
        self.period = 1.0 / p.odr_hz
        self.next_sample = 0.0
        self.vib_phase = [rng.uniform(0, 2 * math.pi) for _ in range(4)]

    def _quantize(self, v, lsb, rng_max):
        if self.p.quantize:
            v = round(v / lsb) * lsb
        return min(rng_max, max(-rng_max, v))

    def process(self, t: float, vehicle):
        p = self.p
        w = vehicle.rates
        f = vehicle.specific_force_body
        r = p.position_m
        if r[0] or r[1] or r[2]:
            f = v_add(f, v_add(v_cross(vehicle.ang_accel, r), v_cross(w, v_cross(w, r))))

        # Motor vibration
        vg = [0.0, 0.0, 0.0]
        va = [0.0, 0.0, 0.0]
        if p.vibration_accel_g or p.vibration_accel_z_g or p.vibration_gyro_dps:
            for i, om in enumerate(vehicle.rotor_speeds):
                k = (om / self.omega_hover) ** 2 * 0.5
                ph = vehicle.rotor_phase[i] + self.vib_phase[i]
                c, s = math.cos(ph), math.sin(ph)
                va[0] += k * p.vibration_accel_g * G0 * c
                va[1] += k * p.vibration_accel_g * G0 * s
                va[2] += k * p.vibration_accel_z_g * G0 * math.sin(2 * ph)
                vg[0] += k * p.vibration_gyro_dps * DEG * s
                vg[1] += k * p.vibration_gyro_dps * DEG * c

        n = self.rng.standard_normal(12)
        for i in range(3):
            self.gyro_bias[i] += self.rw_g * n[6 + i]
            self.accel_bias[i] += self.rw_a * n[9 + i]

        g_true = _apply(self.gyro_m, (w[0] + vg[0], w[1] + vg[1], w[2] + vg[2]))
        a_true = _apply(self.accel_m, (f[0] + va[0], f[1] + va[1], f[2] + va[2]))
        for i in range(3):
            gi = g_true[i] + self.gyro_bias[i] + self.gyro_fault[i] + self.noise_g * n[i]
            ai = a_true[i] + self.accel_bias[i] + self.accel_fault[i] + self.noise_a * n[3 + i]
            self.gyro_f[i] += self.alpha_g * (gi - self.gyro_f[i])
            self.accel_f[i] += self.alpha_a * (ai - self.accel_f[i])

        if t + 1e-9 >= self.next_sample:
            self.next_sample += self.period
            self.gyro_out = tuple(self._quantize(v, self.lsb_g, self.range_g) for v in self.gyro_f)
            self.accel_out = tuple(self._quantize(v, self.lsb_a, self.range_a) for v in self.accel_f)


class Rangefinder:
    def __init__(self, p: RangefinderParams, rng):
        self.p = p
        self.rng = rng
        self.period = 1.0 / p.rate_hz
        self.next_sample = 0.0
        self.pending = deque()           # (release_time, value or None)
        self.value = p.max_range_m
        self.valid = False
        self.new = False
        self.blocked_until = -1.0

    def _measure(self, t, vehicle, terrain):
        p = self.p
        if t < self.blocked_until:
            return None
        R = q_to_rotmat(vehicle.quaternion)
        origin = v_add(vehicle.position, m_mul_v(R, p.position_m))
        direction = m_mul_v(R, p.direction_body)
        d = terrain.raycast(origin, direction, p.max_range_m * 1.2)
        if d is None or not (p.min_range_m <= d <= p.max_range_m):
            return None
        if self.rng.uniform() < p.outlier_probability:
            return self.rng.uniform(p.min_range_m, p.max_range_m)
        meas = d + self.rng.normal(0, p.noise_sigma_m + p.noise_sigma_fraction * d)
        return max(p.min_range_m, meas)

    def prime(self, vehicle, terrain):
        """The sensor has been ranging since power-up: start with a delivered sample, not a gap."""
        if not self.p.enabled:
            return
        meas = self._measure(0.0, vehicle, terrain)
        self.valid = meas is not None
        self.value = meas if meas is not None else self.p.max_range_m

    def update(self, t: float, vehicle, terrain):
        p = self.p
        self.new = False
        if not p.enabled:
            return
        if t + 1e-9 >= self.next_sample:
            self.next_sample += self.period
            self.pending.append((t + p.latency_s, self._measure(t, vehicle, terrain)))

        while self.pending and self.pending[0][0] <= t + 1e-9:
            _, meas = self.pending.popleft()
            self.new = True
            if meas is None:
                self.valid = False
                if p.out_of_range == "max":
                    self.value = p.max_range_m
            else:
                self.valid = True
                self.value = meas


class Barometer:
    """Pressure altitude, as a BMP388/DPS310 reports it after the pressure-to-altitude conversion.

    reading = altitude above sea level (takeoff point + height)
              + drift (random walk) + noise (larger with the rotors running: downwash, turbulence)
              - dynamic pressure error (horizontal air speed at the static port)
              - ground effect (downwash raises the pressure close to the ground)
    sampled at rate_hz, delivered after latency_s, quantised to resolution_m.
    """

    def __init__(self, p: BarometerParams, rng):
        self.p = p
        self.rng = rng
        self.period = 1.0 / p.rate_hz
        self.next_sample = 0.0
        self.pending = deque()
        self.drift = 0.0
        self.value = p.initial_offset_m
        self.new = False
        self.valid = p.enabled
        self.blocked_until = -1.0
        self.last_t = 0.0

    def _measure(self, t, vehicle, wind, height_agl):
        p = self.p
        dt = t - self.last_t
        self.last_t = t
        if dt > 0:
            self.drift += self.rng.normal(0, p.drift_m_per_sqrt_s * math.sqrt(dt))
        if t < self.blocked_until:
            return None
        vel = vehicle.velocity
        air_h = math.hypot(vel[0] - wind[0], vel[1] - wind[1])
        spin = sum(o * o for o in vehicle.rotor_speeds) / (4 * vehicle.omega_hover ** 2)
        sigma = math.hypot(p.noise_sigma_m, p.propwash_sigma_m * min(spin, 2.0))
        ge = 0.0
        if spin > 0.1 and height_agl < p.ground_effect_height_m:
            ge = p.ground_effect_m * (1.0 - max(height_agl, 0.0) / p.ground_effect_height_m)
        alt = (p.initial_offset_m + vehicle.position[2] + self.drift + self.rng.normal(0, sigma)
               - p.dynamic_pressure_coeff * air_h * air_h / (2 * G0) - ge)
        return round(alt / p.resolution_m) * p.resolution_m

    def update(self, t: float, vehicle, wind, height_agl):
        p = self.p
        self.new = False
        if not p.enabled:
            return
        if t + 1e-9 >= self.next_sample:
            self.next_sample += self.period
            self.pending.append((t + p.latency_s, self._measure(t, vehicle, wind, height_agl)))
        while self.pending and self.pending[0][0] <= t + 1e-9:
            _, meas = self.pending.popleft()
            self.new = True
            self.valid = meas is not None
            if meas is not None:
                self.value = meas
