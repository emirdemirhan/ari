"""Sensor model checks: noise levels, scaling, geometry, latency."""

import math

import numpy as np
import pytest

from arisim import config
from arisim.config import ImuParams, RangefinderParams, Scenario
from arisim.environment import Environment, Terrain
from arisim.mathutil import q_from_euler
from arisim.sensors import Imu, Rangefinder

G = 9.80665
DT = 0.0005


class FakeVehicle:
    def __init__(self, rates=(0.0, 0.0, 0.0), sf=(0.0, 0.0, G), q=(1.0, 0.0, 0.0, 0.0), pos=(0.0, 0.0, 1.0)):
        self.rates = rates
        self.specific_force_body = sf
        self.ang_accel = (0.0, 0.0, 0.0)
        self.rotor_speeds = (0.0, 0.0, 0.0, 0.0)
        self.rotor_phase = [0.0] * 4
        self.quaternion = q
        self.position = pos


def quiet_imu(**kw):
    p = ImuParams(gyro_bias_sigma_dps=0, gyro_bias_rw_dps_rts=0, gyro_scale_sigma=0, gyro_misalign_sigma_rad=0,
                  accel_bias_sigma_mg=0, accel_bias_rw_mg_rts=0, accel_scale_sigma=0, accel_misalign_sigma_rad=0,
                  vibration_accel_g=0, vibration_accel_z_g=0, vibration_gyro_dps=0, quantize=False)
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def collect(imu, vehicle, seconds):
    g, a = [], []
    t = 0.0
    last = None
    for _ in range(int(seconds / DT)):
        t += DT
        imu.process(t, vehicle)
        if imu.gyro_out is not last:
            g.append(imu.gyro_out)
            a.append(imu.accel_out)
            last = imu.gyro_out
    return np.array(g), np.array(a)


def test_gyro_and_accel_noise_match_datasheet_density():
    p = quiet_imu()
    imu = Imu(p, np.random.default_rng(1), DT, omega_hover=1000.0)
    g, a = collect(imu, FakeVehicle(), 20.0)
    # White noise through a first-order low-pass: sigma = density * sqrt(pi/2 * f_c)
    exp_g = p.gyro_noise_density_dps_rthz * math.pi / 180 * math.sqrt(math.pi / 2 * p.gyro_lpf_hz)
    exp_a = p.accel_noise_density_ug_rthz * 1e-6 * G * math.sqrt(math.pi / 2 * p.accel_lpf_hz)
    assert g[:, 0].std() == pytest.approx(exp_g, rel=0.15)
    assert a[:, 2].std() == pytest.approx(exp_a, rel=0.15)
    assert a[:, 2].mean() == pytest.approx(G, abs=5e-3)


def test_output_data_rate():
    p = quiet_imu(odr_hz=800)
    imu = Imu(p, np.random.default_rng(1), DT, omega_hover=1000.0)
    g, _ = collect(imu, FakeVehicle(rates=(0.1, 0, 0)), 1.0)
    assert len(g) == pytest.approx(800, abs=2)


def test_quantisation_and_saturation():
    p = quiet_imu(quantize=True, gyro_noise_density_dps_rthz=0, accel_noise_density_ug_rthz=0)
    imu = Imu(p, np.random.default_rng(1), DT, omega_hover=1000.0)
    g, _ = collect(imu, FakeVehicle(rates=(100.0, 0.0, 0.0)), 0.05)   # 5700 dps, beyond 2000 dps
    assert g[-1, 0] == pytest.approx(2000 * math.pi / 180, rel=1e-6)
    lsb = 2000 * math.pi / 180 / 32768
    imu2 = Imu(p, np.random.default_rng(1), DT, omega_hover=1000.0)
    g2, _ = collect(imu2, FakeVehicle(rates=(0.0123, 0.0, 0.0)), 0.05)
    assert abs(g2[-1, 0] / lsb - round(g2[-1, 0] / lsb)) < 1e-9


def test_lever_arm_centripetal_acceleration():
    p = quiet_imu(position_m=(0.1, 0.0, 0.0), accel_noise_density_ug_rthz=0, gyro_noise_density_dps_rthz=0)
    imu = Imu(p, np.random.default_rng(1), DT, omega_hover=1000.0)
    _, a = collect(imu, FakeVehicle(rates=(0.0, 0.0, 5.0)), 0.1)
    assert a[-1, 0] == pytest.approx(-25.0 * 0.1, rel=1e-3)


def test_rangefinder_geometry_latency_and_range_limit():
    p = RangefinderParams(position_m=(0.0, 0.0, 0.0), noise_sigma_m=0, noise_sigma_fraction=0,
                          outlier_probability=0, latency_s=0.03, rate_hz=50, max_range_m=4.0)
    rf = Rangefinder(p, np.random.default_rng(0))
    terrain = Terrain([])
    tilt = 0.3
    veh = FakeVehicle(q=q_from_euler(tilt, 0, 0), pos=(0, 0, 2.0))
    out = []
    for k in range(60):
        rf.update(k * 0.002, veh, terrain)
        out.append((k * 0.002, rf.value, rf.valid, rf.new))
    first = next(o for o in out if o[3])
    assert first[0] == pytest.approx(0.03, abs=0.0021)          # latency
    assert first[1] == pytest.approx(2.0 / math.cos(tilt), rel=1e-9)
    # Above the maximum range: invalid, reported as max range
    rf2 = Rangefinder(p, np.random.default_rng(0))
    veh2 = FakeVehicle(pos=(0, 0, 5.0))
    for k in range(40):
        rf2.update(k * 0.002, veh2, terrain)
    assert not rf2.valid and rf2.value == 4.0


def test_rangefinder_sees_boxes():
    from arisim.config import Box
    terrain = Terrain([Box(center_m=(1.0, 0.0), size_m=(0.5, 0.5), height_m=0.3)])
    assert terrain.height(1.0, 0.0) == 0.3
    assert terrain.height(0.0, 0.0) == 0.0
    assert terrain.raycast((1.0, 0.0, 1.0), (0, 0, -1), 5.0) == pytest.approx(0.7)
    assert terrain.raycast((0.0, 0.0, 1.0), (0, 0, -1), 5.0) == pytest.approx(1.0)
