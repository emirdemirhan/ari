"""Physics model checks against hand calculations."""

import math

import numpy as np
import pytest

from arisim import config
from arisim.config import Scenario
from arisim.environment import Environment
from arisim.mathutil import q_from_euler, q_to_euler, q_to_rotmat, m_mul_v
from arisim.vehicle import Vehicle

SIM = __import__("pathlib").Path(__file__).resolve().parents[1]
G = 9.80665
DT = 0.0005


def make(height=5.0, **overrides):
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml", overrides)
    vp.battery.enabled = False
    vp.rotor.ground_effect = False
    env = Environment(Scenario(), np.random.default_rng(0))
    v = Vehicle(vp, env, (0.0, 0.0, height), 0.0)
    return v, vp


def hover_cmd(v):
    return 100.0 * (v.omega_hover / v.omega_max) ** (1 / v.esc_exp)


def run(v, seconds, cmds=None, wind=(0.0, 0.0, 0.0)):
    if cmds is not None:
        v.set_commands(cmds)
    for _ in range(int(round(seconds / DT))):
        v.substep(DT, wind)


def test_free_fall_without_drag():
    v, _ = make(height=50.0, body={"drag_area_m2": [0, 0, 0]})
    run(v, 1.0, [0, 0, 0, 0])
    assert v.velocity[2] == pytest.approx(-G, abs=1e-6)
    assert v.position[2] == pytest.approx(50.0 - 0.5 * G, abs=1e-4)
    # Free fall: the accelerometer reads zero
    assert max(abs(a) for a in v.specific_force_body) < 1e-6


def test_hover_equilibrium():
    v, _ = make()
    c = hover_cmd(v)
    v.state[13:17] = [v.omega_hover] * 4          # rotors already spun up
    run(v, 3.0, [c] * 4)
    assert abs(v.velocity[2]) < 1e-3
    assert abs(v.position[2] - 5.0) < 1e-3
    assert max(abs(w) for w in v.rates) < 1e-6
    # Hovering accelerometer reads +g along body z
    assert v.specific_force_body[2] == pytest.approx(G, rel=1e-4)


@pytest.mark.parametrize("delta, axis, sign", [
    ((-1, +1, +1, -1), 0, +1),    # left motors (M2, M3) up: left side rises = positive roll
    ((-1, +1, -1, +1), 1, +1),    # rear motors (M2, M4) up: nose down = positive pitch
    ((-1, -1, +1, +1), 2, +1),    # CW motors (M3, M4) up: reaction torque counter-clockwise = positive yaw
])
def test_mixer_signs_match_firmware_conventions(delta, axis, sign):
    v, _ = make()
    c = hover_cmd(v)
    v.state[13:17] = [v.omega_hover] * 4
    run(v, 0.2, [c + 3 * d for d in delta])
    rates = v.rates
    assert math.copysign(1, rates[axis]) == sign
    assert abs(rates[axis]) > 5 * max(abs(rates[i]) for i in range(3) if i != axis)


def test_motor_time_constant():
    v, vp = make()
    run(v, vp.motor.time_constant_up_s, [100, 100, 100, 100])
    # First-order lag: 63.2 % of the step after one time constant
    assert v.rotor_speeds[0] / v.omega_max == pytest.approx(1 - math.exp(-1), rel=0.01)


def test_rotor_drag_decelerates_horizontal_motion():
    v, vp = make(body={"drag_area_m2": [0, 0, 0]})
    v.state[13:17] = [v.omega_hover] * 4
    v.state[3] = 2.0
    v.set_commands([hover_cmd(v)] * 4)
    v.substep(DT, (0.0, 0.0, 0.0))
    # Level attitude, so only rotor drag acts horizontally: a = -c * v
    assert v.accel_world[0] == pytest.approx(-vp.rotor.rotor_drag_accel_per_mps * 2.0, rel=0.02)


def test_wind_is_relative_airspeed():
    v, vp = make(body={"drag_area_m2": [0, 0, 0]})
    v.state[13:17] = [v.omega_hover] * 4
    v.set_commands([hover_cmd(v)] * 4)
    v.substep(DT, (-2.0, 0.0, 0.0))     # still vehicle, wind from +x towards -x
    assert v.accel_world[0] == pytest.approx(-vp.rotor.rotor_drag_accel_per_mps * 2.0, rel=0.02)


def test_resting_on_ground_is_stable():
    v, _ = make(height=0.0)
    z0 = v.position[2]
    run(v, 5.0, [0, 0, 0, 0])
    assert abs(v.position[2] - z0) < 2e-3          # static deflection only
    assert max(abs(x) for x in v.velocity) < 1e-4
    assert v.specific_force_body[2] == pytest.approx(G, rel=1e-3)
    assert v.in_contact


def test_ground_effect_increases_thrust_near_ground():
    v_ge, _ = make(height=0.0)
    v_ge.rotor_ground_effect = True
    v_ge.ground_effect = True
    v_far, _ = make(height=5.0)
    v_far.ground_effect = True
    for v in (v_ge, v_far):
        v.state[13:17] = [v.omega_hover] * 4
        v.set_commands([hover_cmd(v)] * 4)
        v.substep(DT, (0.0, 0.0, 0.0))
    # Near the ground the same rotor speed lifts the vehicle off its gear
    assert v_ge.accel_world[2] > 0.02
    assert abs(v_far.accel_world[2]) < 0.02


def test_battery_sags_under_load():
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    env = Environment(Scenario(), np.random.default_rng(0))
    v = Vehicle(vp, env, (0, 0, 5.0), 0.0)
    v0 = v.voltage
    v.state[13:17] = [v.omega_hover] * 4
    v.update_battery(0.002)
    assert v.current > 5.0                         # hover current of a 500 g quad
    assert v.voltage < v0 - 0.1


def test_torque_free_rotation_conserves_angular_momentum():
    v, _ = make(height=50.0, body={"rotational_damping_nms": [0, 0, 0], "inertia_kg_m2": [0.004, 0.006, 0.01]})
    v.state[10:13] = [1.0, 0.3, 5.0]
    def momentum_world():
        I = v.inertia
        Lb = tuple(I[i] * v.rates[i] for i in range(3))
        return m_mul_v(q_to_rotmat(v.quaternion), Lb)
    L0 = momentum_world()
    run(v, 1.0, [0, 0, 0, 0])
    L1 = momentum_world()
    assert np.allclose(L0, L1, atol=1e-7)


def test_euler_roundtrip():
    for r, p, y in [(0.1, -0.2, 0.3), (-1.0, 0.5, 3.0), (0.0, 1.2, -2.0)]:
        assert np.allclose(q_to_euler(q_from_euler(r, p, y)), (r, p, y), atol=1e-12)
