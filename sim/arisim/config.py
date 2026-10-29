"""Vehicle and scenario configuration: TOML files loaded into dataclasses.

A TOML file may name a `base` file (path relative to itself); its tables are deep-merged on
top of the base. Unknown keys are errors, so a typo cannot silently fall back to a default.
"""

from __future__ import annotations

import dataclasses
import math
import tomllib
from dataclasses import dataclass, field
from pathlib import Path


# ----------------------------------------------------------------------------------------------
# Vehicle
# ----------------------------------------------------------------------------------------------

@dataclass
class BodyParams:
    mass_kg: float = 0.5
    inertia_kg_m2: tuple = (0.005, 0.005, 0.01)          # Ixx, Iyy, Izz about the CG
    drag_area_m2: tuple = (0.008, 0.008, 0.015)           # Cd * A per body axis (quadratic body drag)
    rotational_damping_nms: tuple = (0.0005, 0.0005, 0.0005)


@dataclass
class RotorParams:
    # Rotor hub positions in the body frame (m) and spin directions, in firmware motor order M1..M4
    positions_m: list = field(default_factory=lambda: [
        (0.13, -0.22, 0.023),     # M1 front-right
        (-0.13, 0.22, 0.023),     # M2 rear-left
        (0.13, 0.22, 0.023),      # M3 front-left
        (-0.13, -0.22, 0.023),    # M4 rear-right
    ])
    spin: list = field(default_factory=lambda: ["ccw", "ccw", "cw", "cw"])
    diameter_m: float = 0.127
    max_thrust_n: float = 3.0                # per rotor, command 100 %, reference voltage, static, no ground effect
    max_rpm: float = 17000.0                 # rotor speed at command 100 % and reference voltage
    torque_to_thrust_m: float = 0.016        # aerodynamic yaw torque / thrust
    inertia_kg_m2: float = 6e-6              # prop + motor bell, about the spin axis
    rotor_drag_accel_per_mps: float = 0.35   # horizontal drag of the whole vehicle at hover, as accel per airspeed (1/s)
    climb_thrust_loss_per_mps: float = 0.05  # fractional thrust loss per m/s of axial inflow (climb)
    ground_effect: bool = True


@dataclass
class MotorParams:
    time_constant_up_s: float = 0.030
    time_constant_down_s: float = 0.050      # props spin down slower without active braking
    esc_curve_exponent: float = 0.5          # rotor speed ~ command^e: 0.5 -> thrust linear in command, 1.0 -> rpm linear
    start_command: float = 0.02              # below this command (0..1) the motor stops
    reference_voltage_v: float = 11.55       # battery voltage at which max_thrust_n / max_rpm were measured
    voltage_scaling: bool = True             # rotor speed scales with battery voltage (no ESC compensation)
    esc_delay_s: float = 0.002               # command latency: compute + ESC protocol


@dataclass
class BatteryParams:
    enabled: bool = True
    cells: int = 3
    capacity_mah: float = 1300.0
    internal_resistance_ohm: float = 0.03
    initial_soc: float = 0.95
    drive_efficiency: float = 0.70           # mechanical rotor power / electrical power
    aux_current_a: float = 0.3               # avionics


@dataclass
class GearParams:
    contact_points_m: list = field(default_factory=lambda: [
        (0.06, 0.06, -0.05), (-0.06, 0.06, -0.05), (0.06, -0.06, -0.05), (-0.06, -0.06, -0.05),
        (0.0, 0.0, 0.04),                    # top of the body, so an upside-down vehicle does not sink
    ])
    stiffness_n_per_m: float = 2000.0
    damping_ns_per_m: float = 25.0
    friction: float = 0.6


@dataclass
class ImuParams:
    # Defaults: Bosch BMI160 datasheet, gyro 2000 dps / accel 16 g, ODR 800 Hz, normal filter mode
    position_m: tuple = (0.0, 0.0, 0.0)
    odr_hz: float = 800.0
    quantize: bool = True

    gyro_range_dps: float = 2000.0
    gyro_noise_density_dps_rthz: float = 0.007
    gyro_bias_sigma_dps: float = 1.0          # power-on zero-rate offset, 1 sigma (datasheet +-3 dps)
    gyro_bias_rw_dps_rts: float = 0.002       # in-run bias random walk
    gyro_scale_sigma: float = 0.005
    gyro_misalign_sigma_rad: float = 0.005
    gyro_lpf_hz: float = 254.6

    accel_range_g: float = 16.0
    accel_noise_density_ug_rthz: float = 180.0
    accel_bias_sigma_mg: float = 15.0
    accel_bias_rw_mg_rts: float = 0.05
    accel_scale_sigma: float = 0.005
    accel_misalign_sigma_rad: float = 0.005
    accel_lpf_hz: float = 162.0

    # Motor vibration seen by the IMU, at hover rotor speed (scales with rotor speed squared)
    vibration_accel_g: float = 0.3            # in the rotor plane, at the rotor frequency (imbalance)
    vibration_accel_z_g: float = 0.15         # along z, at the blade-pass frequency (2x)
    vibration_gyro_dps: float = 1.5


@dataclass
class RangefinderParams:
    # Defaults: ST VL53L1X-class time-of-flight sensor in long-distance mode
    enabled: bool = True
    position_m: tuple = (0.12, 0.0, 0.0)
    direction_body: tuple = (0.0, 0.0, -1.0)
    rate_hz: float = 50.0
    min_range_m: float = 0.04
    max_range_m: float = 4.0
    noise_sigma_m: float = 0.005
    noise_sigma_fraction: float = 0.01        # extra noise proportional to range
    latency_s: float = 0.030
    outlier_probability: float = 0.002
    out_of_range: str = "max"                 # "max": report max_range_m, "hold": repeat last valid sample


@dataclass
class BarometerParams:
    # Defaults: Bosch BMP388 / Infineon DPS310-class pressure sensor, high oversampling
    enabled: bool = True
    rate_hz: float = 50.0
    latency_s: float = 0.010
    noise_sigma_m: float = 0.04               # sensor noise at rest
    propwash_sigma_m: float = 0.10            # extra noise in flight (rotor downwash, turbulence), x rotor speed^2 / hover
    drift_m_per_sqrt_s: float = 0.005         # random walk: temperature and weather
    initial_offset_m: float = 120.0           # takeoff point above sea level; the firmware only uses changes
    dynamic_pressure_coeff: float = 0.3       # static-port error -c v^2 / (2 g), v = horizontal air speed
    ground_effect_m: float = 0.15             # downwash raises the pressure near the ground: reads this much low at 0 m
    ground_effect_height_m: float = 0.6       # ... fading to zero at this height
    resolution_m: float = 0.01


@dataclass
class VehicleParams:
    name: str = "vehicle"
    description: str = ""
    body: BodyParams = field(default_factory=BodyParams)
    rotor: RotorParams = field(default_factory=RotorParams)
    motor: MotorParams = field(default_factory=MotorParams)
    battery: BatteryParams = field(default_factory=BatteryParams)
    gear: GearParams = field(default_factory=GearParams)
    imu: ImuParams = field(default_factory=ImuParams)
    rangefinder: RangefinderParams = field(default_factory=RangefinderParams)
    barometer: BarometerParams = field(default_factory=BarometerParams)


# ----------------------------------------------------------------------------------------------
# Scenario
# ----------------------------------------------------------------------------------------------

@dataclass
class Setpoint:
    """Pilot command from time t, in firmware conventions (see hil_protocol.h)."""
    t: float = 0.0
    altitude_m: float | None = None          # altitude reference (sent as rc_thrust)
    roll: float | None = None                # stick -1..1, positive rolls right
    pitch: float | None = None               # stick -1..1, positive pitches nose down (forward)
    yaw_rate: float | None = None            # rad/s, positive counter-clockwise seen from above
    ramp_s: float = 0.0                      # linear transition from the previous value


@dataclass
class Gust:
    t: float = 0.0
    duration_s: float = 1.0
    peak_mps: tuple = (0.0, 0.0, 0.0)        # 1-cosine gust, world frame


@dataclass
class WindParams:
    mean_mps: tuple = (0.0, 0.0, 0.0)
    turbulence_sigma_mps: tuple = (0.0, 0.0, 0.0)
    turbulence_tau_s: float = 1.0            # correlation time of the Gauss-Markov turbulence
    gusts: list = field(default_factory=list)


@dataclass
class Box:
    center_m: tuple = (0.0, 0.0)
    size_m: tuple = (1.0, 1.0)
    height_m: float = 0.3


@dataclass
class Fault:
    t: float = 0.0
    type: str = ""                           # motor_scale | gyro_bias_step | accel_bias_step | range_blocked | baro_blocked
    motor: int = 0                           # 1..4 for motor_scale
    value: object = 0.0                      # scale (motor_scale) or [x, y, z] (bias steps, dps / mg)
    duration_s: float = 0.0                  # range_blocked, baro_blocked


@dataclass
class Criterion:
    metric: str = ""
    max: float | None = None
    min: float | None = None
    window: tuple | None = None              # (t_start, t_end) in s; None = whole run
    estimator: str = "active"                # kf | eskf | active (the one feeding the controller)


@dataclass
class Scenario:
    name: str = "scenario"
    description: str = ""
    duration_s: float = 20.0
    seed: int = 1
    vehicle: str = "x3_500g"
    gravity_mps2: float = 9.80665
    air_density_kg_m3: float = 1.225
    initial_position_m: tuple = (0.0, 0.0, 0.0)  # z = 0: resting on the ground
    initial_yaw_deg: float = 0.0
    setpoints: list = field(default_factory=lambda: [Setpoint()])
    wind: WindParams = field(default_factory=WindParams)
    terrain: list = field(default_factory=list)
    faults: list = field(default_factory=list)
    criteria: list = field(default_factory=list)
    vehicle_overrides: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------------------------------

class ConfigError(Exception):
    pass


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_toml(path: Path) -> dict:
    path = Path(path)
    with open(path, "rb") as f:
        data = tomllib.load(f)
    base = data.pop("base", None)
    if base is not None:
        data = _deep_merge(load_toml(path.parent / base), data)
    return data


_NESTED = {
    VehicleParams: {"body": BodyParams, "rotor": RotorParams, "motor": MotorParams, "battery": BatteryParams,
                    "gear": GearParams, "imu": ImuParams, "rangefinder": RangefinderParams,
                    "barometer": BarometerParams},
    Scenario: {"wind": WindParams},
}
_LISTS = {
    Scenario: {"setpoints": Setpoint, "terrain": Box, "faults": Fault, "criteria": Criterion},
    WindParams: {"gusts": Gust},
}


def from_dict(cls, data: dict, where: str = ""):
    names = {f.name: f for f in dataclasses.fields(cls)}
    kwargs = {}
    for key, value in data.items():
        if key not in names:
            raise ConfigError(f"unknown key '{where}{key}' (valid: {', '.join(sorted(names))})")
        if key in _NESTED.get(cls, {}):
            value = from_dict(_NESTED[cls][key], value, f"{where}{key}.")
        elif key in _LISTS.get(cls, {}):
            value = [from_dict(_LISTS[cls][key], item, f"{where}{key}[{i}].") for i, item in enumerate(value)]
        elif isinstance(value, list) and value and all(isinstance(x, list) for x in value):
            value = [tuple(x) for x in value]
        elif isinstance(value, list):
            value = tuple(value)
        kwargs[key] = value
    return cls(**kwargs)


def load_vehicle(path: Path, overrides: dict | None = None) -> VehicleParams:
    data = load_toml(path)
    if overrides:
        data = _deep_merge(data, overrides)
    vp = from_dict(VehicleParams, data)
    validate_vehicle(vp)
    return vp


def load_scenario(path: Path) -> Scenario:
    sc = from_dict(Scenario, load_toml(path))
    if not sc.setpoints:
        sc.setpoints = [Setpoint()]
    sc.setpoints.sort(key=lambda s: s.t)
    return sc


def validate_vehicle(vp: VehicleParams):
    r = vp.rotor
    if len(r.positions_m) != 4 or len(r.spin) != 4:
        raise ConfigError("rotor.positions_m and rotor.spin need exactly 4 entries (M1..M4)")
    for s in r.spin:
        if s not in ("cw", "ccw"):
            raise ConfigError(f"rotor.spin entries must be 'cw' or 'ccw', got '{s}'")
    if vp.rangefinder.out_of_range not in ("max", "hold"):
        raise ConfigError("rangefinder.out_of_range must be 'max' or 'hold'")
    if r.max_thrust_n * 4 <= vp.body.mass_kg * 9.81:
        raise ConfigError("total max thrust does not exceed the weight; the vehicle cannot hover")


def derived_vehicle_values(vp: VehicleParams, gravity: float = 9.80665) -> dict:
    """Numbers worth knowing before a run: hover point, thrust-to-weight, rotor speeds."""
    omega_max = vp.rotor.max_rpm * 2 * math.pi / 60
    k_t = vp.rotor.max_thrust_n / omega_max ** 2
    hover_thrust = vp.body.mass_kg * gravity / 4
    omega_hover = math.sqrt(hover_thrust / k_t)
    # rotor speed ~ command^e at the reference voltage
    hover_cmd = (omega_hover / omega_max) ** (1 / vp.motor.esc_curve_exponent)
    return {
        "thrust_to_weight": 4 * vp.rotor.max_thrust_n / (vp.body.mass_kg * gravity),
        "hover_command_percent_at_reference_voltage": 100 * hover_cmd,
        "hover_rotor_speed_rpm": omega_hover * 60 / (2 * math.pi),
        "hover_rotor_frequency_hz": omega_hover / (2 * math.pi),
        "blade_pass_frequency_hz": 2 * omega_hover / (2 * math.pi),
        "k_thrust_n_per_rad2s2": k_t,
    }
