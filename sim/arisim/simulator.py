"""Lock-step simulation loop.

Per 2 ms control step k (simulation time t_k = k * 2 ms):
  1. pilot commands and faults for t_k
  2. sensor frame from the current sensor outputs -> flight controller (SIL or board)
  3. the reply's motor commands enter the ESC delay line
  4. physics advances 2 ms in substeps; the IMU runs at the substep rate
  5. rangefinder, battery and wind update; one log row
Simulation time only advances when the controller has answered, so a slow link or a slow PC
changes the wall-clock speed but never the result.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from . import protocol
from .config import Scenario, VehicleParams
from .environment import Environment
from .mathutil import m_mul_v, mt_mul_v, q_to_euler, q_to_rotmat, tilt_angle, v_add
from .protocol import SimData
from .scenario import FaultInjector, SetpointTimeline
from .sensors import Barometer, Imu, Rangefinder
from .vehicle import Vehicle

CONTROL_DT = 0.002          # firmware loop period (Ts in stest_wrapper.h)
SUBSTEPS = 4                # physics at 2 kHz
ROLL_PITCH_SCALE = 0.52     # firmware: angle reference = stick * 0.52 rad

LOG_COLUMNS = [
    "t", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw", "p", "q", "r",
    "agl", "terrain",
    "ref_alt", "ref_roll", "ref_pitch", "ref_yaw_rate",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "range", "range_valid", "baro",
    "m1", "m2", "m3", "m4", "rpm1", "rpm2", "rpm3", "rpm4",
    "kf_roll", "kf_pitch", "kf_yaw", "eskf_roll", "eskf_pitch", "eskf_yaw",
    "kf_alt", "kf_vz", "eskf_alt", "eskf_vz", "eskf_u", "eskf_v", "air_u", "air_v",
    "target_vz", "total_thrust", "roll_rate_ref", "pitch_rate_ref",
    "exec_us", "use_eskf", "fc_flags",
    "wind_x", "wind_y", "wind_z", "battery_v", "battery_a", "soc", "contact",
]


@dataclass
class RunResult:
    scenario: Scenario
    vehicle: VehicleParams
    link: str
    log: dict = field(default_factory=dict)        # column -> numpy array
    crashed: bool = False
    crash_reason: str = ""
    crash_time: float | None = None
    wall_time_s: float = 0.0
    link_stats: dict = field(default_factory=dict)
    faults: list = field(default_factory=list)
    link_error: str = ""


class Simulator:
    def __init__(self, vehicle_params: VehicleParams, scenario: Scenario, link, rc_source=None,
                 realtime: bool = False, observer=None):
        self.vp = vehicle_params
        self.sc = scenario
        self.link = link
        self.rng = np.random.default_rng(scenario.seed)
        self.env = Environment(scenario, self.rng)
        self.vehicle = Vehicle(vehicle_params, self.env, scenario.initial_position_m,
                               math.radians(scenario.initial_yaw_deg))
        self.dt_phys = CONTROL_DT / SUBSTEPS
        self.imu = Imu(vehicle_params.imu, self.rng, self.dt_phys, self.vehicle.omega_hover)
        self.range = Rangefinder(vehicle_params.rangefinder, self.rng)
        # Own random stream: adding the barometer must not change the other sensors' noise or the wind
        self.baro = Barometer(vehicle_params.barometer, np.random.default_rng([scenario.seed, 2]))
        self.timeline = SetpointTimeline(scenario)
        self.rc_source = rc_source            # optional live source (joystick); overrides the timeline
        self.faults = FaultInjector(scenario)
        self.realtime = realtime
        self.observer = observer              # optional callable(sim, row) for live display
        delay_steps = max(0, round(vehicle_params.motor.esc_delay_s / CONTROL_DT))
        self.cmd_delay = deque([(0.0, 0.0, 0.0, 0.0)] * delay_steps)
        self.stop_requested = False

        # Prime the sensors with the resting state
        self.vehicle.substep(self.dt_phys, self.env.wind_now)
        self.imu.process(0.0, self.vehicle)
        self.range.prime(self.vehicle, self.env.terrain)
        self.range.update(0.0, self.vehicle, self.env.terrain)
        self.baro.update(0.0, self.vehicle, self.env.wind_now, 0.0)

    def run(self) -> RunResult:
        sc = self.sc
        steps = int(round(sc.duration_s / CONTROL_DT))
        rows = np.zeros((steps, len(LOG_COLUMNS)))
        res = RunResult(scenario=sc, vehicle=self.vp, link=self.link.name)
        v = self.vehicle
        wall0 = time.perf_counter()
        last_k = steps

        for k in range(steps):
            t = k * CONTROL_DT
            if self.stop_requested:
                last_k = k
                break

            ref = self.rc_source.get(t) if self.rc_source else self.timeline.get(t)
            self.faults.apply(t, v, self.imu, self.range, self.baro)

            roll, pitch, yaw = q_to_euler(v.quaternion)
            flags = (protocol.SIM_FLAG_RANGE_VALID if self.range.valid else 0) | \
                    (protocol.SIM_FLAG_RANGE_NEW if self.range.new else 0) | \
                    (protocol.SIM_FLAG_BARO_VALID if self.baro.valid else 0) | \
                    (protocol.SIM_FLAG_BARO_NEW if self.baro.new else 0)
            sim = SimData(seq=k + 1, time_us=int(round(t * 1e6)), accel=self.imu.accel_out, gyro=self.imu.gyro_out,
                          altitude_z=self.range.value, baro_alt=self.baro.value, rc_roll=ref["roll"], rc_pitch=ref["pitch"],
                          rc_yaw_rate=ref["yaw_rate"], rc_thrust=ref["altitude_m"],
                          true_rpy=(roll, pitch, yaw), flags=flags)
            try:
                fc = self.link.step(sim)
            except Exception as e:          # keep everything flown so far
                res.link_error = str(e)
                last_k = k
                break

            self.cmd_delay.append(fc.motor)
            v.set_commands(self.cmd_delay.popleft())

            wind = self.env.wind.update(t, CONTROL_DT)
            self.env.wind_now = wind
            for s in range(SUBSTEPS):
                ts = t + (s + 1) * self.dt_phys
                v.substep(self.dt_phys, wind)
                self.imu.process(ts, v)
            t_next = t + CONTROL_DT
            self.range.update(t_next, v, self.env.terrain)
            v.update_battery(CONTROL_DT)

            # Log the state at t_next together with the command computed at t_k
            pos, vel, rates = v.position, v.velocity, v.rates
            r2, p2, y2 = q_to_euler(v.quaternion)
            # True velocity relative to the air in the body frame, for the ESKF's velocity estimate
            air_b = mt_mul_v(q_to_rotmat(v.quaternion), (vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2]))
            # Height that the altitude loop regulates: rangefinder position above the terrain below it
            sensor = v_add(pos, m_mul_v(q_to_rotmat(v.quaternion), self.vp.rangefinder.position_m))
            terrain_h = self.env.terrain.height(sensor[0], sensor[1])
            agl = sensor[2] - terrain_h
            self.baro.update(t_next, v, wind, agl)
            omegas = v.rotor_speeds
            row = (t_next, *pos, *vel, r2, p2, y2, *rates, agl, terrain_h,
                   ref["altitude_m"], ref["roll"] * ROLL_PITCH_SCALE, ref["pitch"] * ROLL_PITCH_SCALE, ref["yaw_rate"],
                   *sim.accel, *sim.gyro, sim.altitude_z, 1.0 if self.range.valid else 0.0, sim.baro_alt,
                   *fc.motor, *(o * 60 / (2 * math.pi) for o in omegas),
                   *fc.kf_rpy, *fc.eskf_rpy, fc.kf_alt, fc.kf_vz, fc.eskf_alt, fc.eskf_vz, *fc.eskf_vel, *air_b[:2],
                   fc.target_vz, fc.total_thrust, fc.roll_rate_ref, fc.pitch_rate_ref,
                   fc.exec_cycles / 100.0, 1.0 if fc.flags & protocol.FC_FLAG_USE_ESKF else 0.0, float(fc.flags),
                   *wind, v.voltage, v.current, v.soc, 1.0 if v.in_contact else 0.0)
            rows[k] = row

            if not res.crashed:
                reason = self._crash_check(v)
                if reason:
                    res.crashed, res.crash_reason, res.crash_time = True, reason, t_next
            if res.crashed and t_next > res.crash_time + 1.0:
                last_k = k + 1
                break

            if self.observer:
                self.observer(self, row)
            if self.realtime:
                lag = (wall0 + t_next) - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)

        rows = rows[:last_k]
        res.log = {name: rows[:, i] for i, name in enumerate(LOG_COLUMNS)}
        res.wall_time_s = time.perf_counter() - wall0
        res.link_stats = dict(self.link.stats)
        res.faults = list(self.faults.log)
        return res

    @staticmethod
    def _crash_check(v: Vehicle) -> str:
        tilt = tilt_angle(v.quaternion)
        if tilt > math.radians(120):
            return f"flipped (tilt {math.degrees(tilt):.0f} deg)"
        if v.in_contact and v.contact_speed > 3.0:
            return f"hard impact ({v.contact_speed:.1f} m/s)"
        if v.in_contact and tilt > math.radians(60):
            return f"tipped over on the ground (tilt {math.degrees(tilt):.0f} deg)"
        return ""
