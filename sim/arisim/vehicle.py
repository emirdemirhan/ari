"""Quadrotor rigid-body dynamics with rotor, motor, battery, aerodynamic and ground-contact models.

State (17 floats): position (world, m), velocity (world, m/s), attitude quaternion (body ->
world), body angular rate (rad/s), and the four rotor speeds (rad/s). Integrated with RK4;
motor commands and wind are held constant over one physics substep.

Forces and torques per rotor i, at hub position r_i in the body frame:
  thrust        T_i = k_t w_i^2 * efficiency_i * (1 - k_climb * u_z) * ground_effect(h_i)
  rotor drag    F_i = -k_d w_i (u_x, u_y)        u = airspeed at the hub, body frame
  yaw torque    -s_i k_q w_i^2  - s_i J_r dw_i/dt   (s_i = +1 for a counter-clockwise rotor)
  gyroscopic    -(omega x h_i), h_i = (0, 0, s_i J_r w_i)
plus quadratic body drag, rotational damping, gravity and spring-damper ground contact with
Coulomb friction at the landing-gear points.
"""

from __future__ import annotations

import math

from .config import VehicleParams
from .environment import Environment
from .mathutil import q_from_euler, q_normalize, q_to_rotmat, mt_mul_v, tilt_angle


# Open-circuit voltage of one LiPo cell versus state of charge
_LIPO_OCV = [(0.0, 3.27), (0.05, 3.61), (0.1, 3.69), (0.2, 3.73), (0.3, 3.77), (0.4, 3.79),
             (0.5, 3.82), (0.6, 3.87), (0.7, 3.92), (0.8, 3.98), (0.9, 4.06), (1.0, 4.20)]


def lipo_ocv(soc: float) -> float:
    soc = min(1.0, max(0.0, soc))
    for (s0, v0), (s1, v1) in zip(_LIPO_OCV, _LIPO_OCV[1:]):
        if soc <= s1:
            return v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    return _LIPO_OCV[-1][1]


class Vehicle:
    def __init__(self, vp: VehicleParams, env: Environment, initial_position, initial_yaw_rad: float):
        self.vp = vp
        self.env = env
        g = env.gravity

        b, r, m = vp.body, vp.rotor, vp.motor
        self.mass = b.mass_kg
        self.inertia = tuple(b.inertia_kg_m2)
        self.drag_area = tuple(b.drag_area_m2)
        self.rot_damp = tuple(b.rotational_damping_nms)

        self.omega_max = r.max_rpm * 2 * math.pi / 60
        self.k_t = r.max_thrust_n / self.omega_max ** 2
        self.k_q = r.torque_to_thrust_m * self.k_t
        self.omega_hover = math.sqrt(self.mass * g / (4 * self.k_t))
        # Lumped rotor drag: 4 k_d w_hover = m * accel-per-airspeed at hover
        self.k_d = self.mass * r.rotor_drag_accel_per_mps / (4 * self.omega_hover)
        self.j_rotor = r.inertia_kg_m2
        self.k_climb = r.climb_thrust_loss_per_mps
        self.radius = r.diameter_m / 2
        self.ground_effect = r.ground_effect
        self.rotor_pos = [tuple(p) for p in r.positions_m]
        self.spin = [1.0 if s == "ccw" else -1.0 for s in r.spin]

        self.tau_up = m.time_constant_up_s
        self.tau_down = m.time_constant_down_s
        self.esc_exp = m.esc_curve_exponent
        self.start_cmd = m.start_command
        self.v_ref = m.reference_voltage_v
        self.voltage_scaling = m.voltage_scaling

        gear = vp.gear
        self.contacts = [tuple(c) for c in gear.contact_points_m]
        self.k_gear = gear.stiffness_n_per_m
        self.c_gear = gear.damping_ns_per_m
        self.mu = gear.friction

        bat = vp.battery
        self.battery_enabled = bat.enabled
        self.soc = bat.initial_soc
        self.current = bat.aux_current_a
        self.voltage = bat.cells * lipo_ocv(self.soc) if bat.enabled else self.v_ref

        # Faults
        self.efficiency = [1.0, 1.0, 1.0, 1.0]

        # Initial state: on the ground unless the scenario starts higher
        x0, y0, z0 = initial_position
        rest_z = env.terrain.height(x0, y0) - min(c[2] for c in self.contacts)
        q = q_from_euler(0.0, 0.0, initial_yaw_rad)
        self.state = [x0, y0, max(z0, rest_z), 0.0, 0.0, 0.0, q[0], q[1], q[2], q[3],
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.omega_target = [0.0, 0.0, 0.0, 0.0]
        self.commands = [0.0, 0.0, 0.0, 0.0]

        # Latest derived quantities (updated after every substep)
        self.accel_world = (0.0, 0.0, 0.0)
        self.specific_force_body = (0.0, 0.0, g)
        self.ang_accel = (0.0, 0.0, 0.0)
        self.in_contact = True
        self.contact_speed = 0.0
        self.rotor_phase = [0.0, 1.3, 2.1, 4.4]

    # ------------------------------------------------------------------------------------------

    @property
    def position(self):
        return tuple(self.state[0:3])

    @property
    def velocity(self):
        return tuple(self.state[3:6])

    @property
    def quaternion(self):
        return tuple(self.state[6:10])

    @property
    def rates(self):
        return tuple(self.state[10:13])

    @property
    def rotor_speeds(self):
        return tuple(self.state[13:17])

    def set_commands(self, percent):
        """Motor commands M1..M4 in percent, as sent by the flight controller."""
        self.commands = [min(100.0, max(0.0, c)) for c in percent]
        scale = (self.voltage / self.v_ref) if self.voltage_scaling else 1.0
        for i, c in enumerate(self.commands):
            u = c / 100.0
            self.omega_target[i] = 0.0 if u < self.start_cmd else self.omega_max * scale * u ** self.esc_exp

    # ------------------------------------------------------------------------------------------

    def _derivative(self, x, wind):
        px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz = x[:13]
        R = q_to_rotmat((qw, qx, qy, qz))
        vb = mt_mul_v(R, (vx - wind[0], vy - wind[1], vz - wind[2]))
        terrain = self.env.terrain

        fx = fy = fz = 0.0
        tx = ty = tz = 0.0
        domega = [0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            om = x[13 + i]
            tgt = self.omega_target[i]
            dom = (tgt - om) / (self.tau_up if tgt > om else self.tau_down)
            domega[i] = dom
            if om < 0.0:
                om = 0.0

            rx, ry, rz = self.rotor_pos[i]
            ux = vb[0] + wy * rz - wz * ry
            uy = vb[1] + wz * rx - wx * rz
            uz = vb[2] + wx * ry - wy * rx

            eff = self.efficiency[i]
            thrust = self.k_t * om * om * eff
            loss = 1.0 - self.k_climb * uz
            thrust *= loss if loss > 0.0 else 0.0

            if self.ground_effect:
                hx = px + R[0] * rx + R[1] * ry + R[2] * rz
                hy = py + R[3] * rx + R[4] * ry + R[5] * rz
                hz = pz + R[6] * rx + R[7] * ry + R[8] * rz - terrain.height(hx, hy)
                # Only meaningful while the rotor points roughly down at the ground
                if R[8] > 0.5:
                    h = max(hz / R[8], 0.5 * self.radius)
                    ratio = self.radius / (4.0 * h)
                    thrust /= (1.0 - ratio * ratio)

            dfx = -self.k_d * om * ux
            dfy = -self.k_d * om * uy

            fx += dfx
            fy += dfy
            fz += thrust

            tx += ry * thrust - rz * dfy
            ty += rz * dfx - rx * thrust
            tz += rx * dfy - ry * dfx

            s = self.spin[i]
            tz += -s * self.k_q * om * om * eff - s * self.j_rotor * dom
            h_rot = s * self.j_rotor * om
            tx += -wy * h_rot
            ty += wx * h_rot

        # Quadratic body drag
        rho_half = 0.5 * self.env.air_density
        speed = math.sqrt(vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2])
        fx -= rho_half * self.drag_area[0] * speed * vb[0]
        fy -= rho_half * self.drag_area[1] * speed * vb[1]
        fz -= rho_half * self.drag_area[2] * speed * vb[2]

        tx -= self.rot_damp[0] * wx
        ty -= self.rot_damp[1] * wy
        tz -= self.rot_damp[2] * wz

        # Ground contact (world frame forces)
        cfx = cfy = cfz = 0.0
        in_contact = False
        contact_speed = 0.0
        for cx, cy, cz in self.contacts:
            wpx = px + R[0] * cx + R[1] * cy + R[2] * cz
            wpy = py + R[3] * cx + R[4] * cy + R[5] * cz
            wpz = pz + R[6] * cx + R[7] * cy + R[8] * cz
            pen = terrain.height(wpx, wpy) - wpz
            if pen <= 0.0:
                continue
            in_contact = True
            # Point velocity: v + R (omega x c)
            lx, ly, lz = wy * cz - wz * cy, wz * cx - wx * cz, wx * cy - wy * cx
            pvx = vx + R[0] * lx + R[1] * ly + R[2] * lz
            pvy = vy + R[3] * lx + R[4] * ly + R[5] * lz
            pvz = vz + R[6] * lx + R[7] * ly + R[8] * lz
            contact_speed = max(contact_speed, -pvz)
            fn = self.k_gear * pen - self.c_gear * pvz
            if fn < 0.0:
                fn = 0.0
            vt = math.hypot(pvx, pvy)
            # Regularised Coulomb friction (viscous below 5 cm/s)
            k = self.mu * fn / max(vt, 0.05)
            gx, gy, gz = -k * pvx, -k * pvy, fn
            cfx += gx
            cfy += gy
            cfz += gz
            # Torque about the CG, in the body frame: c x (R' F)
            bx = R[0] * gx + R[3] * gy + R[6] * gz
            by = R[1] * gx + R[4] * gy + R[7] * gz
            bz = R[2] * gx + R[5] * gy + R[8] * gz
            tx += cy * bz - cz * by
            ty += cz * bx - cx * bz
            tz += cx * by - cy * bx

        m = self.mass
        ax = (R[0] * fx + R[1] * fy + R[2] * fz + cfx) / m
        ay = (R[3] * fx + R[4] * fy + R[5] * fz + cfy) / m
        az = (R[6] * fx + R[7] * fy + R[8] * fz + cfz) / m - self.env.gravity

        ixx, iyy, izz = self.inertia
        dwx = (tx - (wy * izz * wz - wz * iyy * wy)) / ixx
        dwy = (ty - (wz * ixx * wx - wx * izz * wz)) / iyy
        dwz = (tz - (wx * iyy * wy - wy * ixx * wx)) / izz

        dqw = 0.5 * (-qx * wx - qy * wy - qz * wz)
        dqx = 0.5 * (qw * wx + qy * wz - qz * wy)
        dqy = 0.5 * (qw * wy - qx * wz + qz * wx)
        dqz = 0.5 * (qw * wz + qx * wy - qy * wx)

        dx = [vx, vy, vz, ax, ay, az, dqw, dqx, dqy, dqz, dwx, dwy, dwz,
              domega[0], domega[1], domega[2], domega[3]]
        return dx, (ax, ay, az), (dwx, dwy, dwz), in_contact, contact_speed

    def substep(self, dt: float, wind):
        x = self.state
        k1, *_ = self._derivative(x, wind)
        x2 = [a + 0.5 * dt * b for a, b in zip(x, k1)]
        k2, *_ = self._derivative(x2, wind)
        x3 = [a + 0.5 * dt * b for a, b in zip(x, k2)]
        k3, *_ = self._derivative(x3, wind)
        x4 = [a + dt * b for a, b in zip(x, k3)]
        k4, *_ = self._derivative(x4, wind)
        x = [a + dt / 6.0 * (b + 2 * c + 2 * d + e) for a, b, c, d, e in zip(x, k1, k2, k3, k4)]

        q = q_normalize((x[6], x[7], x[8], x[9]))
        x[6:10] = q
        for i in range(13, 17):
            if x[i] < 0.0:
                x[i] = 0.0
        self.state = x

        for i in range(4):
            self.rotor_phase[i] = (self.rotor_phase[i] + x[13 + i] * dt * self.spin[i]) % (2 * math.pi)

        # Accelerations at the new state, for the IMU
        _, acc, dw, contact, cspeed = self._derivative(x, wind)
        self.accel_world = acc
        self.ang_accel = dw
        self.in_contact = contact
        self.contact_speed = cspeed
        R = q_to_rotmat(q)
        self.specific_force_body = mt_mul_v(R, (acc[0], acc[1], acc[2] + self.env.gravity))

    def update_battery(self, dt: float):
        if not self.battery_enabled:
            return
        bat = self.vp.battery
        power = sum(self.k_q * om ** 3 * eff for om, eff in zip(self.rotor_speeds, self.efficiency))
        self.current = power / (bat.drive_efficiency * max(self.voltage, 1.0)) + bat.aux_current_a
        self.soc -= self.current * dt / (3.6 * bat.capacity_mah)
        self.voltage = bat.cells * lipo_ocv(self.soc) - self.current * bat.internal_resistance_ohm

    def tilt(self) -> float:
        return tilt_angle(self.quaternion)
