#ifndef ESKF_H
#define ESKF_H

#include <math.h>
#include "ari_math/ari_matrix.h"


/**
 * @brief Error-state Kalman filter (ESKF) for a quadrotor: attitude, gyro bias and the
 *        horizontal velocity through the air, using the rotor-drag model.
 *
 * Nominal state:  q (unit quaternion, body -> world, scalar first), gyro bias b_g (rad/s),
 *                 u, v (velocity relative to the air along body x and y, m/s).
 * Error state (8): d_theta (3, rotation error in the body frame: q_true = q (x) dq(d_theta)),
 *                  d_bg (3), d_u, d_v.
 * The nominal state is propagated with the full nonlinear model; the filter estimates the small
 * error, injects it into the nominal state after every update and resets it to zero.
 *
 * Why the drag model: in flight a multirotor's accelerometer does not measure gravity. Thrust is
 * along body z only, and in the rotor plane the only force is rotor drag, proportional to the
 * air velocity:  a_x = -k u,  a_y = -k v  (k grows with rotor speed, ~ sqrt(thrust)).
 * Treating a_x, a_y as gravity (as a classic attitude filter does) reads every acceleration as
 * tilt. Here they are a velocity measurement, and tilt becomes observable through the dynamics:
 *     du/dt = g_b,x - k u - (q w - r v),   dv/dt = g_b,y - k v - (r u - p w)
 * with g_b = R' (0, 0, -g) gravity in the body frame and w the body z velocity (from the
 * altitude filter). A tilt error makes the predicted velocity, and so the predicted drag,
 * disagree with the accelerometer.
 * On the ground (contact forces, no drag) the accelerometer is a gravity measurement and the
 * velocity is known to be zero; setLanded() switches between the two measurement models.
 *
 * Body frame x forward, y left, z up; at rest the accelerometer reads (0, 0, +g).
 */
class AttitudeESKF
{
public:
    static constexpr int N = 8;

    struct Params {
        float dt;           // predict period (s)
        float gravity;      // m/s^2
        float rotor_drag;   // k at hover thrust (1/s): horizontal drag acceleration per m/s of air speed
        float q_theta;      // attitude error process noise per step (rad^2)
        float q_bias;       // gyro bias process noise per step ((rad/s)^2)
        float q_vel;        // air velocity process noise per step ((m/s)^2): gusts, model error
        float r_drag;       // accelerometer x/y noise in flight ((m/s^2)^2)
        float r_gravity;    // accelerometer noise on the ground ((m/s^2)^2)
        float r_zero_vel;   // zero-velocity pseudo-measurement on the ground ((m/s)^2)
        float p0_theta;     // initial attitude error variance (rad^2)
        float p0_bias;      // initial gyro bias variance ((rad/s)^2)
        float p0_vel;       // initial velocity variance ((m/s)^2)
    };

private:
    float q[4];
    float bg[3];
    float u, v;
    float k;                // drag coefficient at the current thrust
    bool landed;

    ari_matrix<N, N> P;
    ari_matrix<N, N> Q;
    ari_matrix<N, N> I;
    Params prm;

    void normalize()
    {
        float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n < 1e-6f) return;
        for (int i = 0; i < 4; i++) q[i] /= n;
    }

    /* q = q (x) [1, d/2]: apply a small body-frame rotation */
    void rotate(float dx, float dy, float dz)
    {
        const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        const float hx = 0.5f * dx, hy = 0.5f * dy, hz = 0.5f * dz;
        q[0] = q0 - q1 * hx - q2 * hy - q3 * hz;
        q[1] = q1 + q0 * hx + q2 * hz - q3 * hy;
        q[2] = q2 + q0 * hy - q1 * hz + q3 * hx;
        q[3] = q3 + q0 * hz + q1 * hy - q2 * hx;
        normalize();
    }

    /* Kalman correction with an M-dimensional measurement, then error injection and reset */
    template <int M>
    void correct(const ari_matrix<M, N>& H, const ari_matrix<M, 1>& y, const ari_matrix<M, M>& R)
    {
        ari_matrix<M, M> S = H * P * H.transpose() + R;
        ari_matrix<M, M> S_inv;
        if (!S.inverse(S_inv)) return;

        ari_matrix<N, M> K = P * H.transpose() * S_inv;
        ari_matrix<N, 1> dx = K * y;

        // Inject the error into the nominal state; the error state is then zero again
        rotate(dx(0, 0), dx(1, 0), dx(2, 0));
        bg[0] += dx(3, 0);
        bg[1] += dx(4, 0);
        bg[2] += dx(5, 0);
        u += dx(6, 0);
        v += dx(7, 0);

        // Joseph form keeps P symmetric and positive in single precision
        ari_matrix<N, N> IKH = I - K * H;
        P = IKH * P * IKH.transpose() + K * R * K.transpose();
    }

public:
    explicit AttitudeESKF(const Params& p) : prm(p)
    {
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        bg[0] = bg[1] = bg[2] = 0.0f;
        u = v = 0.0f;
        k = p.rotor_drag;
        landed = true;

        for (int i = 0; i < 3; i++)
        {
            P(i, i) = p.p0_theta;      Q(i, i) = p.q_theta;
            P(3 + i, 3 + i) = p.p0_bias; Q(3 + i, 3 + i) = p.q_bias;
        }
        for (int i = 6; i < N; i++)
        {
            P(i, i) = p.p0_vel;        Q(i, i) = p.q_vel;
        }
        I.setIdentity();
    }

    /* Ground: gravity + zero-velocity measurements. Flight: rotor-drag measurement. */
    void setLanded(bool on_ground) { landed = on_ground; }

    /**
     * @brief Propagate one step.
     * @param gx, gy, gz  gyro (rad/s)
     * @param az          accelerometer z (m/s^2): thrust per mass, sets the drag coefficient
     * @param w_body      velocity along body z (m/s), from the altitude filter
     */
    void predict(float gx, float gy, float gz, float az, float w_body)
    {
        const float dt = prm.dt;
        const float wx = gx - bg[0], wy = gy - bg[1], wz = gz - bg[2];

        rotate(wx * dt, wy * dt, wz * dt);

        float ux, uy, uz;
        getUpInBody(ux, uy, uz);
        const float gbx = -prm.gravity * ux, gby = -prm.gravity * uy, gbz = -prm.gravity * uz;

        // Rotor drag grows with rotor speed, i.e. with the square root of thrust
        float thrust = az > 0.2f * prm.gravity ? az : 0.2f * prm.gravity;
        k = prm.rotor_drag * sqrtf(thrust / prm.gravity);

        ari_matrix<N, N> F;
        F.setIdentity();

        // d_theta' = -[w]x d_theta - d_bg
        F(0, 1) =  wz * dt;  F(0, 2) = -wy * dt;
        F(1, 0) = -wz * dt;  F(1, 2) =  wx * dt;
        F(2, 0) =  wy * dt;  F(2, 1) = -wx * dt;
        F(0, 3) = F(1, 4) = F(2, 5) = -dt;

        if (landed)
        {
            u = v = 0.0f;
        }
        else
        {
            const float du = gbx - k * u - (wy * w_body - wz * v);
            const float dv = gby - k * v - (wz * u - wx * w_body);

            // d(g_b)/d_theta = [g_b]x, rows x and y
            F(6, 1) = -gbz * dt;  F(6, 2) =  gby * dt;
            F(7, 0) =  gbz * dt;  F(7, 2) = -gbx * dt;
            // gyro bias enters through w x v
            F(6, 4) =  w_body * dt;  F(6, 5) = -v * dt;
            F(7, 3) = -w_body * dt;  F(7, 5) =  u * dt;
            // drag and rotation of the velocity vector
            F(6, 6) = 1.0f - k * dt;  F(6, 7) =  wz * dt;
            F(7, 6) = -wz * dt;       F(7, 7) = 1.0f - k * dt;

            u += du * dt;
            v += dv * dt;
        }

        P = F * P * F.transpose() + Q;
    }

    /* Correct with the accelerometer (m/s^2), ideally averaged over the update period */
    void update(float ax, float ay, float az)
    {
        if (!landed)
        {
            // In flight: a_x = -k u, a_y = -k v
            ari_matrix<2, N> H;
            H(0, 6) = -k;
            H(1, 7) = -k;
            ari_matrix<2, 1> y = { ax + k * u, ay + k * v };
            ari_matrix<2, 2> R;
            R.setIdentity();
            R = R * prm.r_drag;
            correct<2>(H, y, R);
        }
        else
        {
            // On the ground: a = -g_b = g * up, and the velocity is zero
            float ux, uy, uz;
            getUpInBody(ux, uy, uz);
            const float g = prm.gravity;
            const float gbx = -g * ux, gby = -g * uy, gbz = -g * uz;

            ari_matrix<5, N> H;
            // d(-g_b)/d_theta = -[g_b]x
            H(0, 1) =  gbz;  H(0, 2) = -gby;
            H(1, 0) = -gbz;  H(1, 2) =  gbx;
            H(2, 0) =  gby;  H(2, 1) = -gbx;
            H(3, 6) = 1.0f;
            H(4, 7) = 1.0f;
            ari_matrix<5, 1> y = { ax - g * ux, ay - g * uy, az - g * uz, -u, -v };
            ari_matrix<5, 5> R;
            R(0, 0) = R(1, 1) = R(2, 2) = prm.r_gravity;
            R(3, 3) = R(4, 4) = prm.r_zero_vel;
            correct<5>(H, y, R);
        }
    }

    /* Returns euler angles (rad), ZYX order */
    void getEuler(float& roll, float& pitch, float& yaw) const
    {
        const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
        float s = 2.0f * (q0 * q2 - q3 * q1);
        if (s > 1.0f) s = 1.0f;
        else if (s < -1.0f) s = -1.0f;
        pitch = asinf(s);
        yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }

    /* World z (up) axis in the body frame; the z component is cos(roll) cos(pitch) */
    void getUpInBody(float& ux, float& uy, float& uz) const
    {
        const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        ux = 2.0f * (q1 * q3 - q0 * q2);
        uy = 2.0f * (q2 * q3 + q0 * q1);
        uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    }

    /* Velocity relative to the air along body x and y (m/s) */
    void getAirVelocity(float& vx, float& vy) const { vx = u; vy = v; }

    float getGyroBias(int i) const { return bg[i]; }
};


#endif
