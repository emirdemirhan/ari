/*
 * Host-side comparison of the linear Kalman filter and the ESKF (run with: make host-test).
 *
 * A small quadrotor truth model flies scripted body-rate manoeuvres while holding altitude:
 *   - attitude from the body rates (exact quaternion integration)
 *   - specific force in the body frame: thrust along z (T/m = g / cos(tilt), altitude hold) and
 *     rotor drag in the rotor plane, -k (u, v), k = k0 sqrt(T/m / g)  (what the ESKF assumes)
 *   - world velocity and height from that force plus gravity; no wind
 * Gyro and accelerometer get white noise and a small gyro bias (the residual after the boot
 * calibration in controller.c); the rangefinder reads height / cos(tilt).
 * Both filters are driven through the same C API and call pattern as controller_step
 * (500 Hz predict, 50 Hz update with the accelerometer averaged for the ESKF; the first
 * seconds are "on the ground" for the ESKF).
 *
 * The linear filter treats the accelerometer as gravity; in flight it reads thrust + drag, so it
 * misreads every acceleration as tilt. The ESKF uses the drag model instead.
 * The last scenario gives the truth 30 % less drag than the ESKF assumes (a mis-identified k).
 */

#include <cmath>
#include <cstdio>
#include <random>
#include "flightcontrol/stest_wrapper.h"

static const double G = 9.81;
static const double PI = 3.14159265358979323846;
static const double K0 = 0.35;          // rotor drag at hover, must match ESKF_ROTOR_DRAG

struct Quat { double w, x, y, z; };

static Quat integrate(Quat q, double wx, double wy, double wz, double dt)
{
    double angle = std::sqrt(wx*wx + wy*wy + wz*wz) * dt;
    Quat d = {1.0, 0.0, 0.0, 0.0};
    if (angle > 1e-12)
    {
        double s = std::sin(angle / 2.0) / (angle / dt);
        d = {std::cos(angle / 2.0), wx * s, wy * s, wz * s};
    }
    Quat r = {
        q.w*d.w - q.x*d.x - q.y*d.y - q.z*d.z,
        q.w*d.x + q.x*d.w + q.y*d.z - q.z*d.y,
        q.w*d.y - q.x*d.z + q.y*d.w + q.z*d.x,
        q.w*d.z + q.x*d.y - q.y*d.x + q.z*d.w
    };
    double n = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
    return {r.w / n, r.x / n, r.y / n, r.z / n};
}

/* Rotation matrix body -> world, row major */
static void rotmat(Quat q, double R[9])
{
    double w = q.w, x = q.x, y = q.y, z = q.z;
    R[0] = 1 - 2*(y*y + z*z); R[1] = 2*(x*y - w*z);     R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);     R[4] = 1 - 2*(x*x + z*z); R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);     R[7] = 2*(y*z + w*x);     R[8] = 1 - 2*(x*x + y*y);
}

static void to_euler(Quat q, double& roll, double& pitch, double& yaw)
{
    roll = std::atan2(2.0 * (q.w*q.x + q.y*q.z), 1.0 - 2.0 * (q.x*q.x + q.y*q.y));
    pitch = std::asin(std::fmax(-1.0, std::fmin(1.0, 2.0 * (q.w*q.y - q.z*q.x))));
    yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}

struct Segment { double t0, t1, wx, wy, wz; };

struct Result { double kf_tilt, eskf_tilt, eskf_vel, kf_alt, eskf_alt; };

/* The filters are global objects with no reset: scenarios run back to back, each starting and
   ending level and at rest (the run ends with a long hover so the velocity decays). */
static Result run(const Segment* segs, int nseg, double duration, double drag_scale)
{
    static Quat q = {1.0, 0.0, 0.0, 0.0};
    static double vw[3] = {0.0, 0.0, 0.0};
    static double h = 2.0;
    static bool first = true;
    static std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);

    const double bias[3] = {0.002, -0.001, 0.0};   // rad/s, residual after the boot calibration
    const double gyro_noise = 0.002, accel_noise = 0.3;
    const double settle = 2.0;
    Result res = {0, 0, 0, 0, 0};
    double acc_sum[3] = {0, 0, 0};
    int acc_n = 0;

    int steps = (int)(duration / Ts);
    for (int k = 1; k <= steps; k++)
    {
        double t = k * (double)Ts;
        // The very first seconds are on the ground (gravity + zero-velocity updates)
        bool landed = first && t < 2.0;
        double wx = 0, wy = 0, wz = 0;
        for (int i = 0; i < nseg; i++)
            if (t >= segs[i].t0 && t < segs[i].t1) { wx += segs[i].wx; wy += segs[i].wy; wz += segs[i].wz; }
        q = integrate(q, wx, wy, wz, Ts);

        double R[9];
        rotmat(q, R);
        double uz = R[8];
        // Body-frame velocity through the air (no wind)
        double u = R[0]*vw[0] + R[3]*vw[1] + R[6]*vw[2];
        double v = R[1]*vw[0] + R[4]*vw[1] + R[7]*vw[2];
        // Altitude hold: thrust cancels gravity and the vertical part of the rotor drag
        double kd = drag_scale * K0 * std::sqrt(G / std::fmax(uz, 0.3) / G);
        double drag_up = R[6] * (-kd * u) + R[7] * (-kd * v);
        double thrust = (G - drag_up) / std::fmax(uz, 0.3);
        kd = drag_scale * K0 * std::sqrt(std::fmax(thrust, 0.2 * G) / G);
        double f[3] = {-kd * u, -kd * v, thrust};                  // specific force, body
        if (landed) { f[0] = 0; f[1] = 0; f[2] = G; }
        for (int i = 0; i < 3; i++)
            vw[i] += (R[3*i]*f[0] + R[3*i+1]*f[1] + R[3*i+2]*f[2] - (i == 2 ? G : 0.0)) * Ts;
        if (landed) vw[0] = vw[1] = vw[2] = 0.0;
        h += vw[2] * Ts;

        float gx = (float)(wx + bias[0] + gyro_noise * n01(rng));
        float gy = (float)(wy + bias[1] + gyro_noise * n01(rng));
        float gz = (float)(wz + bias[2] + gyro_noise * n01(rng));
        float ax = (float)(f[0] + accel_noise * n01(rng));
        float ay = (float)(f[1] + accel_noise * n01(rng));
        float az = (float)(f[2] + accel_noise * n01(rng));
        float range = (float)(h / std::fmax(uz, 0.3));

        // Same call pattern as controller_step
        kalman_predict_altitude(0, 0, az);
        kalman_predict_angles(gx, gy, gz);
        eskf_set_landed(landed ? 1 : 0);
        eskf_predict_angles(gx, gy, gz, az);
        eskf_predict_altitude(ax, ay, az);
        acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az; acc_n++;
        if (k % 10 == 0)
        {
            kalman_update_angles(ax, ay, az);
            eskf_update_angles((float)(acc_sum[0] / acc_n), (float)(acc_sum[1] / acc_n), (float)(acc_sum[2] / acc_n));
            acc_sum[0] = acc_sum[1] = acc_sum[2] = 0; acc_n = 0;
            kalman_update_altitude(range);
            eskf_update_altitude(range);
        }

        if (t < settle) continue;

        double roll, pitch, yaw;
        to_euler(q, roll, pitch, yaw);
        float r, p, y, hh, vv, eu, ev;
        kalman_getangles(&r, &p, &y);
        res.kf_tilt = std::fmax(res.kf_tilt, std::fmax(std::fabs(r - roll), std::fabs(p - pitch)));
        kalman_getaltitude(&hh, &vv);
        res.kf_alt = std::fmax(res.kf_alt, std::fabs(hh - h));

        eskf_getangles(&r, &p, &y);
        res.eskf_tilt = std::fmax(res.eskf_tilt, std::fmax(std::fabs(r - roll), std::fabs(p - pitch)));
        eskf_getaltitude(&hh, &vv);
        res.eskf_alt = std::fmax(res.eskf_alt, std::fabs(hh - h));
        eskf_getairvelocity(&eu, &ev);
        res.eskf_vel = std::fmax(res.eskf_vel, std::fmax(std::fabs(eu - u), std::fabs(ev - v)));
    }
    first = false;
    return res;
}

static void report(const char* name, Result r)
{
    printf("%-44s %8.3f %8.3f   %8.2f   %8.3f %8.3f\n", name, r.kf_tilt, r.eskf_tilt, r.eskf_vel, r.kf_alt, r.eskf_alt);
}

int main(void)
{
    // 1. Roll to 0.3 rad (17 deg) for 2 s, then back: the quad accelerates sideways, then drag stops it
    const Segment small_roll[] = {{3.0, 3.5, 0.6, 0, 0}, {5.5, 6.0, -0.6, 0, 0}};
    // 2. Pitch to 0.4 rad, yaw 90 deg about body z while tilted, undo both
    const Segment yaw_tilted[] = {{3.0, 3.5, 0, 0.8, 0}, {4.0, 5.0, 0, 0, PI / 2.0},
                                  {7.0, 8.0, 0, 0, -PI / 2.0}, {8.5, 9.0, 0, -0.8, 0}};
    // 3. Roll to 0.8 rad (46 deg) for 2 s, back
    const Segment big_roll[] = {{3.0, 4.0, 0.8, 0, 0}, {6.0, 7.0, -0.8, 0, 0}};

    printf("Max abs error after settling             tilt (rad)        air speed (m/s)   height (m)\n");
    printf("%-44s %8s %8s   %8s   %8s %8s\n", "scenario", "KF", "ESKF", "ESKF", "KF", "ESKF");

    Result r1 = run(small_roll, 2, 25.0, 1.0);  report("1 roll 0.3 rad held 2 s", r1);
    Result r2 = run(yaw_tilted, 4, 25.0, 1.0);  report("2 yaw 90 deg while pitched 0.4 rad", r2);
    Result r3 = run(big_roll, 2, 25.0, 1.0);    report("3 roll 0.8 rad held 2 s", r3);
    Result r4 = run(small_roll, 2, 25.0, 0.7);  report("4 as 1, true drag 30 % below the model", r4);

    // The ESKF must track tilt and height; scenario 4 is a model-error stress test with looser
    // limits (its air-speed estimate is ~30 % low by design: velocity scales with k_true / k_model)
    bool ok = r1.eskf_tilt < 0.035 && r2.eskf_tilt < 0.035 && r3.eskf_tilt < 0.035 &&
              r1.eskf_alt < 0.1 && r2.eskf_alt < 0.1 && r3.eskf_alt < 0.1 &&
              r4.eskf_tilt < 0.06 && r4.eskf_alt < 0.25;

    printf("\n%s\n", ok ? "PASS" : "FAIL: ESKF tilt > 0.035 rad or height > 0.1 m (scenario 4: 0.06 rad / 0.25 m)");
    return ok ? 0 : 1;
}
