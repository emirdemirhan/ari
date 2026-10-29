#include "flightcontrol/kalman.h"
#include "flightcontrol/eskf.h"
#include "flightcontrol/stest_wrapper.h"
#include "ari_math/ari_matrix.h"
#include <cmath>
#include "stest_wrapper.h"


/* ********* Kalman filter angle prediction matrices ******************** */

/* System matrix A is considered as 
\theta_(k+1) = \theta_k + (w_theta - bias_theta) * dt   */
ari_Matrix6f A = {
            1.0f, 0.0f, 0.0f, -Ts, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f, -Ts, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -Ts,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
        };

/* Control input matrix (6,3) which is compatible with measurement (3,1) */
ari_matrix<6,3> B = { Ts,0.0f,0,
                        0,Ts,0,
                        0,0,Ts,
                        0,0,0,
                        0,0,0,
                        0,0,0};

/* Set P matrix */
/* Initial error covariance (6,6): angles are unknown at start.
   Gyro bias: (0.003 rad/s)^2. controller.c removes the gyro offset at boot (2 s still on the
   ground), so only a small residual remains. A large value here lets the filter explain real
   rotation during a manoeuvre as "bias" and stop following the gyro. */
ari_matrix<6,6> P = {
    0.1f,  0.0f,  0.0f,  0.0f,   0.0f,   0.0f,
    0.0f,  0.1f,  0.0f,  0.0f,   0.0f,   0.0f,
    0.0f,  0.0f,  0.1f,  0.0f,   0.0f,   0.0f,
    0.0f,  0.0f,  0.0f,  1e-5f  , 0.0f,    0.0f,
    0.0f,  0.0f,  0.0f,  0.0f,    1e-5f  , 0.0f,
    0.0f,  0.0f,  0.0f,  0.0f,    0.0f,    1e-5f  
};


/* Set Q matrix */
/* Process covariance matrix (6,6), added once per 2 ms predict step.
   Together with R these give a steady-state gain of ~0.005 at the 50 Hz update,
   i.e. the accelerometer corrects gyro drift with a time constant of a few seconds. */
ari_matrix<6,6> Q = {
    // Angle variances (Roll, Pitch, Yaw)
    1e-7f,  0.0f,   0.0f,   0.0f,    0.0f,    0.0f,
    0.0f,   1e-7f,  0.0f,   0.0f,    0.0f,    0.0f,
    0.0f,   0.0f,   1e-7f,  0.0f,    0.0f,    0.0f,
    
    // Bias variances (X, Y, Z)
    0.0f,   0.0f,   0.0f,   1e-9f ,  0.0f,    0.0f,
    0.0f,   0.0f,   0.0f,   0.0f,    1e-9f ,  0.0f,
    0.0f,   0.0f,   0.0f,   0.0f,    0.0f,    1e-9f 
};


/* Set R matrix */
/* Measurement (accelerometer angle) noise covariance (2,2) */
ari_matrix<2,2> R = {
    0.05f, 0.0f,
    0.0f,  0.05f
};



/* Set H matrix */
ari_matrix<2,6> H = {
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f
        };

Kalman<6,3,2> filter_angles(A, B, H, Q, R, P);

/* ** Kalman filter for altitude estimation ** */

/* State transition matrix */
ari_Matrix3f F = {
        1.0f,  Ts,   0.0f,
        0.0f,  1.0f, -Ts,
        0.0f,  0.0f,  1.0f
    };

/* Control input matrix */
ari_Vector3f B_alt = {
        0.0f,
        Ts,
        0.0f
    };

/* Observation matrix */
ari_matrix<1,3> H_alt = {
        1.0f, 0.0f, 0.0f
    };

ari_Matrix3f Q_alt = {
        0.001f, 0.0f,   0.0f,    // Yükseklik tahmini genelde iyidir
        0.0f,   0.01f,  0.0f,    // Hız, ivmeden entegre edildiği için biraz daha oynaktır
        0.0f,   0.0f,   0.0001f  // Bias ÇOK YAVAŞ değişir, o yüzden varyansı çok küçüktür
    };

/* Measurement noise covariance */
ari_matrix<1,1> R_alt = {
        0.05f 
    };

/* 6. Initial Error Covariance (3x3) -> P_alt */
    // Başlangıçta hiçbir şey bilmediğimiz için belirsizlik yüksek
ari_Matrix3f P_alt = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };


Kalman<3,1,1> filter_altitude(F, B_alt, H_alt, Q_alt, R_alt, P_alt);

/* ********* Error-state Kalman filter, runs alongside the linear filters above ************** */

/* Rotor drag at hover thrust: horizontal acceleration per m/s of air speed (1/s).
   The sim's x3_500g uses 0.35. On the real airframe identify it from a flight log: in steady
   forward flight, accelerometer x divided by air speed (or fit the decay of the speed after
   releasing the stick). A wrong value scales the velocity estimate but steady-state tilt stays
   right (drag and gravity balance whatever k is). */
static const float ESKF_ROTOR_DRAG = 0.35f;

static const AttitudeESKF::Params eskf_params = {
    Ts,            // dt
    9.81f,         // gravity
    ESKF_ROTOR_DRAG,
    1e-8f,         // q_theta: attitude error process noise per step (rad^2)
    3e-9f,         // q_bias: gyro bias process noise per step ((rad/s)^2)
    5e-3f,         // q_vel: air velocity process noise per step ((m/s)^2), gusts and model error
    0.1f,          // r_drag: accelerometer x/y noise in flight (averaged over 20 ms, vibration included)
    0.1f,          // r_gravity: accelerometer noise on the ground
    1e-4f,         // r_zero_vel: zero-velocity pseudo-measurement on the ground
    1e-4f,         // p0_theta: the boot calibration starts the filter level
    1e-5f,         // p0_bias: (0.003 rad/s)^2, residual after the boot gyro calibration
    1e-4f          // p0_vel: starts on the ground
};

AttitudeESKF eskf_attitude(eskf_params);

/* Altitude filter of the ESKF path. Same linear model as filter_altitude,
   but its inputs are rotated to the world frame with the ESKF attitude. */
Kalman<3,1,1> eskf_altitude(F, B_alt, H_alt, Q_alt, R_alt, P_alt);

/* ********* Barometer fusion (both altitude filters) ****************************************
   The rangefinder (VL53L1X) measures height above the ground precisely but only up to ~4 m.
   The barometer works at any height but is noisy (rotor downwash, wind) and has an unknown,
   slowly drifting zero. So:
     - range valid: the rangefinder drives the height as before; the barometer only learns its
       offset = baro - height (ground elevation under the vehicle + baro drift), low-passed
     - range not valid (above its reach, blocked): the barometer minus that offset is the height
       measurement. The offset is frozen meanwhile: the height is then relative to the ground
       where the rangefinder last saw it. */
static const float R_BARO = 0.1f;                  // (m^2): sensor + downwash noise, dynamic pressure error
static const float BARO_OFFSET_UPDATES = 100.0f;   // offset low-pass time constant in 50 Hz updates (2 s)
/* Moving air lowers the static pressure at the sensor, so it reads low by c v^2 / (2 g) (8 m/s:
   about 1 m). The ESKF path corrects it with its air-velocity estimate (as PX4's EKF2_PCOEF_*).
   0.3 matches the simulator's x3_500g; on the real airframe identify it from fast passes at
   constant height inside the rangefinder's reach: baro error against air speed squared. */
static const float BARO_DYN_PRESSURE_COEF = 0.3f;

struct BaroFusion { float offset; bool init; };
static BaroFusion baro_kf = {0.0f, false};
static BaroFusion baro_eskf = {0.0f, false};

static void baro_fuse(Kalman<3,1,1>& f, BaroFusion& b, float baro, int range_ok)
{
    float h = f.getState(0,0);
    if (!b.init)
    {
        b.offset = baro - h;
        b.init = true;
        return;
    }
    if (range_ok)
    {
        b.offset += (baro - h - b.offset) / BARO_OFFSET_UPDATES;
        return;
    }
    ari_matrix<1,1> r_baro = {R_BARO};
    ari_matrix<1,1> meas_h = {baro - b.offset};
    f.setR(r_baro);
    f.update(meas_h);
    f.setR(R_alt);
}

#ifdef __cplusplus
extern "C" {
#endif

/* Takes gyro measurements and runs kalman algorithm to predict the state*/
void kalman_predict_angles(float wx, float wy, float wz)
{
    ari_Vector3f gyro = {wx, wy, wz};

    filter_angles.predict(gyro);

}


/* Takes accelerometer measurements to update prediction */
void kalman_update_angles(float ax, float ay, float az)
{
    // Roll calculation
    float phi_acc = std::atan2(ay, az);

    // Pitch calculation
    float theta_acc = std::atan2(-ax, std::sqrt(ay*ay + az*az));

    ari_matrix<2,1> acc = {phi_acc, theta_acc};

    filter_angles.update(acc);

}

void kalman_getangles(float* roll, float* pitch, float* yaw)
{
    *roll = filter_angles.getState(0,0);
    *pitch = filter_angles.getState(1,0);
    *yaw = filter_angles.getState(2,0);
}

void kalman_predict_altitude(float wx, float wy, float wz)
{
    ari_matrix <1,1> u = {wz - 9.81f}; // Gyro Z ekseninden açısal hız, yerçekimi etkisi çıkarıldı

    filter_altitude.predict(u);

}

void kalman_update_altitude(float hz)
{
    ari_matrix <1,1> meas_h = {hz};

    filter_altitude.update(meas_h);
}

/* Scales the accelerometer measurement noise of the linear attitude filter (1 = tuning in this
   file). The ESKF does not need it: it models what the accelerometer measures in flight. */
void estimators_set_accel_noise_scale(float scale)
{
    filter_angles.setR(R * scale);
}

void kalman_getaltitude(float *height, float *v_speed)
{
    *height = filter_altitude.getState(0,0);
    *v_speed = filter_altitude.getState(1,0);
}


void eskf_set_landed(int landed)
{
    eskf_attitude.setLanded(landed != 0);
}

/* Takes gyro (rad/s) and accelerometer z (m/s^2) and runs the ESKF predict step */
void eskf_predict_angles(float wx, float wy, float wz, float az)
{
    // Body z velocity from the altitude filter's vertical speed: v_z = ux u + uy v + uz w
    float ux, uy, uz, u, v;
    eskf_attitude.getUpInBody(ux, uy, uz);
    eskf_attitude.getAirVelocity(u, v);
    if (uz < 0.5f) uz = 0.5f;
    float w_body = (eskf_altitude.getState(1,0) - ux * u - uy * v) / uz;

    eskf_attitude.predict(wx, wy, wz, az, w_body);
}

/* Takes accelerometer measurements (m/s^2, averaged over the update period) to correct the ESKF */
void eskf_update_angles(float ax, float ay, float az)
{
    eskf_attitude.update(ax, ay, az);
}

void eskf_getangles(float* roll, float* pitch, float* yaw)
{
    eskf_attitude.getEuler(*roll, *pitch, *yaw);
}

void eskf_getairvelocity(float* vx, float* vy)
{
    eskf_attitude.getAirVelocity(*vx, *vy);
}

void eskf_predict_altitude(float ax, float ay, float az)
{
    // Vertical (world z) component of the body acceleration, gravity removed
    float ux, uy, uz;
    eskf_attitude.getUpInBody(ux, uy, uz);

    ari_matrix <1,1> u = {ax*ux + ay*uy + az*uz - 9.81f};

    eskf_altitude.predict(u);
}

void eskf_update_altitude(float range)
{
    // The range is measured along the body z axis: height = range * cos(roll) * cos(pitch)
    float ux, uy, uz;
    eskf_attitude.getUpInBody(ux, uy, uz);

    if (uz < 0.3f) return;  // Tilted more than ~72 deg, the beam does not see the ground below

    ari_matrix <1,1> meas_h = {range * uz};

    eskf_altitude.update(meas_h);
}

void kalman_update_baro(float baro, int range_ok)
{
    baro_fuse(filter_altitude, baro_kf, baro, range_ok);
}

void eskf_update_baro(float baro, int range_ok)
{
    float u, v;
    eskf_attitude.getAirVelocity(u, v);
    baro += BARO_DYN_PRESSURE_COEF * (u * u + v * v) / (2.0f * 9.81f);
    baro_fuse(eskf_altitude, baro_eskf, baro, range_ok);
}

void eskf_getaltitude(float *height, float *v_speed)
{
    *height = eskf_altitude.getState(0,0);
    *v_speed = eskf_altitude.getState(1,0);
}


#ifdef __cplusplus
}
#endif
