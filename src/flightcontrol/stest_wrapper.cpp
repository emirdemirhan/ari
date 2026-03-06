#include "flightcontrol/kalman.h"
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
ari_matrix<6,6> P;


/* Set Q matrix */
/* Process covariance matrix (6,6) */
ari_matrix<6,6> Q = {
    // Angle variances (Roll, Pitch, Yaw)
    0.01f,  0.0f,   0.0f,   0.0f,    0.0f,    0.0f,
    0.0f,   0.01f,  0.0f,   0.0f,    0.0f,    0.0f,
    0.0f,   0.0f,   0.01f,  0.0f,    0.0f,    0.0f,
    
    // Bias variances (X, Y, Z)
    0.0f,   0.0f,   0.0f,   0.001f,  0.0f,    0.0f,
    0.0f,   0.0f,   0.0f,   0.0f,    0.001f,  0.0f,
    0.0f,   0.0f,   0.0f,   0.0f,    0.0f,    0.001f
};


/* Set R matrix */
ari_matrix<2,2> R;



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
    ari_matrix <1,1> u = {wz - 1.0f};

    filter_altitude.predict(u);

}

void kalman_update_altitude(float hz)
{
    ari_matrix <1,1> meas_h = {hz};

    filter_altitude.predict(meas_h);
}

void kalman_getaltitude(float *height, float *v_speed)
{
    *height = filter_altitude.getState(1,0);
    *v_speed = filter_altitude.getState(2,0);
}


#ifdef __cplusplus
}
#endif
