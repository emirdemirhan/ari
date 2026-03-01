#include "flightcontrol/kalman.h"
#include "flightcontrol/stest_wrapper.h"
#include "ari_math/ari_matrix.h"
#include <cmath>


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

Kalman<6,3,2> filter_angles(A, B, H, Q, R, P, Ts);



//Kalman filter_altitude(Ts);

#ifdef __cplusplus
extern "C" {
#endif

/* Takes gyro measurements and runs kalman algorithm to predict the state*/
void kalman_predict(float wx, float wy, float wz)
{
    ari_Vector3f gyro = {wx, wy, wz};

    filter_angles.predict(gyro);

}


/* Takes accelerometer measurements to update prediction */
void kalman_update(float ax, float ay, float az)
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
    *roll = filter_angles.getRoll();
    *pitch = filter_angles.getPitch();
    *yaw = filter_angles.getYaw();
}



#ifdef __cplusplus
}
#endif
