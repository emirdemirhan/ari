#include "flightcontrol/kalman.h"
#include "flightcontrol/stest_wrapper.h"
#include <cmath>

Kalman filter_angles(Ts);
Kalman filter_altitude(Ts);

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
