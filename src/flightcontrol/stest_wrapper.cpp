#include "flightcontrol/state_estimation.h"
#include "flightcontrol/stest_wrapper.h"

Kalman filter(Ts);

#ifdef __cplusplus
extern "C" {
#endif

/* Takes gyro measurements and runs kalman algorithm to predict the state*/
void kalman_predict(float wx, float wy, float wz)
{
    ari_Vector3f gyro = {wx, wy, wz};

    filter.predict(gyro);

}


/* Takes accelerometer measurements to update prediction */
void kalman_update(float ax, float ay)
{
    ari_matrix<2,1> acc = {ax, ay};

    filter.update(acc);

}

void kalman_getangles(float* roll, float* pitch, float* yaw)
{
    *roll = filter.getRoll();
    *pitch = filter.getPitch();
    *yaw = filter.getYaw();
}



#ifdef __cplusplus
}
#endif
