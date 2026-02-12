#ifndef STEST_WRAPPER_H
#define STEST_WRAPPER_H


#define Ts  0.01    // Define sampling time (dt)


#ifdef __cplusplus
extern "C" {
#endif

/* Takes gyro measurements and runs kalman algorithm to predict the state */
void kalman_predict(float wx, float wy, float wz);


/* Takes accelerometer measurements to update prediction */
void kalman_update(float ax, float ay, float az);


void kalman_getangles(float* roll, float* pitch, float* yaw);







#ifdef __cplusplus
}
#endif

#endif