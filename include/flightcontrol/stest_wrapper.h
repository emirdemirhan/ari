#ifndef STEST_WRAPPER_H
#define STEST_WRAPPER_H


#define Ts  0.01    // Define sampling time (dt)


#ifdef __cplusplus
extern "C" {
#endif

/* Takes gyro measurements and runs kalman algorithm to predict the state */
void kalman_predict_angles(float wx, float wy, float wz);

/* Takes accelerometer measurements to update prediction */
void kalman_update_angles(float ax, float ay, float az);

/* Returns euler angles */
void kalman_getangles(float* roll, float* pitch, float* yaw);


/* Takes measurements and runs kalman algorithm to predict the altitude */
void kalman_predict_altitude(float wx, float wy, float wz);

/* Takes measurements to update the altitude prediction */
void kalman_update_altitude(float hz);

/* Returns altitude */
void kalman_getaltitude(float* height, float* v_speed);






#ifdef __cplusplus
}
#endif

#endif