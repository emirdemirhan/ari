#ifndef STEST_WRAPPER_H
#define STEST_WRAPPER_H


#define Ts  0.002f   // Define sampling time (dt)


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

/* Barometric altitude (m, any zero), at 50 Hz. range_ok: a valid rangefinder sample was used in
   this update; then the barometer only learns its offset, otherwise it measures the height */
void kalman_update_baro(float baro, int range_ok);

/* Returns altitude */
void kalman_getaltitude(float* height, float* v_speed);


/* Scales the accelerometer noise of both attitude filters: > 1 trusts the accelerometer less */
void estimators_set_accel_noise_scale(float scale);


/* ---- Error-state Kalman filter (ESKF), runs alongside the linear filters above ---- */

/* Ground (1): gravity + zero-velocity measurements. Flight (0): rotor-drag measurement. */
void eskf_set_landed(int landed);

/* Takes gyro (rad/s) and accelerometer z (m/s^2) and runs the ESKF predict step */
void eskf_predict_angles(float wx, float wy, float wz, float az);

/* Takes accelerometer measurements (m/s^2, averaged over the update period) to correct the ESKF */
void eskf_update_angles(float ax, float ay, float az);

/* Returns euler angles */
void eskf_getangles(float* roll, float* pitch, float* yaw);

/* Returns the velocity relative to the air along body x and y (m/s) */
void eskf_getairvelocity(float* vx, float* vy);

/* Takes the body accelerometer vector, rotates it with the ESKF attitude and predicts the altitude */
void eskf_predict_altitude(float ax, float ay, float az);

/* Takes the range measured along the body z axis to update the altitude prediction */
void eskf_update_altitude(float range);

/* Same as kalman_update_baro, for the ESKF-path altitude filter */
void eskf_update_baro(float baro, int range_ok);

/* Returns altitude */
void eskf_getaltitude(float* height, float* v_speed);



#ifdef __cplusplus
}
#endif

#endif