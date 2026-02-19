#ifndef PID_H
#define PID_H

#include <stdint.h>

#define MAX_U  100  // Actuator saturation
#define MAX_I  50   // Integral saturation


typedef struct pid
{
    float Kp;
    float Ki;
    float Kd;

}pid_coeffs;


/* Calculate PID according to the classical PID formula:
    - u[k] = Kp.e + Kd . (e - e_prev)/dt + Ki . (e + e_sum)
    returns u  */
static inline float calculate_pid(pid_coeffs* pid, float r, float y, float ts);


#endif