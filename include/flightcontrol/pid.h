#ifndef PID_H
#define PID_H

#include <stdint.h>

#define MAX_U  100  // Actuator saturation


typedef struct pid
{
    float Kp;
    float Ki;
    float Kd;

}pid_coeffs;


static inline float calculate_pid(pid_coeffs* pid, float r, float y);


#endif