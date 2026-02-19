#include "flightcontrol/pid.h"

static inline float calculate_pid(pid_coeffs* pid, float r, float y, float ts)
{
    

    float u = 0;
    float e_prev = 0;
    float e_sum = 0;

    float e = r - y;

    u = pid->Kp * e + pid->Kd * (e - e_prev)*(1/ts) + pid->Ki + (e + e_sum)*ts;

    e_prev = e;
    e_sum += e; 

    return u;

}   