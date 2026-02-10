#include "flightcontrol/pid.h"

static inline float calculate_pid(pid_coeffs* pid, float r, float y)
{
    /* PID Control Algorithm
    - u[k] = Kp.e + Kd . (e - e_prev) + Ki . (e + e_sum)  
    */

    float u = 0;
    float e_prev = 0;
    float e_sum = 0;

    float e = r - y;

    u = pid->Kp * e + pid->Kd * (e - e_prev) + pid->Ki + (e + e_sum);

    e_prev = e;
    e_sum += e; 

    return u;

}   