#ifndef PID_H
#define PID_H

#include <stdint.h>

#define MAX_U  100.0f
#define MAX_I  50.0f

typedef struct pid
{
    float Kp;
    float Ki;
    float Kd;
    float e_prev;
    float e_sum;
} pid_coeffs;

static inline float calculate_pid(pid_coeffs* pid, float r, float y, float ts)
{
    float e = r - y;

    // Integral birikimi
    pid->e_sum += e * ts;

    // Anti-windup: integral saturasyonu
    if (pid->e_sum > MAX_I) pid->e_sum = MAX_I;
    else if (pid->e_sum < -MAX_I) pid->e_sum = -MAX_I;

    // PID hesabı
    float P = pid->Kp * e;
    float I = pid->Ki * pid->e_sum;
    float D = pid->Kd * (e - pid->e_prev) / ts;

    float u = P + I + D;

    // Çıkış saturasyonu
    if (u > MAX_U) u = MAX_U;
    else if (u < -MAX_U) u = 0.0f; // Motor komutları negatif olamaz, 0'a saturate

    pid->e_prev = e;

    return u;
}

#endif