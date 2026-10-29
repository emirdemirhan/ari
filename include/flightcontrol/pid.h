#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct pid {
    float Kp;
    float Ki;
    float Kd;
    float y_prev;  // Önceki ölçüm (türev ölçüm üzerinden alınır)
    float e_sum;
    float max_i;   // İntegral rüzgarlanma limiti
    float out_min; // Döngüye özel minimum çıkış
    float out_max; // Döngüye özel maksimum çıkış
} pid_coeffs;

static inline float calculate_pid(pid_coeffs* pid, float r, float y, float ts) {
    float e = r - y;

    // Integral birikimi ve Anti-windup
    pid->e_sum += e * ts;
    if (pid->e_sum > pid->max_i) pid->e_sum = pid->max_i;
    else if (pid->e_sum < -pid->max_i) pid->e_sum = -pid->max_i;

    // PID hesabı
    float P = pid->Kp * e;
    float I = pid->Ki * pid->e_sum;
    // Türev hata yerine ölçüm üzerinden alınır: referans basamak değiştirdiğinde
    // (ör. 50 Hz dış döngü çıkışı) D terimi sıçrama (derivative kick) üretmez.
    float D = -pid->Kd * (y - pid->y_prev) / ts;

    float u = P + I + D;

    // Dinamik Çıkış Saturasyonu (Her döngünün kendi limitlerine göre)
    if (u > pid->out_max) u = pid->out_max;
    else if (u < pid->out_min) u = pid->out_min;

    pid->y_prev = y;

    return u;
}

#endif