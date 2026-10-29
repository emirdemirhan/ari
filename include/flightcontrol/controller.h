#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "simulation/hil_protocol.h"


/* PID katsayılarını ve kontrolcü durumunu başlangıç değerlerine getirir.
   Kalman/EKF nesneleri globaldir ve burada sıfırlanmaz. */
void controller_init(void);

/* Bir kontrol adımı (2 ms): sensör + RC girişinden motor komutlarını ve telemetriyi üretir.
   out->exec_cycles 0 yazılır; ölçümü çağıran taraf doldurur. */
void controller_step(const HIL_Sim_Data_t *in, HIL_Fc_Data_t *out);

#endif
