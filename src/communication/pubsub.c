#include "communication/pubsub.h"
#include "FreeRTOS.h"
#include "task.h"


static pubsub_data_t _rc_data = {0};


void pubsub_set(const pubsub_data_t *new_data)
{
    taskENTER_CRITICAL();
    // ARM Cortex-M burada harika bir iş yapar. 
    // Tüm struct'ı byte byte değil, 32-bit (word) bloklar halinde saniyesinde kopyalar.
    _rc_data = *new_data; 
    taskEXIT_CRITICAL();
}

void pubsub_get(pubsub_data_t *out_data)
{
    taskENTER_CRITICAL();
    *out_data = _rc_data; // O anki güncel veriyi, dışarıdan gelen adrese yapıştır.
    taskEXIT_CRITICAL();
}