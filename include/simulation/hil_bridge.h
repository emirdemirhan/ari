#ifndef HIL_BRIDGE_H
#define HIL_BRIDGE_H

#include <stdint.h>

/* --- HIL Veri Yapıları --- */
typedef struct __attribute__((packed)) {
    uint8_t header; // 0xAA
    float roll;
    float pitch;
    float yaw;
} WebotsToSTM_t;

typedef struct __attribute__((packed)) {
    float main_motor_val;
    float tail_motor_val;
} STMToWebots_t;

#define RECEIVE_BYTES   16

/* --- Tek Fonksiyon --- */
// Bu fonksiyonu main içinde çağırman yeterli olacak.
void start_usb_hil_system(void);
void hil_send(const char *str);
void hil_receive(char *str);


#endif