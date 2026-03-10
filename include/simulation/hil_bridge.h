#ifndef HIL_BRIDGE_H
#define HIL_BRIDGE_H

#include <stdint.h>

/* --- HIL Veri Yapıları --- */
#include <stdint.h>

// Simülasyondan STM32'ye gelecek tüm veriler
typedef struct __attribute__((packed)) {
    // 1. Sensörler
    float accel[3];    // İvme (m/s^2)
    float gyro[3];     // Açısal hız (rad/s)
    float altitude_z;  // Lazer yüksekliği (metre)
    
    // 2. RC / Kumanda Komutları
    float rc_roll;     // İstenen Roll açısı
    float rc_pitch;    // İstenen Pitch açısı
    float rc_yaw_rate; // İstenen Yaw dönüş hızı
    float rc_thrust;   // Gaz kolu (0.0 - 1.0)
    
    // 3. Simülasyon Gerçekliği (Hata ayıklama için)
    float true_roll;
    float true_pitch;
    float true_yaw;
} HIL_Sim_Data_t;


typedef struct __attribute__((packed)) {
    float main_motor_pwm; // 4 Byte
    float tail_motor_pwm; // 4 Byte
} HIL_Motor_Data_t;


#define RECEIVE_BYTES   56 // HIL_Sim_Data_t'nin byte cinsinden boyutu (7 float * 4 byte/float = 28 byte) * 2 (header+footer) = 56 byte

/* --- Tek Fonksiyon --- */
// Bu fonksiyonu main içinde çağırman yeterli olacak.
void start_usb_hil_system(void);
void hil_send(const char *str);
void hil_receive(char *str);
void hil_send_float(float val);
uint8_t hil_receive_full_packet(HIL_Sim_Data_t *data_out);
void hil_send_motor_cmds(float main_pwm, float tail_pwm);

#endif