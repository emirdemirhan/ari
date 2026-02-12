#include "bmi160.h"
#include "bmi160_wrapper.h" // Include the static inline wrapper

/* * Register Map
 * Private to this file, no need to expose to header.
 */
#define REG_CHIP_ID      0x00
#define REG_GYRO_DATA    0x0C
#define REG_ACCEL_DATA   0x12
#define REG_ACC_CONF     0x40
#define REG_ACC_RANGE    0x41
#define REG_GYR_CONF     0x42
#define REG_GYR_RANGE    0x43
#define REG_CMD          0x7E

// Commands
#define CMD_SOFT_RESET   0xB6
#define CMD_ACC_NORMAL   0x11
#define CMD_GYR_NORMAL   0x15

volatile uint8_t id;

bool bmi160_init(const BMI160_Config *config) {
    // 1. Soft Reset
    bmi160_ll_write(REG_CMD, CMD_SOFT_RESET);
    bmi160_delay_us(15000); // Wait 15ms for boot-up

    while(1){
    // 2. Check Chip ID
    id = bmi160_ll_read(REG_CHIP_ID);
    if (id != 0xD1) {
        //return false; // ID Mismatch or connection error
        continue;
    }   
}

    // 3. Wake up Sensors (Bring to Normal Power Mode)
    bmi160_ll_write(REG_CMD, CMD_ACC_NORMAL);
    bmi160_delay_us(4000); // Wait for Accel wakeup (~3.8ms)
    
    bmi160_ll_write(REG_CMD, CMD_GYR_NORMAL);
    bmi160_delay_us(80000); // Wait for Gyro wakeup (~80ms)

    // 4. Apply Configuration
    bmi160_ll_write(REG_ACC_RANGE, config->acc_range);
    bmi160_ll_write(REG_GYR_RANGE, config->gyr_range);

    // Filter Settings (ODR & Bandwidth)
    // Bit 0x20 enables "Normal Bandwidth" mode (DLPF Active)
    // The lower 4 bits set the ODR.
    bmi160_ll_write(REG_ACC_CONF, 0x20 | config->acc_odr);
    bmi160_ll_write(REG_GYR_CONF, 0x20 | config->gyr_odr);

    return true;
}

void bmi160_read_all(BMI160_RawData *data) {
    uint8_t buf[12];
    // Burst Read 12 bytes starting from Gyro X LSB
    // Order: GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H...
    bmi160_ll_read_burst(REG_GYRO_DATA, buf, 12);

    // Combine bytes (Little Endian)
    data->gx = (int16_t)((buf[1] << 8) | buf[0]);
    data->gy = (int16_t)((buf[3] << 8) | buf[2]);
    data->gz = (int16_t)((buf[5] << 8) | buf[4]);
    
    data->ax = (int16_t)((buf[7] << 8) | buf[6]);
    data->ay = (int16_t)((buf[9] << 8) | buf[8]);
    data->az = (int16_t)((buf[11] << 8) | buf[10]);
}

void bmi160_convert_to_si(const BMI160_RawData *raw, float *gyro_si, float *accel_si) {
    // Hardcoded scales for 2000dps and 16g.
    // Ideally, these should be derived from the config struct.
    
    // Gyro: 2000dps -> 16.4 LSB/deg/s
    // Conversion factor: (1 / 16.4) * (PI / 180) = 0.0010642
    const float gyro_scale = 0.0010642f; 
    
    // Accel: 16g -> 2048 LSB/g
    // Conversion factor: (1 / 2048) * 9.80665 = 0.004788
    const float accel_scale = 0.004788f;
    
    gyro_si[0] = raw->gx * gyro_scale;
    gyro_si[1] = raw->gy * gyro_scale;
    gyro_si[2] = raw->gz * gyro_scale;

    accel_si[0] = raw->ax * accel_scale;
    accel_si[1] = raw->ay * accel_scale;
    accel_si[2] = raw->az * accel_scale;
}