#ifndef BMI160_H
#define BMI160_H

#include <stdint.h>
#include <stdbool.h>

/* --- SETTINGS AND ENUMS --- */

// Accelerometer Range
typedef enum {
    BMI160_ACCEL_RANGE_2G  = 0x03,
    BMI160_ACCEL_RANGE_4G  = 0x05,
    BMI160_ACCEL_RANGE_8G  = 0x08,
    BMI160_ACCEL_RANGE_16G = 0x0C // Recommended for Drones
} BMI160_AccelRange;

// Gyroscope Range
typedef enum {
    BMI160_GYRO_RANGE_250  = 0x03,
    BMI160_GYRO_RANGE_500  = 0x02,
    BMI160_GYRO_RANGE_1000 = 0x01,
    BMI160_GYRO_RANGE_2000 = 0x00 // Recommended for Drones
} BMI160_GyroRange;

// Hardware Filter (DLPF) Settings
// ODR: Output Data Rate
typedef enum {
    BMI160_ODR_100HZ  = 0x08,
    BMI160_ODR_200HZ  = 0x09,
    BMI160_ODR_400HZ  = 0x0A,
    BMI160_ODR_800HZ  = 0x0B, // Ideal for Drones (Low latency, good filtering)
    BMI160_ODR_1600HZ = 0x0C,
    BMI160_ODR_3200HZ = 0x0D
} BMI160_ODR;

// Configuration Structure
typedef struct {
    BMI160_AccelRange acc_range;
    BMI160_GyroRange  gyr_range;
    BMI160_ODR        acc_odr; // Accel Output Data Rate
    BMI160_ODR        gyr_odr; // Gyro Output Data Rate
} BMI160_Config;

// Raw Data Structure
typedef struct {
    int16_t gx, gy, gz; // Gyro Raw Data
    int16_t ax, ay, az; // Accel Raw Data
    int16_t temp;       // Temperature Raw Data
} BMI160_RawData;

/* --- FUNCTION PROTOTYPES --- */

/**
 * @brief Initializes the BMI160 sensor.
 * @param config Configuration struct containing ranges and ODR settings.
 * @return true if initialization successful (Chip ID verified), false otherwise.
 */
bool bmi160_init(const BMI160_Config *config);

/**
 * @brief Reads all sensor data (Gyro + Accel) in a single burst.
 * @param data Pointer to store the raw data.
 */
void bmi160_read_all(BMI160_RawData *data);

/**
 * @brief Converts raw LSB data to SI units (rad/s and m/s^2).
 * @param raw Pointer to raw data.
 * @param gyro_si Output array for gyro [x, y, z] in rad/s.
 * @param accel_si Output array for accel [x, y, z] in m/s^2.
 */
void bmi160_convert_to_si(const BMI160_RawData *raw, float *gyro_si, float *accel_si);

#endif // BMI160_H