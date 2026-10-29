#include <stdint.h>
#include "start.h"
#include "clock_config.h"
#include "peripherals.h"
#include "drivers/nrf24.h"
#include "drivers/bmi160.h"

devicesetup_err_t setup_device(void)
{
    volatile uint8_t err;

    /* Configure clocks */
    err = clock_config();

    if (err != clocksetup_success)
    {
        return devicesetup_fail;
    }

    /* Initialize peripherals */
    setup_perihperals();

    /* Setup NRF24 for communication */
    nrf24_init();

    BMI160_Config sensor_conf = {
        .acc_range = BMI160_ACCEL_RANGE_16G, // Drone titrer, yüksek range iyidir
        .gyr_range = BMI160_GYRO_RANGE_2000, // Hızlı dönüşler için 2000dps
        .acc_odr   = BMI160_ODR_800HZ,       // 800Hz Output Data Rate
        .gyr_odr   = BMI160_ODR_800HZ        // Donanımsal Filtre (DLPF) burada aktif
    };

    /* BMI160 init */
    if (!bmi160_init(&sensor_conf))
    {
        return devicesetup_fail;
    }

    return devicesetup_success;
}
