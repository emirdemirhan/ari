#include <stdint.h>
#include "start.h"
#include "clock_config.h"
#include "peripherals.h"
#include "drivers/nrf24.h"

devicesetup_err_t setup_device(void)
{
    uint8_t err;

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
    

    return devicesetup_success;
}
