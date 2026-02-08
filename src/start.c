#include <stdint.h>
#include "start.h"
#include "clock_config.h"
#include "peripherals.h"

devicesetup_err_t setup_device(void)
{
    uint8_t err;

    err = clock_config();

    if (err != clocksetup_success)
    {
        return devicesetup_fail;
    }

    setup_perihperals();
    

    return devicesetup_success;
}
