#include <stdint.h>
#include "start.h"
#include "ari_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "simulation/hil_bridge.h"

int main(void) {
    uint8_t err;

    // Setup device
    err = setup_device();

    /* Configure usart for simulation */
    start_usb_hil_system();
 

    // Start blink task
    control_task_start();
    
    /* Start RTOS */
    vTaskStartScheduler();

    while (1);
    return 0;
}