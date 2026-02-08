#include <stdint.h>
#include "start.h"
#include "ari_tasks.h"
#include "FreeRTOS.h"
#include "task.h"


int main(void) {
    uint8_t err;

    // Setup device
    err = setup_device();

    // Start blink task
    blink_task_start();
    
    /* Start RTOS */
    vTaskStartScheduler();

    while (1);
    return 0;
}