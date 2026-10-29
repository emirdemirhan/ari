#include <stdint.h>
#include "start.h"
#include "ari_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "simulation/hil_bridge.h"
#include <libopencm3/stm32/iwdg.h>

int main(void) {
    uint8_t err;

    // Setup device
    err = setup_device();

    /* Configure usart for simulation */
    start_usb_hil_system();
 

    // Start blink task
    blink_task_start();

    // Start control task
    control_task_start();
    
    /* Bağımsız watchdog (IWDG, 500 ms): LED görevi her 50 ms'de besler. Kart takılırsa (kesmeler
       kapalı kalırsa, hata döngüsü, zamanlayıcı durursa) 0.5 s içinde kendini sıfırlar ve açılış
       mesajında sebebi "IWDG" olarak yazar. */
    iwdg_set_period_ms(500);
    iwdg_start();

    /* Start RTOS */
    vTaskStartScheduler();

    while (1);
    return 0;
}