#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include "FreeRTOS.h"
#include "task.h"
#include "clock_config.h"

#define LED_PORT GPIOB
#define LED_PIN  GPIO2

/* FreeRTOS Görevi: LED Blink */
static void task_blink(void *args) {
    (void)args;
    
    while (1) {
        gpio_toggle(LED_PORT, LED_PIN);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

int main(void) {

    // Configure clok
    (void)clock_config();

    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

    /* 4. Görev Oluştur */
    /* Stack boyutu: 128 kelime, Öncelik: 2 */
    xTaskCreate(task_blink, "LED_Task", 128, NULL, 2, NULL);

    /* 5. Motorları Çalıştır! (RTOS Başlat) */
    vTaskStartScheduler();

    /* Kod buraya asla gelmemeli */
    while (1);
    return 0;
}