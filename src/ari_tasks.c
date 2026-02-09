#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ari_tasks.h"
#include "peripherals.h"
#include "drivers/nrf24.h"
#include "ari_error.h"


/* FreeRTOS task: LED Blink */
static void task_blink(void *args) {
    (void)args;
    
    while (1) {
        gpio_toggle(LED_PORT, LED_PIN);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void blink_task_start(void)
{
    // Create blink task
    xTaskCreate(task_blink, "LED_Task", 128, NULL, 2, NULL);
}

/* FreeRTOS task: Communication Task */
static void task_comm(void *args) {
    (void)args;

    /* 2. BAĞLANTI TESTİ (SPI Ping) */
    uint8_t connection = nrf24_test_connection();

    while (1) {
        if (connection == nrfsetup_success) {
            /* BAŞARILI: Yavaş Yanıp Sönme (1 saniye) */
            /* "Sakin ol şampiyon, her şey yolunda" */
            gpio_toggle(LED_PORT, LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        } else {
            /* BAŞARISIZ: Çok Hızlı Yanıp Sönme (100 ms) */
            /* "İmdat! Kabloları kontrol et!" */
            gpio_toggle(LED_PORT, LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(100)); 
            
            /* Belki kablo oynadı, tekrar dene */
            connection = nrf24_test_connection();
        }
    }

    
}

void communication_task_start(void)
{
    // Create blink task
    xTaskCreate(task_comm, "Com_Task", 128, NULL, 2, NULL);
}