#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ari_tasks.h"
#include "peripherals.h"
#include "drivers/nrf24.h"
#include "ari_error.h"
#include "communication/communication.h"
#include "drivers/bmi160.h"
#include "flightcontrol/controller.h"
#include "simulation/hil_bridge.h"
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/iwdg.h>


/* Task Handles */
TaskHandle_t control_task_handle = NULL;


/* FreeRTOS task: LED Blink */
static void task_blink(void *args) {
    (void)args;
    
    // Watchdog'u besler (100 ms'de bir, IWDG süresi 500 ms) ve LED'i 1 Hz yakıp söndürür. Kontrol
    // görevi aynı öncelikte meşgul beklese de zaman dilimleme sayesinde bu görev çalışır.
    // Her uyanış kontrol görevinin USART okumasını birkaç us keser; 2 Mbps'te bir bayt 5 us
    // olduğundan bu bazen taşmaya (ORE) yol açar, bu yüzden gereğinden sık uyanmaz.
    uint32_t n = 0;
    while (1) {
        iwdg_reset();
        if (++n % 10 == 0) {
            gpio_toggle(LED_PORT, LED_PIN);
#if HIL_RX_DIAG
            hil_print_rx_stats();
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
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

            /* Belki kablo oynadı, tekrar dene */
            connection = nrf24_test_connection();

            gnp_controlcmd();

            
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


/* HIL lock-step: 1 ise kontrol döngüsü TIM3 yerine simülasyon paketleriyle ilerler
   (her paket bir 2 ms adımdır, simülatör saati yönetir). 0 ise eski davranış:
   önce TIM3 tikini bekler, sonra paketi okur; o sırada gelen baytlar kaybolabilir. */
#define HIL_LOCKSTEP 1

/* FreeRTOS task: Control Task */
static void task_control(void *args) {
    (void)args;

    HIL_Sim_Data_t sim_data;
    HIL_Fc_Data_t fc_data;

    controller_init();

    while (1)
    {
#if !HIL_LOCKSTEP
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0) continue;
#endif
        if (hil_receive_full_packet(&sim_data) == 0) continue;

        // Kontrol adımının süresini CPU çevrimi olarak ölç (DWT, delay_init ile açıldı)
        uint32_t start = DWT_CYCCNT;
        controller_step(&sim_data, &fc_data);
        fc_data.exec_cycles = DWT_CYCCNT - start;

        hil_send_fc_data(&fc_data);
    }
}

void communication_task_start(void)
{
    // Create blink task
    xTaskCreate(task_comm, "Com_Task", 128, NULL, 2, NULL);
}

void control_task_start(void)
{
    // Create control task
    xTaskCreate(task_control, "Control_Task", 1024*4, NULL, 2, &control_task_handle);


}

void blink_task_start(void)
{
    // Create blink task
    xTaskCreate(task_blink, "LED_Task", 128, NULL, 2, NULL);
}


