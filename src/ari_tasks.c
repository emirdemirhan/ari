#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ari_tasks.h"
#include "peripherals.h"
#include "drivers/nrf24.h"
#include "ari_error.h"
#include"communication/communication.h"



/* FreeRTOS task: LED Blink */
static void task_blink(void *args) {
    (void)args;
    
    while (1) {
        gpio_toggle(LED_PORT, LED_PIN);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
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


/* FreeRTOS task: Control Task */
static void task_control(void *args) {
    (void)args;

    // Get reference values from communication task
    cmd reference_cmd;
    get_cmd(&reference_cmd);

    // Get_current_state();  <-- bu tarz bişey gelcek state estimation
    // Sonra reference ile aradaki farka PID uygulaması yapıcaz
    // Ama cascade PID lazım robustness arıyoruz
    // Konum kontrolü yapmaya çalışıyoruz hız ile cascade yapıcaz o yüzden önce iç döngü yani hız
    // Şimdi roll pitch birlikte olması lazım ama önce kalman kısmını yazalım sonra bu döngüyü kuralım
    


    
    
}


void communication_task_start(void)
{
    // Create blink task
    xTaskCreate(task_comm, "Com_Task", 128, NULL, 2, NULL);
}

void control_task_start(void)
{
    // Create control task
    xTaskCreate(task_control, "Control_Task", 128, NULL, 2, NULL);


}

void blink_task_start(void)
{
    // Create blink task
    xTaskCreate(task_blink, "LED_Task", 128, NULL, 2, NULL);
}

