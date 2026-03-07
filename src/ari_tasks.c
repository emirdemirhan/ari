#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ari_tasks.h"
#include "peripherals.h"
#include "drivers/nrf24.h"
#include "ari_error.h"
#include "communication/communication.h"
#include "drivers/bmi160.h"
#include "flightcontrol/stest_wrapper.h"
#include "simulation/hil_bridge.h"
#include "flightcontrol/pid.h"


/* Task Handles */
TaskHandle_t control_task_handle = NULL;


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
    
    

    uint8_t loop_tick = 0;

    // Get reference values from communication task
    //cmd reference_cmd;
    //get_cmd(&reference_cmd);

    // Get_current_state();  <-- bu tarz bişey gelcek state estimation
    // Sonra reference ile aradaki farka PID uygulaması yapıcaz
    // Ama cascade PID lazım robustness arıyoruz
    // Konum kontrolü yapmaya çalışıyoruz hız ile cascade yapıcaz o yüzden önce iç döngü yani hız
    // Şimdi roll pitch birlikte olması lazım ama önce kalman kısmını yazalım sonra bu döngüyü kuralım
    
    // Kalman filtresini deneyelim. Simülasyondan veri çekip uyguluyoruz.

    // init pid coeffs

    pid_coeffs v_pos;
    pid_coeffs v_spd;

    v_pos.Kp = 5;
    v_pos.Ki = 5;
    v_pos.Kd = 5;
    v_pos.e_prev = 0;
    v_pos.e_sum = 0;        

    v_spd.Kp = 5;
    v_spd.Ki = 5;
    v_spd.Kd = 5;
    v_spd.e_prev = 0;
    v_spd.e_sum = 0;

    HIL_Sim_Data_t sim_data;

    float heigth = 0, v_speed = 0, target_v = 0, target_u = 0;    
    while (1) 
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
        {
        
            hil_receive_full_packet(&sim_data);

            //hil_receive(sim_data);
            
            /*
            float roll, pitch, yaw;

            // --- ADIM C: Filtreye Gönder ---
            // Gyro verisiyle TAHMİN (Predict) yap (1kHz döngüde burası çalışmalı)
            kalman_predict(sim_data[0], sim_data[1], sim_data[2]); // dt = 0.001s (1kHz varsayım)

            // İvmeölçer verisiyle DÜZELTME (Update) yap
            kalman_update(sim_data[3], sim_data[4], sim_data[5]);
            
            // --- ADIM D: Açıyı Al ---
            
            kalman_getangles(&roll, &pitch, &yaw);

            // İlk adım olarak yükseklik kontrolü deneyeceğiz  

            // Kontrol işareti üretimi
            pid_coeffs K_angle, K_angle_rate;

            calculate_pid(&K_angle, sim_data[6], sim_data[7], Ts); 

            */

            // Denemek için sadece altitude


            // Predict (500hz)
            kalman_predict_altitude(0,0,sim_data.gyro[2]);

            if (loop_tick >= 10)
            {
                // Update (50hz)
                kalman_update_altitude(sim_data.altitude_z);
                kalman_getaltitude(&heigth, &v_speed);
                // Calculate outer loop
                target_v = calculate_pid(&v_pos, sim_data.rc_thrust, heigth, 0.02);
                loop_tick = 0;
                
                
            }
            
            kalman_getaltitude(&heigth, &v_speed);

            // Calculate inner loop
            target_u = calculate_pid(&v_spd, target_v, v_speed, 0.002);

            hil_send_packet(target_u);

            loop_tick++;

        
        }
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


