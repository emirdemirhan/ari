#include "ari_timer.h"
#include "peripherals.h"
#include "ari_tasks.h" // flight_task_handle'ı görebilmek için
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"

void init_500hz_timer(void)
{
    // 2. NVIC (Kesme Yöneticisi) Öncelik Ayarları
    // DİKKAT: FreeRTOS kullanırken ISR içinden FreeRTOS API'si çağıracaksan (vTaskNotify...), 
    // kesme önceliği configMAX_SYSCALL_INTERRUPT_PRIORITY değerinden daha düşük (rakamsal olarak büyük) olmalıdır!
    // STM32'de alt 4 bit donanımsal olarak sıfırdır, bu yüzden "<< 4" kaydırması yapıyoruz.
    nvic_set_priority(NVIC_TIM3_IRQ, 5 << 4); // Güvenli bir RTOS önceliği (5)
    nvic_enable_irq(NVIC_TIM3_IRQ);

    // 3. Timer Temel Mod Ayarları (Yukarı sayan, düz timer)
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    
    // 4. Frekans Matematiği (APB1 hattının 100 MHz'de çalıştığı varsayımıyla)
    // Prescaler: İşlemci saatini 1 MHz'e (mikrosaniye) düşür
    timer_set_prescaler(TIM3, 100 - 1); 
    
    // Period (ARR): 2000 mikrosaniyede bir taşma yapsın (1.000.000 / 2000us = 500 Hz)
    timer_set_period(TIM3, 2000 - 1);

    // 5. Taşma (Update) kesmesini aktif et
    timer_enable_irq(TIM3, TIM_DIER_UIE);

    // 6. Sayacı başlat!
    timer_enable_counter(TIM3);
}

void tim3_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Kesmenin "Update" (Taşma) kaynaklı olduğundan emin ol
    if (timer_get_flag(TIM3, TIM_SR_UIF)) {
        
        // Bayrağı hemen temizle (Yoksa sonsuz döngüye girer)
        timer_clear_flag(TIM3, TIM_SR_UIF);
        
        // Doğrudan PID taskına "Uyan!" sinyali gönder
        // Handle'ın içi dolu mu (Task yaratılmış mı) diye kontrol etmek hayat kurtarır
        if (control_task_handle != NULL) {
            vTaskNotifyGiveFromISR(control_task_handle, &xHigherPriorityTaskWoken);
        }
    }

    // Context Switch: Eğer uçuş taskının önceliği şu an bölünen işten yüksekse beklemeden geç
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}