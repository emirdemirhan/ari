/**
 * @file delay.h
 * @brief DWT (Data Watchpoint & Trace) based precision delay library.
 * @note  Uses Cortex-M4 Core Debug Cycle Counter. No Timers/Interrupts used.
 */

 #ifndef DELAY_H
 #define DELAY_H
 
 #include <stdint.h>
 
 /**
  * @brief Initializes the DWT Cycle Counter.
  * @note  Must be called once at startup (before using delay_us).
  */
 void delay_init(void);
 
 /**
  * @brief Blocking microsecond delay.
  * @param us Microseconds to wait.
  */
 void delay_us(uint32_t us);
 
 /**
  * @brief Blocking millisecond delay.
  * @warning Blocks CPU. Prefer vTaskDelay() if FreeRTOS scheduler is running.
  * @param ms Milliseconds to wait.
  */
 void delay_ms(uint32_t ms);
 
 #endif /* DELAY_H */