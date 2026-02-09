/**
 * @file delay.c
 * @brief Implementation of DWT based delays.
 */

 #include "delay.h"
 #include <libopencm3/cm3/dwt.h>
 #include <libopencm3/cm3/scb.h>
 #include <libopencm3/cm3/scs.h>
 
 /* CPU Frequency  */
 #define CPU_FREQ_HZ   100000000
 #define CYCLES_PER_US (CPU_FREQ_HZ / 1000000)
 
 void delay_init(void) {
     /* 1. Enable Core Debug Trace (TRCENA) in DEMCR register */
     /* Without this, DWT registers are not accessible */
     SCS_DEMCR |= SCS_DEMCR_TRCENA;
 
     /* 2. Reset the Cycle Counter */
     DWT_CYCCNT = 0;
 
     /* 3. Enable the Cycle Counter (CYCCNTENA) */
     DWT_CTRL |= DWT_CTRL_CYCCNTENA;
 }
 
 void delay_us(uint32_t us) {
     uint32_t start_tick = DWT_CYCCNT;
     uint32_t ticks = us * CYCLES_PER_US; // 100 cycles per us
 
     /* Wait until the difference exceeds the required ticks.
      * Unsigned arithmetic handles overflow automatically. */
     while ((DWT_CYCCNT - start_tick) < ticks);
 }
 
 void delay_ms(uint32_t ms) {
     
    delay_us(1000*ms);
     
 }