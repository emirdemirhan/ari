#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Clock Speed (168 MHz)
#define configCPU_CLOCK_HZ             ( ( unsigned long ) 168000000 )
#define configTICK_RATE_HZ             ( ( TickType_t ) 1000 ) // 1ms Tick

// Basic config
#define configUSE_PREEMPTION           1
#define configUSE_IDLE_HOOK            0
#define configUSE_TICK_HOOK            0
#define configMAX_PRIORITIES           ( 5 )
#define configMINIMAL_STACK_SIZE       ( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE          ( ( size_t ) ( 20 * 1024 ) ) // 20KB Heap
#define configMAX_TASK_NAME_LEN        ( 16 )
#define configUSE_16_BIT_TICKS         0
#define configIDLE_SHOULD_YIELD        1

// Cortex-M4F FPU config
#define configFPU_ISR_USED             1

#define vPortSVCHandler    sv_call_handler
#define xPortPendSVHandler pend_sv_handler
#define xPortSysTickHandler sys_tick_handler

#endif /* FREERTOS_CONFIG_H */
