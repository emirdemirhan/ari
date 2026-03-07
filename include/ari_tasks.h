#ifndef ARI_TASKS_H
#define ARI_TASKS_H

#include "FreeRTOS.h"
#include "task.h"


// Define functions
void blink_task_start(void);
void communication_task_start(void);
void control_task_start(void);

// Define global handles
extern TaskHandle_t control_task_handle; 

#endif