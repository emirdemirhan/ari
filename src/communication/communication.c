#include "ari_error.h"
#include "drivers/nrf24.h"
#include "communication.h"
#include "flightcontrol/manuever_limits.h"
#include "FreeRTOS.h"
#include "queue.h"


/* Command queue */
static QueueHandle_t _cmdqueue;


comm_err_t cmd_init(void)
{
    if (_cmdqueue == NULL) {
        
        _cmdqueue = xQueueCreate(1, sizeof(cmd));   // Length 1 in order to process only last command
    }
    
    return comm_success;
}


comm_err_t get_cmd(cmd *data)
{

    if (_cmdqueue == NULL)
    {
        return comm_not_init;
    }
    
    if (xQueuePeek(_cmdqueue, data, 0) == pdTRUE) {
        return comm_success;
    }
    

    return comm_fail;
}

comm_err_t gnp_controlcmd(void)
{
    cmd communication_cmd;
    cmd buffer_cmd;

    if (nrf24_data_available())
    {
        nrf24_read((uint8_t*)&buffer_cmd, NRF_PAYLOAD_SIZE);
    }

    else
    {
        return comm_datanotready;
    }

    communication_cmd.roll_deg = buffer_cmd.roll_deg * MAX_ROLL;
    communication_cmd.pitch_deg = buffer_cmd.pitch_deg * MAX_PITCH;
    communication_cmd.yaw_rate = buffer_cmd.yaw_rate * MAX_YAW_RATE;
    communication_cmd.throttle = buffer_cmd.throttle * MAX_THROTTLE;

    xQueueOverwrite(_cmdqueue, &communication_cmd);
    


    return comm_success;
}


