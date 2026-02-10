#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "ari_error.h"
#include <stdint.h>


#define NRF_PAYLOADSIZE 16
#define PAYLOAD_START_BYTE 1
#define ARI_CMDTYPE_DEFAULT 0


typedef struct __attribute__((packed))
{
    float roll_deg;
    float pitch_deg;
    float yaw_rate;
    float throttle;
    bool is_armed;

}cmd;


comm_err_t cmd_init(void);
comm_err_t get_cmd(cmd* data);
comm_err_t gnp_controlcmd(void);
static inline comm_err_t parse_cmd(uint8_t* payload);



#endif