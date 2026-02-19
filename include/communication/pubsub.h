#ifndef PUBSUB_H
#define PUBSUB_H

#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <stdint.h>

// Define structs
typedef struct {
    float target_roll;
    float target_pitch;
    float target_yaw_rate;
    float thrust;
    uint8_t mode;
} pubsub_data_t;

// Define functions

/* set data to global static variable */
void pubsub_set(const pubsub_data_t* new_data);

/* get data from rc data topic */
void pubsub_get(pubsub_data_t* out_data);

#endif




#endif
