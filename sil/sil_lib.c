/*
 * SIL (software-in-the-loop) library: the flight controller step built for the PC.
 *
 * Built with `make sil` into build/sil/libari_sil.so and loaded by the simulator
 * (sim/arisim/links.py). Bytes from the simulator go through the same frame decoder and
 * controller_step() as on the board, and the reply is encoded with the same encoder,
 * so a protocol mismatch between C and Python fails here as well.
 *
 * The Kalman/EKF objects are globals without a reset, so every run loads a fresh copy
 * of this library.
 */

#include <string.h>
#include "simulation/hil_protocol.h"
#include "flightcontrol/controller.h"


static hil_decoder_t decoder;
static uint8_t last_reply[sizeof(HIL_Fc_Data_t) + HIL_FRAME_OVERHEAD];
static int last_reply_len = 0;
static uint32_t last_reply_seq = 0;
static int initialized = 0;


/* Payload sizes, so the simulator can check them against its own struct formats */
void sil_struct_sizes(int *sim_data_size, int *fc_data_size)
{
    *sim_data_size = (int)sizeof(HIL_Sim_Data_t);
    *fc_data_size = (int)sizeof(HIL_Fc_Data_t);
}

uint32_t sil_crc_errors(void)
{
    return decoder.crc_errors;
}

/*
 * Feeds n bytes from the simulator. When they complete a HIL_Sim_Data_t frame, runs one
 * controller step and writes the reply frame to out. Returns the reply length, or 0 when
 * no reply is due (incomplete or invalid frame).
 */
int sil_exchange(const uint8_t *in, int n, uint8_t *out, int cap)
{
    if (!initialized)
    {
        hil_decoder_reset(&decoder);
        controller_init();
        initialized = 1;
    }

    int reply_len = 0;

    for (int i = 0; i < n; i++)
    {
        if (!hil_decoder_push(&decoder, in[i])) continue;
        if (decoder.type != HIL_TYPE_SIM_DATA || decoder.len != sizeof(HIL_Sim_Data_t)) continue;

        HIL_Sim_Data_t sim_data;
        memcpy(&sim_data, decoder.payload, sizeof(sim_data));

        // Same duplicate handling as the firmware (hil_bridge.c)
        if (last_reply_len == 0 || sim_data.seq != last_reply_seq)
        {
            HIL_Fc_Data_t fc_data;
            controller_step(&sim_data, &fc_data);

            last_reply_len = hil_encode_frame(HIL_TYPE_FC_DATA, &fc_data, sizeof(fc_data),
                                              last_reply, sizeof(last_reply));
            last_reply_seq = sim_data.seq;
        }

        if (last_reply_len > cap) return -1;
        memcpy(out, last_reply, (size_t)last_reply_len);
        reply_len = last_reply_len;
    }

    return reply_len;
}
