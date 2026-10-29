#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

/*
 * HIL/PIL wire protocol, version 2. Pure C, no hardware access: the firmware (hil_bridge.c)
 * and the PC-side SIL library (sil/sil_lib.c) both use it, and sim/arisim/protocol.py mirrors it.
 * Any change here must be made in protocol.py too; the SIL run fails on a mismatch.
 *
 * Frame:  0xAA 0xBB | type (1) | len (1) | payload (len) | crc16 (2, little endian)
 *         crc16 = CRC-16/CCITT-FALSE over type, len and payload.
 * All multi-byte fields are little endian; floats are IEEE-754 single precision.
 *
 * Lock-step: the PC sends one HIL_Sim_Data_t per simulation step and waits for the
 * HIL_Fc_Data_t carrying the same seq. A repeated seq means the PC did not get the reply;
 * the firmware then resends its cached reply instead of running the controller again.
 */

#include <stdint.h>

#define HIL_SYNC_1              0xAA
#define HIL_SYNC_2              0xBB

#define HIL_TYPE_SIM_DATA       0x01    /* PC  -> MCU: HIL_Sim_Data_t */
#define HIL_TYPE_RESET          0x02    /* PC  -> MCU: no payload, reboot the MCU (fresh state for a new run) */
#define HIL_TYPE_FC_DATA        0x81    /* MCU -> PC : HIL_Fc_Data_t  */

#define HIL_FRAME_OVERHEAD      6       /* sync (2) + type + len + crc (2) */
#define HIL_MAX_PAYLOAD         255

/* HIL_Sim_Data_t.flags */
#define HIL_SIM_FLAG_RANGE_VALID    (1u << 0)   /* altitude_z is a real return, not the out-of-range value */
#define HIL_SIM_FLAG_RANGE_NEW      (1u << 1)   /* altitude_z was sampled during this step */
#define HIL_SIM_FLAG_BARO_VALID     (1u << 2)   /* baro_alt is a real reading (sensor working) */
#define HIL_SIM_FLAG_BARO_NEW       (1u << 3)   /* baro_alt was sampled during this step */

/* HIL_Fc_Data_t.flags */
#define HIL_FC_FLAG_USE_ESKF        (1u << 0)   /* the ESKF feeds the controller */
#define HIL_FC_FLAG_CALIBRATING     (1u << 1)   /* boot IMU calibration running: motors off, keep still */
#define HIL_FC_FLAG_LANDED          (1u << 2)   /* landed state: motors idle until the altitude reference rises */
#define HIL_FC_FLAG_RANGE_LOST      (1u << 3)   /* no valid rangefinder (out of range or blocked): height from the barometer */
#define HIL_FC_FLAG_HEIGHT_FAILSAFE (1u << 4)   /* neither rangefinder nor barometer: hover-thrust hold, slow descent, tilt limited */


/* Simulator -> flight controller, one per simulation step */
typedef struct __attribute__((packed)) {
    uint32_t seq;          // Step counter, increments by one per step
    uint32_t time_us;      // Simulation time (us)

    // 1. Sensors (body frame: x forward, y left, z up)
    float accel[3];        // Specific force (m/s^2), +9.81 on z at rest
    float gyro[3];         // Angular rate (rad/s)
    float altitude_z;      // Rangefinder distance along body -z (m)
    float baro_alt;        // Barometric altitude (m above sea level; only changes are used)

    // 2. RC / pilot commands, already in firmware conventions
    float rc_roll;         // Roll stick, -1..1 (positive: roll right, left side up)
    float rc_pitch;        // Pitch stick, -1..1 (positive: nose down, fly forward)
    float rc_yaw_rate;     // Yaw rate (rad/s, positive: counter-clockwise seen from above)
    float rc_thrust;       // Altitude reference (m)

    // 3. Ground truth, for debugging only
    float true_roll;
    float true_pitch;
    float true_yaw;

    uint32_t flags;        // HIL_SIM_FLAG_*
} HIL_Sim_Data_t;


/* Flight controller -> simulator, one per received HIL_Sim_Data_t */
typedef struct __attribute__((packed)) {
    uint32_t seq;          // seq of the HIL_Sim_Data_t this answers

    float motor[4];        // M1..M4 command (%, 0..100)

    // Estimator outputs, both filters always run
    float kf_rpy[3];       // Linear Kalman roll, pitch, yaw (rad)
    float eskf_rpy[3];     // ESKF roll, pitch, yaw (rad)
    float kf_alt;          // Linear Kalman height (m)
    float kf_vz;           // Linear Kalman vertical speed (m/s)
    float eskf_alt;        // ESKF-path height (m)
    float eskf_vz;         // ESKF-path vertical speed (m/s)
    float eskf_vel[2];     // ESKF velocity relative to the air along body x, y (m/s)

    // Controller internals
    float target_vz;       // Outer altitude loop output (m/s)
    float total_thrust;    // Collective command before mixing (%)
    float roll_rate_ref;   // Roll angle loop output (rad/s)
    float pitch_rate_ref;  // Pitch angle loop output (rad/s)

    uint32_t exec_cycles;  // CPU cycles spent in controller_step() (0 in SIL)
    uint32_t flags;        // HIL_FC_FLAG_*
} HIL_Fc_Data_t;


/* Streaming frame decoder: feed it one byte at a time */
typedef struct {
    uint8_t state;
    uint8_t type;
    uint8_t len;
    uint16_t index;
    uint16_t crc_rx;
    uint8_t payload[HIL_MAX_PAYLOAD];
    uint32_t crc_errors;   // Frames dropped because of a CRC mismatch
} hil_decoder_t;


#ifdef __cplusplus
extern "C" {
#endif

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) */
uint16_t hil_crc16(const uint8_t *data, uint16_t len, uint16_t crc);

/* Writes a complete frame into out (capacity cap). Returns the frame length, 0 if it does not fit. */
uint16_t hil_encode_frame(uint8_t type, const void *payload, uint8_t len, uint8_t *out, uint16_t cap);

void hil_decoder_reset(hil_decoder_t *dec);

/* Returns 1 when this byte completes a frame with a valid CRC; dec->type/len/payload hold it */
uint8_t hil_decoder_push(hil_decoder_t *dec, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
