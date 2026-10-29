#include <string.h>
#include "simulation/hil_protocol.h"


/* Decoder states */
enum {
    DEC_SYNC_1 = 0,
    DEC_SYNC_2,
    DEC_TYPE,
    DEC_LEN,
    DEC_PAYLOAD,
    DEC_CRC_LO,
    DEC_CRC_HI
};


uint16_t hil_crc16(const uint8_t *data, uint16_t len, uint16_t crc)
{
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}


uint16_t hil_encode_frame(uint8_t type, const void *payload, uint8_t len, uint8_t *out, uint16_t cap)
{
    uint16_t total = (uint16_t)len + HIL_FRAME_OVERHEAD;
    if (cap < total) return 0;

    out[0] = HIL_SYNC_1;
    out[1] = HIL_SYNC_2;
    out[2] = type;
    out[3] = len;
    memcpy(&out[4], payload, len);

    uint16_t crc = hil_crc16(&out[2], (uint16_t)(len + 2), 0xFFFFu);
    out[4 + len] = (uint8_t)(crc & 0xFF);
    out[5 + len] = (uint8_t)(crc >> 8);

    return total;
}


void hil_decoder_reset(hil_decoder_t *dec)
{
    uint32_t crc_errors = dec->crc_errors;
    memset(dec, 0, sizeof(*dec));
    dec->crc_errors = crc_errors;
    dec->state = DEC_SYNC_1;
}


uint8_t hil_decoder_push(hil_decoder_t *dec, uint8_t byte)
{
    switch (dec->state)
    {
    case DEC_SYNC_1:
        if (byte == HIL_SYNC_1) dec->state = DEC_SYNC_2;
        break;

    case DEC_SYNC_2:
        // A repeated first sync byte may still start a frame
        if (byte == HIL_SYNC_2) dec->state = DEC_TYPE;
        else if (byte != HIL_SYNC_1) dec->state = DEC_SYNC_1;
        break;

    case DEC_TYPE:
        dec->type = byte;
        dec->state = DEC_LEN;
        break;

    case DEC_LEN:
        dec->len = byte;
        dec->index = 0;
        dec->state = (byte == 0) ? DEC_CRC_LO : DEC_PAYLOAD;
        break;

    case DEC_PAYLOAD:
        dec->payload[dec->index++] = byte;
        if (dec->index >= dec->len) dec->state = DEC_CRC_LO;
        break;

    case DEC_CRC_LO:
        dec->crc_rx = byte;
        dec->state = DEC_CRC_HI;
        break;

    case DEC_CRC_HI:
    {
        dec->crc_rx |= (uint16_t)byte << 8;
        dec->state = DEC_SYNC_1;

        uint8_t head[2] = {dec->type, dec->len};
        uint16_t crc = hil_crc16(head, 2, 0xFFFFu);
        crc = hil_crc16(dec->payload, dec->len, crc);

        if (crc == dec->crc_rx) return 1;

        dec->crc_errors++;
        break;
    }

    default:
        dec->state = DEC_SYNC_1;
        break;
    }

    return 0;
}
