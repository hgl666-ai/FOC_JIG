#include "protocol_frame.h"
#include <string.h>

uint16_t Frame_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001u)
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len)
{
    buf[0] = FRAME_HDR0;
    buf[1] = FRAME_HDR1;
    buf[2] = (uint8_t)(cmd >> 8);
    buf[3] = (uint8_t)(cmd);
    buf[4] = (uint8_t)(seq >> 8);
    buf[5] = (uint8_t)(seq);
    buf[6] = (uint8_t)(len >> 8);
    buf[7] = (uint8_t)(len);
    if (len > 0 && data != 0) {
        memcpy(&buf[8], data, len);
    }

    uint16_t total = (uint16_t)(8u + len);
    uint16_t crc   = Frame_CRC16(buf, total);
    buf[total]      = (uint8_t)(crc >> 8);   /* CRC 高字节在前 */
    buf[total + 1u] = (uint8_t)(crc);

    return (uint16_t)(total + 2u);
}

uint8_t Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                    uint16_t *cmd, uint16_t *seq,
                    uint8_t *data, uint16_t *len)
{
    if (raw == 0 || raw_len < FRAME_OVERHEAD) return 0;
    if (raw[0] != FRAME_HDR0 || raw[1] != FRAME_HDR1) return 0;

    uint16_t dlen = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    if (raw_len != (uint16_t)(8u + dlen + 2u)) return 0;

    uint16_t calc = Frame_CRC16(raw, (uint16_t)(8u + dlen));
    uint16_t recv = (uint16_t)(((uint16_t)raw[8u + dlen] << 8) | raw[9u + dlen]);
    if (calc != recv) return 0;

    if (cmd)  *cmd = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    if (seq)  *seq = (uint16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    if (data && dlen > 0) memcpy(data, &raw[8], dlen);
    if (len)  *len = dlen;

    return 1;
}
