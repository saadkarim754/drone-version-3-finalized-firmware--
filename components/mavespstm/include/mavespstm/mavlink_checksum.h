/**
 * @file mavlink_checksum.h
 * @brief X.25 CRC checksum implementation for MAVLink
 */

#ifndef MAVESPSTM_MAVLINK_CHECKSUM_H
#define MAVESPSTM_MAVLINK_CHECKSUM_H

#include <stdint.h>

/**
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 * @param data new byte to hash
 * @param crc_accum the already accumulated checksum
 */
static inline void crc_accumulate(uint8_t data, uint16_t *crc_accum)
{
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crc_accum & 0xff);
    tmp ^= (tmp << 4);
    *crc_accum = (*crc_accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/**
 * @brief Initialize the buffer for the X.25 CRC
 */
static inline void crc_init(uint16_t *crc_accum)
{
    *crc_accum = 0xffff;
}

/**
 * @brief Calculates the X.25 checksum on a buffer
 */
static inline uint16_t crc_calculate(const uint8_t *buf, uint16_t len)
{
    uint16_t crc_accum = 0xffff;
    while (len--) {
        crc_accumulate(*buf++, &crc_accum);
    }
    return crc_accum;
}

/**
 * @brief Accumulate the X.25 CRC by adding a buffer
 */
static inline void crc_accumulate_buffer(uint16_t *crc_accum, const char *buf, uint16_t len)
{
    while (len--) {
        crc_accumulate(*buf++, crc_accum);
    }
}

#endif // MAVESPSTM_MAVLINK_CHECKSUM_H
