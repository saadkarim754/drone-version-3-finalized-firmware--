/**
 * @file mavlink_parser.h
 * @brief MAVLink message parser
 */

#ifndef MAVESPSTM_MAVLINK_PARSER_H
#define MAVESPSTM_MAVLINK_PARSER_H

#include "mavlink_types.h"
#include "mavlink_checksum.h"
#include "mavlink_messages.h"

// ============================================================================
// Parser Functions
// ============================================================================

/**
 * @brief Initialize a MAVLink status structure
 */
static inline void mavlink_status_init(mavlink_status_t *status) {
    status->msg_received = 0;
    status->buffer_overrun = 0;
    status->parse_error = 0;
    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
    status->packet_idx = 0;
    status->current_rx_seq = 0;
    status->current_tx_seq = 0;
    status->packet_rx_success_count = 0;
    status->packet_rx_drop_count = 0;
    status->flags = 0;
    status->signature_wait = 0;
    status->signing = NULL;
    status->signing_streams = NULL;
}

/**
 * @brief Parse a MAVLink byte stream
 * @param chan Channel number (unused, for compatibility)
 * @param c Byte to parse
 * @param msg Message buffer to fill
 * @param status Parser status
 * @return MAVLINK_FRAMING_OK when complete message received
 */
static inline uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *msg, mavlink_status_t *status) {
    (void)chan;  // Channel not used in simple implementation
    
    uint8_t *payload = (uint8_t *)msg->payload64;
    
    switch (status->parse_state) {
    case MAVLINK_PARSE_STATE_UNINIT:
    case MAVLINK_PARSE_STATE_IDLE:
        if (c == MAVLINK_STX_V2) {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            msg->magic = c;
            msg->len = 0;
            msg->incompat_flags = 0;
            msg->compat_flags = 0;
            status->packet_idx = 0;
            crc_init(&msg->checksum);
        } else if (c == MAVLINK_STX_V1) {
            // MAVLink v1 - not supported in this implementation
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_STX:
        msg->len = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_LENGTH:
        msg->incompat_flags = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
        msg->compat_flags = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
        msg->seq = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_SEQ:
        msg->sysid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_SYSID:
        msg->compid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_COMPID:
        msg->msgid = c;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID1:
        msg->msgid |= (uint32_t)c << 8;
        crc_accumulate(c, &msg->checksum);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID2:
        msg->msgid |= (uint32_t)c << 16;
        crc_accumulate(c, &msg->checksum);
        if (msg->len == 0) {
            // No payload
            crc_accumulate(mavlink_get_crc_extra(msg->msgid), &msg->checksum);
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
            status->packet_idx = 0;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_MSGID3:
        payload[status->packet_idx++] = c;
        crc_accumulate(c, &msg->checksum);
        if (status->packet_idx >= msg->len) {
            // Add CRC extra byte
            crc_accumulate(mavlink_get_crc_extra(msg->msgid), &msg->checksum);
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
        msg->ck[0] = c;
        if ((msg->checksum & 0xFF) != c) {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC;
            status->parse_error++;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_CRC1:
        msg->ck[1] = c;
        if ((msg->checksum >> 8) != c) {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_drop_count++;
            return MAVLINK_FRAMING_BAD_CRC;
        }
        // Check for signature (if incompat_flags has bit 0 set)
        if (msg->incompat_flags & 0x01) {
            status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;
            status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
        } else {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_success_count++;
            return MAVLINK_FRAMING_OK;
        }
        break;
        
    case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
        status->signature_wait--;
        if (status->signature_wait == 0) {
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            status->packet_rx_success_count++;
            return MAVLINK_FRAMING_OK;
        }
        break;
        
    case MAVLINK_PARSE_STATE_GOT_BAD_CRC:
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        status->packet_rx_drop_count++;
        return MAVLINK_FRAMING_BAD_CRC;
        
    default:
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        break;
    }
    
    return MAVLINK_FRAMING_INCOMPLETE;
}

#endif // MAVESPSTM_MAVLINK_PARSER_H
