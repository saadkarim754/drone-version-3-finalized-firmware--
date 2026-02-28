/**
 * @file mavlink_types.h
 * @brief MAVLink type definitions and constants
 */

#ifndef MAVESPSTM_MAVLINK_TYPES_H
#define MAVESPSTM_MAVLINK_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// MAVLink 2 Protocol Constants
// ============================================================================

#define MAVLINK_STX_V2 0xFD
#define MAVLINK_STX_V1 0xFE
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_SIGNATURE_BLOCK_LEN 13
#define MAVLINK_CORE_HEADER_LEN 9
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1)
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)
#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN)

// ============================================================================
// MAVLink Message IDs
// ============================================================================

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYSTEM_TIME 2
#define MAVLINK_MSG_ID_SET_MODE 11
#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_SCALED_PRESSURE 29
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_STATUSTEXT 253

// ============================================================================
// MAVLink Component IDs
// ============================================================================

#define MAV_COMP_ID_ALL 0
#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_COMP_ID_ONBOARD_COMPUTER 191
#define MAV_COMP_ID_ONBOARD_COMPUTER2 192
#define MAV_COMP_ID_ONBOARD_COMPUTER3 193
#define MAV_COMP_ID_ONBOARD_COMPUTER4 194

// ============================================================================
// MAVLink System Types (MAV_TYPE)
// ============================================================================

#define MAV_TYPE_GENERIC 0
#define MAV_TYPE_FIXED_WING 1
#define MAV_TYPE_QUADROTOR 2
#define MAV_TYPE_COAXIAL 3
#define MAV_TYPE_HELICOPTER 4
#define MAV_TYPE_GROUND_ROVER 10
#define MAV_TYPE_SUBMARINE 12
#define MAV_TYPE_HEXAROTOR 13
#define MAV_TYPE_OCTOROTOR 14
#define MAV_TYPE_ONBOARD_CONTROLLER 18
#define MAV_TYPE_GCS 6

// ============================================================================
// MAVLink Autopilot Types (MAV_AUTOPILOT)
// ============================================================================

#define MAV_AUTOPILOT_GENERIC 0
#define MAV_AUTOPILOT_ARDUPILOTMEGA 3
#define MAV_AUTOPILOT_PX4 12

// ============================================================================
// MAVLink Modes
// ============================================================================

#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#define MAV_MODE_PREFLIGHT 0

// MAVLink Parameter Types
#define MAV_PARAM_TYPE_REAL32 9

// Force arm magic value
#define MAV_ARM_FORCE_MAGIC 21196

// ============================================================================
// MAVLink System States (MAV_STATE)
// ============================================================================

#define MAV_STATE_UNINIT 0
#define MAV_STATE_BOOT 1
#define MAV_STATE_CALIBRATING 2
#define MAV_STATE_STANDBY 3
#define MAV_STATE_ACTIVE 4
#define MAV_STATE_CRITICAL 5
#define MAV_STATE_EMERGENCY 6
#define MAV_STATE_POWEROFF 7
#define MAV_STATE_FLIGHT_TERMINATION 8

// ============================================================================
// Parser States
// ============================================================================

typedef enum {
    MAVLINK_PARSE_STATE_UNINIT = 0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,
    MAVLINK_PARSE_STATE_GOT_MSGID2,
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC,
    MAVLINK_PARSE_STATE_SIGNATURE_WAIT
} mavlink_parse_state_t;

typedef enum {
    MAVLINK_FRAMING_INCOMPLETE = 0,
    MAVLINK_FRAMING_OK = 1,
    MAVLINK_FRAMING_BAD_CRC = 2,
    MAVLINK_FRAMING_BAD_SIGNATURE = 3
} mavlink_framing_t;

// ============================================================================
// MAVLink Message Structure
// ============================================================================

typedef struct __mavlink_message {
    uint16_t checksum;      // Checksum
    uint8_t magic;          // Protocol magic marker
    uint8_t len;            // Length of payload
    uint8_t incompat_flags; // Incompatibility flags
    uint8_t compat_flags;   // Compatibility flags
    uint8_t seq;            // Sequence number
    uint8_t sysid;          // System ID
    uint8_t compid;         // Component ID
    uint32_t msgid:24;      // Message ID (24 bits)
    uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN + 7) / 8];
    uint8_t ck[2];          // CRC bytes
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
} mavlink_message_t;

// ============================================================================
// MAVLink Status Structure
// ============================================================================

typedef struct __mavlink_status {
    uint8_t msg_received;
    uint8_t buffer_overrun;
    uint8_t parse_error;
    mavlink_parse_state_t parse_state;
    uint8_t packet_idx;
    uint8_t current_rx_seq;
    uint8_t current_tx_seq;
    uint16_t packet_rx_success_count;
    uint16_t packet_rx_drop_count;
    uint8_t flags;
    uint8_t signature_wait;
    struct __mavlink_signing *signing;
    struct __mavlink_signing_streams *signing_streams;
} mavlink_status_t;

#endif // MAVESPSTM_MAVLINK_TYPES_H
