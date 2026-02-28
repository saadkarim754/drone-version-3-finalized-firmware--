/**
 * @file mavlink_messages.h
 * @brief MAVLink message structures and encoding/decoding functions
 */

#ifndef MAVESPSTM_MAVLINK_MESSAGES_H
#define MAVESPSTM_MAVLINK_MESSAGES_H

#include "mavlink_types.h"
#include "mavlink_checksum.h"

// ============================================================================
// CRC Extra Values
// ============================================================================

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9

#define MAVLINK_MSG_ID_ATTITUDE_CRC 39
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28

#define MAVLINK_MSG_ID_SET_MODE 11
#define MAVLINK_MSG_ID_SET_MODE_CRC 89
#define MAVLINK_MSG_ID_SET_MODE_LEN 6

#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_LONG_CRC 152
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33

#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_COMMAND_ACK_CRC 143
#define MAVLINK_MSG_ID_COMMAND_ACK_LEN 3

#define MAVLINK_MSG_ID_PARAM_SET_CRC 168
#define MAVLINK_MSG_ID_PARAM_SET_LEN 23

#define MAVLINK_MSG_ID_PARAM_VALUE_CRC 220
#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC 124
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN 18

#define MAVLINK_MSG_ID_SYS_STATUS_CRC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31

#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30

#define MAVLINK_MSG_ID_SCALED_PRESSURE_CRC 115
#define MAVLINK_MSG_ID_SCALED_PRESSURE_LEN 14

#define MAVLINK_MSG_ID_VFR_HUD_CRC 20
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC 104
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28

#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_MSG_ID_STATUSTEXT_CRC 83
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 54

// MAVLink Commands
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#define MAV_CMD_DO_SET_MODE 176

// MAVLink Command Results
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_TEMPORARILY_REJECTED 1
#define MAV_RESULT_DENIED 2
#define MAV_RESULT_UNSUPPORTED 3
#define MAV_RESULT_FAILED 4
#define MAV_RESULT_IN_PROGRESS 5

// ============================================================================
// Message Structures
// ============================================================================

/**
 * @brief SYS_STATUS message structure (MSG ID 1)
 * Subset of fields we care about (battery)
 */
typedef struct __mavlink_sys_status_t {
    uint16_t voltage_battery;    // Battery voltage [mV]
    int16_t current_battery;     // Battery current [cA] (10 * mA), -1 if not measured
    int8_t battery_remaining;    // Battery remaining [%] 0-100, -1 if not estimated
} mavlink_sys_status_t;

/**
 * @brief Heartbeat message structure
 */
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;    // Custom mode (flight mode)
    uint8_t type;            // Vehicle type (MAV_TYPE)
    uint8_t autopilot;       // Autopilot type (MAV_AUTOPILOT)
    uint8_t base_mode;       // Base mode flags
    uint8_t system_status;   // System status (MAV_STATE)
    uint8_t mavlink_version; // MAVLink version
} mavlink_heartbeat_t;

/**
 * @brief Attitude message structure (MSG ID 30)
 * Contains vehicle attitude (roll, pitch, yaw) in radians
 */
typedef struct __mavlink_attitude_t {
    uint32_t time_boot_ms;   // Timestamp (ms since boot)
    float roll;              // Roll angle (-pi..+pi) [rad]
    float pitch;             // Pitch angle (-pi..+pi) [rad]
    float yaw;               // Yaw angle (-pi..+pi) [rad]
    float rollspeed;         // Roll angular speed [rad/s]
    float pitchspeed;        // Pitch angular speed [rad/s]
    float yawspeed;          // Yaw angular speed [rad/s]
} mavlink_attitude_t;

/**
 * @brief COMMAND_LONG message structure (MSG ID 76)
 */
typedef struct __mavlink_command_long_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
} mavlink_command_long_t;

/**
 * @brief COMMAND_ACK message structure (MSG ID 77)
 */
typedef struct __mavlink_command_ack_t {
    uint16_t command;
    uint8_t result;
} mavlink_command_ack_t;

/**
 * @brief STATUSTEXT message structure (MSG ID 253)
 */
typedef struct __mavlink_statustext_t {
    uint8_t severity;
    char text[50];
} mavlink_statustext_t;

/**
 * @brief PARAM_SET message structure (MSG ID 23)
 */
typedef struct __mavlink_param_set_t {
    float param_value;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t param_type;
} mavlink_param_set_t;

/**
 * @brief PARAM_VALUE message structure (MSG ID 22)
 */
typedef struct __mavlink_param_value_t {
    float param_value;
    uint16_t param_count;
    uint16_t param_index;
    char param_id[16];
    uint8_t param_type;
} mavlink_param_value_t;

/**
 * @brief RC_CHANNELS_OVERRIDE message structure (MSG ID 70)
 */
typedef struct __mavlink_rc_channels_override_t {
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint8_t target_system;
    uint8_t target_component;
} mavlink_rc_channels_override_t;

/**
 * @brief GPS_RAW_INT message structure (MSG ID 24)
 * Raw GPS data from the sensor
 */
typedef struct __mavlink_gps_raw_int_t {
    uint64_t time_usec;          // Timestamp (UNIX epoch) [us]
    int32_t lat;                 // Latitude (WGS84, degE7)
    int32_t lon;                 // Longitude (WGS84, degE7)
    int32_t alt;                 // Altitude MSL [mm]
    uint16_t eph;                // GPS HDOP * 100
    uint16_t epv;                // GPS VDOP * 100
    uint16_t vel;                // Ground speed [cm/s]
    uint16_t cog;                // Course over ground [cdeg]
    uint8_t fix_type;            // GPS fix type: 0=no, 1=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
    uint8_t satellites_visible;  // Number of satellites visible
} mavlink_gps_raw_int_t;

/**
 * @brief SCALED_PRESSURE message structure (MSG ID 29)
 * Barometric pressure and temperature
 */
typedef struct __mavlink_scaled_pressure_t {
    uint32_t time_boot_ms;   // Timestamp [ms since boot]
    float press_abs;         // Absolute pressure [hPa / millibar]
    float press_diff;        // Differential pressure [hPa]
    int16_t temperature;     // Temperature [cdegC] (centi-degrees)
} mavlink_scaled_pressure_t;

/**
 * @brief VFR_HUD message structure (MSG ID 74)
 * Metrics typically displayed on a heads-up display
 */
typedef struct __mavlink_vfr_hud_t {
    float airspeed;       // Current airspeed [m/s]
    float groundspeed;    // Current ground speed [m/s]
    float alt;            // Current altitude MSL [m]
    float climb;          // Current climb rate [m/s]
    int16_t heading;      // Current heading [deg] 0..359
    uint16_t throttle;    // Current throttle [%] 0..100
} mavlink_vfr_hud_t;

/**
 * @brief GLOBAL_POSITION_INT message structure (MSG ID 33)
 * Fused GPS + IMU position estimate
 */
typedef struct __mavlink_global_position_int_t {
    uint32_t time_boot_ms;  // Timestamp [ms since boot]
    int32_t lat;            // Latitude (WGS84, degE7)
    int32_t lon;            // Longitude (WGS84, degE7)
    int32_t alt;            // Altitude MSL [mm]
    int32_t relative_alt;   // Altitude above home [mm]
    int16_t vx;             // Ground speed X (North) [cm/s]
    int16_t vy;             // Ground speed Y (East) [cm/s]
    int16_t vz;             // Ground speed Z (Down) [cm/s]
    uint16_t hdg;           // Vehicle heading [cdeg] 0..35999
} mavlink_global_position_int_t;

// ============================================================================
// CRC Extra Lookup Function
// ============================================================================

/**
 * @brief Get CRC extra byte for a message ID
 * @param msgid Message ID
 * @return CRC extra byte
 */
static inline uint8_t mavlink_get_crc_extra(uint32_t msgid) {
    switch (msgid) {
        case 0:   return 50;   // HEARTBEAT
        case 1:   return 124;  // SYS_STATUS
        case 2:   return 137;  // SYSTEM_TIME
        case 4:   return 237;  // PING
        case 5:   return 217;  // CHANGE_OPERATOR_CONTROL
        case 6:   return 104;  // CHANGE_OPERATOR_CONTROL_ACK
        case 7:   return 119;  // AUTH_KEY
        case 11:  return 89;   // SET_MODE
        case 20:  return 214;  // PARAM_REQUEST_READ
        case 21:  return 159;  // PARAM_REQUEST_LIST
        case 22:  return 220;  // PARAM_VALUE
        case 23:  return 168;  // PARAM_SET
        case 24:  return 24;   // GPS_RAW_INT
        case 25:  return 23;   // GPS_STATUS
        case 26:  return 170;  // SCALED_IMU
        case 27:  return 144;  // RAW_IMU
        case 28:  return 67;   // RAW_PRESSURE
        case 29:  return 115;  // SCALED_PRESSURE
        case 30:  return 39;   // ATTITUDE
        case 31:  return 246;  // ATTITUDE_QUATERNION
        case 32:  return 185;  // LOCAL_POSITION_NED
        case 33:  return 104;  // GLOBAL_POSITION_INT
        case 34:  return 237;  // RC_CHANNELS_SCALED
        case 35:  return 244;  // RC_CHANNELS_RAW
        case 36:  return 222;  // SERVO_OUTPUT_RAW
        case 37:  return 212;  // MISSION_REQUEST_PARTIAL_LIST
        case 38:  return 9;    // MISSION_WRITE_PARTIAL_LIST
        case 39:  return 254;  // MISSION_ITEM
        case 40:  return 230;  // MISSION_REQUEST
        case 41:  return 28;   // MISSION_SET_CURRENT
        case 42:  return 28;   // MISSION_CURRENT
        case 43:  return 132;  // MISSION_REQUEST_LIST
        case 44:  return 221;  // MISSION_COUNT
        case 45:  return 232;  // MISSION_CLEAR_ALL
        case 46:  return 11;   // MISSION_ITEM_REACHED
        case 47:  return 153;  // MISSION_ACK
        case 48:  return 41;   // SET_GPS_GLOBAL_ORIGIN
        case 49:  return 39;   // GPS_GLOBAL_ORIGIN
        case 50:  return 78;   // PARAM_MAP_RC
        case 51:  return 196;  // MISSION_REQUEST_INT
        case 54:  return 15;   // SAFETY_SET_ALLOWED_AREA
        case 55:  return 3;    // SAFETY_ALLOWED_AREA
        case 61:  return 167;  // ATTITUDE_QUATERNION_COV
        case 62:  return 183;  // NAV_CONTROLLER_OUTPUT
        case 63:  return 119;  // GLOBAL_POSITION_INT_COV
        case 64:  return 191;  // LOCAL_POSITION_NED_COV
        case 65:  return 118;  // RC_CHANNELS
        case 66:  return 148;  // REQUEST_DATA_STREAM
        case 67:  return 21;   // DATA_STREAM
        case 69:  return 243;  // MANUAL_CONTROL
        case 70:  return 124;  // RC_CHANNELS_OVERRIDE
        case 73:  return 38;   // MISSION_ITEM_INT
        case 74:  return 20;   // VFR_HUD
        case 75:  return 158;  // COMMAND_INT
        case 76:  return 152;  // COMMAND_LONG
        case 77:  return 143;  // COMMAND_ACK
        case 81:  return 106;  // MANUAL_SETPOINT
        case 82:  return 49;   // SET_ATTITUDE_TARGET
        case 83:  return 22;   // ATTITUDE_TARGET
        case 84:  return 143;  // SET_POSITION_TARGET_LOCAL_NED
        case 85:  return 140;  // POSITION_TARGET_LOCAL_NED
        case 86:  return 5;    // SET_POSITION_TARGET_GLOBAL_INT
        case 87:  return 150;  // POSITION_TARGET_GLOBAL_INT
        case 89:  return 231;  // LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
        case 90:  return 183;  // HIL_STATE
        case 91:  return 63;   // HIL_CONTROLS
        case 92:  return 54;   // HIL_RC_INPUTS_RAW
        case 93:  return 47;   // HIL_ACTUATOR_CONTROLS
        case 100: return 175;  // OPTICAL_FLOW
        case 101: return 102;  // GLOBAL_VISION_POSITION_ESTIMATE
        case 102: return 158;  // VISION_POSITION_ESTIMATE
        case 103: return 208;  // VISION_SPEED_ESTIMATE
        case 104: return 56;   // VICON_POSITION_ESTIMATE
        case 105: return 93;   // HIGHRES_IMU
        case 106: return 138;  // OPTICAL_FLOW_RAD
        case 107: return 108;  // HIL_SENSOR
        case 108: return 32;   // SIM_STATE
        case 109: return 185;  // RADIO_STATUS
        case 110: return 84;   // FILE_TRANSFER_PROTOCOL
        case 111: return 34;   // TIMESYNC
        case 112: return 174;  // CAMERA_TRIGGER
        case 113: return 124;  // HIL_GPS
        case 114: return 237;  // HIL_OPTICAL_FLOW
        case 115: return 4;    // HIL_STATE_QUATERNION
        case 116: return 76;   // SCALED_IMU2
        case 117: return 128;  // LOG_REQUEST_LIST
        case 118: return 56;   // LOG_ENTRY
        case 119: return 116;  // LOG_REQUEST_DATA
        case 120: return 134;  // LOG_DATA
        case 121: return 237;  // LOG_ERASE
        case 122: return 203;  // LOG_REQUEST_END
        case 123: return 250;  // GPS_INJECT_DATA
        case 124: return 87;   // GPS2_RAW
        case 125: return 203;  // POWER_STATUS
        case 126: return 220;  // SERIAL_CONTROL
        case 127: return 25;   // GPS_RTK
        case 128: return 226;  // GPS2_RTK
        case 129: return 46;   // SCALED_IMU3
        case 130: return 29;   // DATA_TRANSMISSION_HANDSHAKE
        case 131: return 223;  // ENCAPSULATED_DATA
        case 132: return 85;   // DISTANCE_SENSOR
        case 133: return 6;    // TERRAIN_REQUEST
        case 134: return 229;  // TERRAIN_DATA
        case 135: return 203;  // TERRAIN_CHECK
        case 136: return 1;    // TERRAIN_REPORT
        case 137: return 195;  // SCALED_PRESSURE2
        case 138: return 109;  // ATT_POS_MOCAP
        case 139: return 168;  // SET_ACTUATOR_CONTROL_TARGET
        case 140: return 181;  // ACTUATOR_CONTROL_TARGET
        case 141: return 47;   // ALTITUDE
        case 142: return 72;   // RESOURCE_REQUEST
        case 143: return 131;  // SCALED_PRESSURE3
        case 144: return 127;  // FOLLOW_TARGET
        case 146: return 103;  // CONTROL_SYSTEM_STATE
        case 147: return 154;  // BATTERY_STATUS
        case 148: return 178;  // AUTOPILOT_VERSION
        case 149: return 200;  // LANDING_TARGET
        case 152: return 208;  // MEMINFO (ArduPilot)
        case 162: return 189;  // FENCE_STATUS
        case 163: return 127;  // AHRS (ArduPilot)
        case 164: return 154;  // SIMSTATE
        case 165: return 21;   // HWSTATUS
        case 166: return 134;  // RADIO
        case 167: return 169;  // LIMITS_STATUS
        case 168: return 185;  // WIND
        case 169: return 139;  // DATA16
        case 170: return 108;  // DATA32
        case 171: return 49;   // DATA64
        case 172: return 182;  // DATA96
        case 173: return 229;  // RANGEFINDER
        case 174: return 85;   // AIRSPEED_AUTOCAL
        case 175: return 159;  // RALLY_POINT
        case 176: return 186;  // RALLY_FETCH_POINT
        case 177: return 65;   // COMPASSMOT_STATUS
        case 178: return 115;  // AHRS2 (ArduPilot)
        case 179: return 134;  // CAMERA_STATUS
        case 180: return 99;   // CAMERA_FEEDBACK
        case 181: return 90;   // BATTERY2
        case 182: return 149;  // AHRS3
        case 183: return 228;  // AUTOPILOT_VERSION_REQUEST
        case 184: return 15;   // REMOTE_LOG_DATA_BLOCK
        case 185: return 245;  // REMOTE_LOG_BLOCK_STATUS
        case 186: return 100;  // LED_CONTROL
        case 191: return 92;   // MAG_CAL_PROGRESS
        case 192: return 36;   // MAG_CAL_REPORT
        case 193: return 71;   // EKF_STATUS_REPORT (ArduPilot)
        case 194: return 134;  // PID_TUNING
        case 200: return 134;  // GIMBAL_REPORT
        case 201: return 205;  // GIMBAL_CONTROL
        case 214: return 69;   // GIMBAL_TORQUE_CMD_REPORT
        case 215: return 101;  // GOPRO_HEARTBEAT
        case 216: return 50;   // GOPRO_GET_REQUEST
        case 217: return 202;  // GOPRO_GET_RESPONSE
        case 218: return 17;   // GOPRO_SET_REQUEST
        case 219: return 162;  // GOPRO_SET_RESPONSE
        case 226: return 207;  // RPM
        case 230: return 163;  // ESTIMATOR_STATUS
        case 231: return 105;  // WIND_COV
        case 232: return 151;  // GPS_INPUT
        case 233: return 35;   // GPS_RTCM_DATA
        case 234: return 150;  // HIGH_LATENCY
        case 241: return 90;   // VIBRATION (ArduPilot)
        case 242: return 104;  // HOME_POSITION
        case 243: return 85;   // SET_HOME_POSITION
        case 244: return 95;   // MESSAGE_INTERVAL
        case 245: return 130;  // EXTENDED_SYS_STATE
        case 246: return 184;  // ADSB_VEHICLE
        case 247: return 81;   // COLLISION
        case 248: return 8;    // V2_EXTENSION
        case 249: return 204;  // MEMORY_VECT
        case 250: return 49;   // DEBUG_VECT
        case 251: return 170;  // NAMED_VALUE_FLOAT
        case 252: return 44;   // NAMED_VALUE_INT
        case 253: return 83;   // STATUSTEXT
        case 254: return 46;   // DEBUG
        case 256: return 71;   // SETUP_SIGNING
        case 257: return 131;  // BUTTON_CHANGE
        case 258: return 187;  // PLAY_TUNE
        case 259: return 92;   // CAMERA_INFORMATION
        case 260: return 146;  // CAMERA_SETTINGS
        case 261: return 179;  // STORAGE_INFORMATION
        case 262: return 12;   // CAMERA_CAPTURE_STATUS
        case 263: return 133;  // CAMERA_IMAGE_CAPTURED
        case 264: return 49;   // FLIGHT_INFORMATION
        case 265: return 26;   // MOUNT_ORIENTATION
        case 266: return 193;  // LOGGING_DATA
        case 267: return 35;   // LOGGING_DATA_ACKED
        case 268: return 14;   // LOGGING_ACK
        case 299: return 19;   // WIFI_CONFIG_AP
        case 300: return 217;  // PROTOCOL_VERSION
        case 310: return 28;   // UAVCAN_NODE_STATUS
        case 311: return 95;   // UAVCAN_NODE_INFO
        case 320: return 243;  // PARAM_EXT_REQUEST_READ
        case 321: return 88;   // PARAM_EXT_REQUEST_LIST
        case 322: return 243;  // PARAM_EXT_VALUE
        case 323: return 78;   // PARAM_EXT_SET
        case 324: return 132;  // PARAM_EXT_ACK
        case 330: return 23;   // OBSTACLE_DISTANCE
        case 331: return 91;   // ODOMETRY
        case 335: return 225;  // ISBD_LINK_STATUS
        case 339: return 199;  // RAW_RPM
        case 340: return 99;   // UTM_GLOBAL_POSITION
        case 350: return 232;  // DEBUG_FLOAT_ARRAY
        case 360: return 11;   // ORBIT_EXECUTION_STATUS
        case 370: return 98;   // SMART_BATTERY_INFO
        case 373: return 117;  // GENERATOR_STATUS
        case 375: return 251;  // ACTUATOR_OUTPUT_STATUS
        case 385: return 147;  // TUNNEL
        case 390: return 156;  // CAN_FRAME
        case 9000: return 113; // WHEEL_DISTANCE
        default:  return 0;    // Unknown message
    }
}

// ============================================================================
// Heartbeat Message Functions
// ============================================================================

/**
 * @brief Decode heartbeat message from a MAVLink message
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t *msg, mavlink_heartbeat_t *heartbeat) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    heartbeat->custom_mode = payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24);
    heartbeat->type = payload[4];
    heartbeat->autopilot = payload[5];
    heartbeat->base_mode = payload[6];
    heartbeat->system_status = payload[7];
    heartbeat->mavlink_version = payload[8];
}

/**
 * @brief Pack a heartbeat message into buffer
 * @return Total packet length
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id,
                                                   uint8_t *buf, uint8_t type, uint8_t autopilot,
                                                   uint8_t base_mode, uint32_t custom_mode,
                                                   uint8_t system_status) {
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;   // Magic
    buf[1] = 9;                // Payload length
    buf[2] = 0;                // Incompat flags
    buf[3] = 0;                // Compat flags
    buf[4] = 0;                // Seq (will be filled by caller if needed)
    buf[5] = system_id;        // System ID
    buf[6] = component_id;     // Component ID
    buf[7] = 0;                // Message ID low byte (HEARTBEAT = 0)
    buf[8] = 0;                // Message ID mid byte
    buf[9] = 0;                // Message ID high byte
    
    // Payload
    buf[10] = custom_mode & 0xFF;
    buf[11] = (custom_mode >> 8) & 0xFF;
    buf[12] = (custom_mode >> 16) & 0xFF;
    buf[13] = (custom_mode >> 24) & 0xFF;
    buf[14] = type;
    buf[15] = autopilot;
    buf[16] = base_mode;
    buf[17] = system_status;
    buf[18] = 2;  // MAVLink version 2
    
    // Calculate CRC
    crc_init(&crc);
    for (int i = 1; i <= 18; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_HEARTBEAT_CRC, &crc);  // CRC extra
    
    buf[19] = crc & 0xFF;
    buf[20] = (crc >> 8) & 0xFF;
    
    return 21;  // Total packet length
}

// ============================================================================
// Attitude Message Functions
// ============================================================================

/**
 * @brief Helper to decode a float from payload bytes (little-endian)
 */
static inline float mavlink_decode_float(const uint8_t *payload, uint8_t offset) {
    union {
        uint32_t i;
        float f;
    } u;
    u.i = payload[offset] | (payload[offset+1] << 8) | 
          (payload[offset+2] << 16) | (payload[offset+3] << 24);
    return u.f;
}

/**
 * @brief Decode attitude message from a MAVLink message
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t *msg, mavlink_attitude_t *attitude) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    
    attitude->time_boot_ms = payload[0] | (payload[1] << 8) | 
                             (payload[2] << 16) | (payload[3] << 24);
    attitude->roll = mavlink_decode_float(payload, 4);
    attitude->pitch = mavlink_decode_float(payload, 8);
    attitude->yaw = mavlink_decode_float(payload, 12);
    attitude->rollspeed = mavlink_decode_float(payload, 16);
    attitude->pitchspeed = mavlink_decode_float(payload, 20);
    attitude->yawspeed = mavlink_decode_float(payload, 24);
}

/**
 * @brief Convert radians to degrees
 */
static inline float mavlink_rad_to_deg(float rad) {
    return rad * 57.295779513f;  // 180/PI
}

// ============================================================================
// COMMAND_LONG Message Functions
// ============================================================================

/**
 * @brief Helper to encode a float into payload bytes (little-endian)
 */
static inline void mavlink_encode_float(uint8_t *payload, uint8_t offset, float value) {
    union {
        uint32_t i;
        float f;
    } u;
    u.f = value;
    payload[offset] = u.i & 0xFF;
    payload[offset+1] = (u.i >> 8) & 0xFF;
    payload[offset+2] = (u.i >> 16) & 0xFF;
    payload[offset+3] = (u.i >> 24) & 0xFF;
}

/**
 * @brief Pack a COMMAND_LONG message into buffer
 * @return Total packet length
 */
static inline uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id,
                                                     uint8_t *buf, uint8_t target_system,
                                                     uint8_t target_component, uint16_t command,
                                                     uint8_t confirmation, float param1,
                                                     float param2, float param3, float param4,
                                                     float param5, float param6, float param7) {
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;   // Magic
    buf[1] = 33;               // Payload length
    buf[2] = 0;                // Incompat flags
    buf[3] = 0;                // Compat flags
    buf[4] = 0;                // Seq (will be filled by caller)
    buf[5] = system_id;        // System ID
    buf[6] = component_id;     // Component ID
    buf[7] = 76;               // Message ID low byte (COMMAND_LONG = 76)
    buf[8] = 0;                // Message ID mid byte
    buf[9] = 0;                // Message ID high byte
    
    // Payload (wire order: floats first, then command, target_system, target_component, confirmation)
    mavlink_encode_float(buf, 10, param1);
    mavlink_encode_float(buf, 14, param2);
    mavlink_encode_float(buf, 18, param3);
    mavlink_encode_float(buf, 22, param4);
    mavlink_encode_float(buf, 26, param5);
    mavlink_encode_float(buf, 30, param6);
    mavlink_encode_float(buf, 34, param7);
    buf[38] = command & 0xFF;
    buf[39] = (command >> 8) & 0xFF;
    buf[40] = target_system;
    buf[41] = target_component;
    buf[42] = confirmation;
    
    // Calculate CRC
    crc_init(&crc);
    for (int i = 1; i <= 42; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_COMMAND_LONG_CRC, &crc);
    
    buf[43] = crc & 0xFF;
    buf[44] = (crc >> 8) & 0xFF;
    
    return 45;  // Total packet length
}

// ============================================================================
// COMMAND_ACK Message Functions
// ============================================================================

/**
 * @brief Decode COMMAND_ACK message from a MAVLink message
 */
static inline void mavlink_msg_command_ack_decode(const mavlink_message_t *msg, mavlink_command_ack_t *ack) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    ack->command = payload[0] | (payload[1] << 8);
    ack->result = payload[2];
}

/**
 * @brief Get result string for COMMAND_ACK
 */
static inline const char* mavlink_result_to_string(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
        case MAV_RESULT_DENIED: return "DENIED";
        case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case MAV_RESULT_FAILED: return "FAILED";
        case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// SET_MODE Message Functions
// ============================================================================

/**
 * @brief Pack a SET_MODE message into buffer
 * @return Total packet length
 */
static inline uint16_t mavlink_msg_set_mode_pack(uint8_t system_id, uint8_t component_id,
                                                  uint8_t *buf, uint8_t target_system,
                                                  uint8_t base_mode, uint32_t custom_mode) {
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;   // Magic
    buf[1] = 6;                // Payload length
    buf[2] = 0;                // Incompat flags
    buf[3] = 0;                // Compat flags
    buf[4] = 0;                // Seq
    buf[5] = system_id;        // System ID
    buf[6] = component_id;     // Component ID
    buf[7] = 11;               // Message ID low byte (SET_MODE = 11)
    buf[8] = 0;                // Message ID mid byte
    buf[9] = 0;                // Message ID high byte
    
    // Payload (wire order: custom_mode first, then base_mode, target_system)
    buf[10] = custom_mode & 0xFF;
    buf[11] = (custom_mode >> 8) & 0xFF;
    buf[12] = (custom_mode >> 16) & 0xFF;
    buf[13] = (custom_mode >> 24) & 0xFF;
    buf[14] = base_mode;
    buf[15] = target_system;
    
    // Calculate CRC
    crc_init(&crc);
    for (int i = 1; i <= 15; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_SET_MODE_CRC, &crc);
    
    buf[16] = crc & 0xFF;
    buf[17] = (crc >> 8) & 0xFF;
    
    return 18;  // Total packet length
}

// ============================================================================
// STATUSTEXT Message Functions
// ============================================================================

/**
 * @brief Decode STATUSTEXT message from a MAVLink message
 */
static inline void mavlink_msg_statustext_decode(const mavlink_message_t *msg, mavlink_statustext_t *statustext) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    statustext->severity = payload[0];
    for (int i = 0; i < 50; i++) {
        statustext->text[i] = payload[1 + i];
    }
}

/**
 * @brief Get severity string for STATUSTEXT
 */
static inline const char* mavlink_severity_to_string(uint8_t severity) {
    switch (severity) {
        case 0: return "EMERGENCY";
        case 1: return "ALERT";
        case 2: return "CRITICAL";
        case 3: return "ERROR";
        case 4: return "WARNING";
        case 5: return "NOTICE";
        case 6: return "INFO";
        case 7: return "DEBUG";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// PARAM_SET Message Functions
// ============================================================================

/**
 * @brief Pack a PARAM_SET message into buffer
 * Wire order: param_value(float), target_system(u8), target_component(u8),
 *             param_id(char[16]), param_type(u8)
 * @return Total packet length
 */
static inline uint16_t mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id,
                                                   uint8_t *buf, uint8_t target_system,
                                                   uint8_t target_component,
                                                   const char *param_id, float param_value,
                                                   uint8_t param_type) {
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;
    buf[1] = 23;               // Payload length
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;                // Seq
    buf[5] = system_id;
    buf[6] = component_id;
    buf[7] = 23;               // Message ID low (PARAM_SET = 23)
    buf[8] = 0;
    buf[9] = 0;
    
    // Payload
    mavlink_encode_float(buf, 10, param_value);  // param_value at 10-13
    buf[14] = target_system;                      // target_system at 14
    buf[15] = target_component;                   // target_component at 15
    memset(&buf[16], 0, 16);                      // param_id at 16-31
    strncpy((char *)&buf[16], param_id, 16);
    buf[32] = param_type;                         // param_type at 32
    
    // CRC over bytes 1 through 32
    crc_init(&crc);
    for (int i = 1; i <= 32; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_PARAM_SET_CRC, &crc);
    
    buf[33] = crc & 0xFF;
    buf[34] = (crc >> 8) & 0xFF;
    
    return 35;
}

// ============================================================================
// PARAM_VALUE Message Functions
// ============================================================================

/**
 * @brief Decode PARAM_VALUE message from a MAVLink message
 * Wire order: param_value(float), param_count(u16), param_index(u16),
 *             param_id(char[16]), param_type(u8)
 */
static inline void mavlink_msg_param_value_decode(const mavlink_message_t *msg,
                                                   mavlink_param_value_t *param) {
    const uint8_t *payload = (const uint8_t *)msg->payload64;
    param->param_value = mavlink_decode_float(payload, 0);
    param->param_count = payload[4] | (payload[5] << 8);
    param->param_index = payload[6] | (payload[7] << 8);
    memcpy(param->param_id, &payload[8], 16);
    param->param_id[15] = '\0';
    param->param_type = payload[24];
}

// ============================================================================
// RC_CHANNELS_OVERRIDE Message Functions
// ============================================================================

/**
 * @brief Pack RC_CHANNELS_OVERRIDE message into buffer
 * Wire order: chan1-chan8(u16 each), target_system(u8), target_component(u8)
 * @return Total packet length
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack(
    uint8_t system_id, uint8_t component_id, uint8_t *buf,
    uint8_t target_system, uint8_t target_component,
    uint16_t chan1, uint16_t chan2, uint16_t chan3, uint16_t chan4,
    uint16_t chan5, uint16_t chan6, uint16_t chan7, uint16_t chan8) {
    
    uint16_t crc;
    
    buf[0] = MAVLINK_STX_V2;
    buf[1] = 18;               // Payload length
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;                // Seq
    buf[5] = system_id;
    buf[6] = component_id;
    buf[7] = 70;               // Message ID low (RC_CHANNELS_OVERRIDE = 70)
    buf[8] = 0;
    buf[9] = 0;
    
    // Payload: uint16 channels first, then uint8 targets
    buf[10] = chan1 & 0xFF; buf[11] = (chan1 >> 8) & 0xFF;
    buf[12] = chan2 & 0xFF; buf[13] = (chan2 >> 8) & 0xFF;
    buf[14] = chan3 & 0xFF; buf[15] = (chan3 >> 8) & 0xFF;
    buf[16] = chan4 & 0xFF; buf[17] = (chan4 >> 8) & 0xFF;
    buf[18] = chan5 & 0xFF; buf[19] = (chan5 >> 8) & 0xFF;
    buf[20] = chan6 & 0xFF; buf[21] = (chan6 >> 8) & 0xFF;
    buf[22] = chan7 & 0xFF; buf[23] = (chan7 >> 8) & 0xFF;
    buf[24] = chan8 & 0xFF; buf[25] = (chan8 >> 8) & 0xFF;
    buf[26] = target_system;
    buf[27] = target_component;
    
    // CRC over bytes 1 through 27
    crc_init(&crc);
    for (int i = 1; i <= 27; i++) {
        crc_accumulate(buf[i], &crc);
    }
    crc_accumulate(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC, &crc);
    
    buf[28] = crc & 0xFF;
    buf[29] = (crc >> 8) & 0xFF;
    
    return 30;
}

// ============================================================================
// SYS_STATUS Message Functions
// ============================================================================

/**
 * @brief Decode SYS_STATUS message (MSG ID 1)
 * Wire order: sensors_present(u32), sensors_enabled(u32), sensors_health(u32),
 *             load(u16), voltage_battery(u16), current_battery(i16), ...
 *             battery_remaining(i8) at byte 18
 */
static inline void mavlink_msg_sys_status_decode(const mavlink_message_t *msg,
                                                  mavlink_sys_status_t *status) {
    const uint8_t *p = (const uint8_t *)msg->payload64;
    // voltage_battery at offset 14-15 (after 3x uint32 + 1x uint16)
    status->voltage_battery = p[14] | (p[15] << 8);
    // current_battery at offset 16-17
    status->current_battery = (int16_t)(p[16] | (p[17] << 8));
    // battery_remaining at offset 30 (after drop_rate_comm, errors_comm, errors_count1-4)
    status->battery_remaining = (int8_t)p[30];
}

// ============================================================================
// GPS_RAW_INT Message Functions
// ============================================================================

/**
 * @brief Helper to decode int32 from payload bytes (little-endian)
 */
static inline int32_t mavlink_decode_int32(const uint8_t *payload, uint8_t offset) {
    return (int32_t)(payload[offset] | (payload[offset+1] << 8) |
                     (payload[offset+2] << 16) | (payload[offset+3] << 24));
}

/**
 * @brief Helper to decode uint16 from payload bytes (little-endian)
 */
static inline uint16_t mavlink_decode_uint16(const uint8_t *payload, uint8_t offset) {
    return (uint16_t)(payload[offset] | (payload[offset+1] << 8));
}

/**
 * @brief Helper to decode int16 from payload bytes (little-endian)
 */
static inline int16_t mavlink_decode_int16(const uint8_t *payload, uint8_t offset) {
    return (int16_t)(payload[offset] | (payload[offset+1] << 8));
}

/**
 * @brief Decode GPS_RAW_INT message (MSG ID 24)
 * Wire order: time_usec(u64), lat(i32), lon(i32), alt(i32), eph(u16), epv(u16),
 *             vel(u16), cog(u16), fix_type(u8), satellites_visible(u8)
 */
static inline void mavlink_msg_gps_raw_int_decode(const mavlink_message_t *msg,
                                                    mavlink_gps_raw_int_t *gps) {
    const uint8_t *p = (const uint8_t *)msg->payload64;
    gps->time_usec = (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) |
                     ((uint64_t)p[3] << 24) | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
                     ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
    gps->lat = mavlink_decode_int32(p, 8);
    gps->lon = mavlink_decode_int32(p, 12);
    gps->alt = mavlink_decode_int32(p, 16);
    gps->eph = mavlink_decode_uint16(p, 20);
    gps->epv = mavlink_decode_uint16(p, 22);
    gps->vel = mavlink_decode_uint16(p, 24);
    gps->cog = mavlink_decode_uint16(p, 26);
    gps->fix_type = p[28];
    gps->satellites_visible = p[29];
}

/**
 * @brief Get GPS fix type string
 */
static inline const char* mavlink_gps_fix_type_string(uint8_t fix_type) {
    switch (fix_type) {
        case 0: return "No GPS";
        case 1: return "No Fix";
        case 2: return "2D Fix";
        case 3: return "3D Fix";
        case 4: return "DGPS";
        case 5: return "RTK Float";
        case 6: return "RTK Fixed";
        default: return "Unknown";
    }
}

// ============================================================================
// SCALED_PRESSURE Message Functions
// ============================================================================

/**
 * @brief Decode SCALED_PRESSURE message (MSG ID 29)
 * Wire order: time_boot_ms(u32), press_abs(float), press_diff(float), temperature(i16)
 */
static inline void mavlink_msg_scaled_pressure_decode(const mavlink_message_t *msg,
                                                       mavlink_scaled_pressure_t *press) {
    const uint8_t *p = (const uint8_t *)msg->payload64;
    press->time_boot_ms = (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    press->press_abs = mavlink_decode_float(p, 4);
    press->press_diff = mavlink_decode_float(p, 8);
    press->temperature = mavlink_decode_int16(p, 12);
}

// ============================================================================
// VFR_HUD Message Functions
// ============================================================================

/**
 * @brief Decode VFR_HUD message (MSG ID 74)
 * Wire order: airspeed(float), groundspeed(float), alt(float), climb(float),
 *             heading(i16), throttle(u16)
 */
static inline void mavlink_msg_vfr_hud_decode(const mavlink_message_t *msg,
                                                mavlink_vfr_hud_t *hud) {
    const uint8_t *p = (const uint8_t *)msg->payload64;
    hud->airspeed = mavlink_decode_float(p, 0);
    hud->groundspeed = mavlink_decode_float(p, 4);
    hud->alt = mavlink_decode_float(p, 8);
    hud->climb = mavlink_decode_float(p, 12);
    hud->heading = mavlink_decode_int16(p, 16);
    hud->throttle = mavlink_decode_uint16(p, 18);
}

// ============================================================================
// GLOBAL_POSITION_INT Message Functions
// ============================================================================

/**
 * @brief Decode GLOBAL_POSITION_INT message (MSG ID 33)
 * Wire order: time_boot_ms(u32), lat(i32), lon(i32), alt(i32), relative_alt(i32),
 *             vx(i16), vy(i16), vz(i16), hdg(u16)
 */
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t *msg,
                                                           mavlink_global_position_int_t *pos) {
    const uint8_t *p = (const uint8_t *)msg->payload64;
    pos->time_boot_ms = (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    pos->lat = mavlink_decode_int32(p, 4);
    pos->lon = mavlink_decode_int32(p, 8);
    pos->alt = mavlink_decode_int32(p, 12);
    pos->relative_alt = mavlink_decode_int32(p, 16);
    pos->vx = mavlink_decode_int16(p, 20);
    pos->vy = mavlink_decode_int16(p, 22);
    pos->vz = mavlink_decode_int16(p, 24);
    pos->hdg = mavlink_decode_uint16(p, 26);
}

#endif // MAVESPSTM_MAVLINK_MESSAGES_H
