/**
 * @file web-interface.c
 * @brief ESP32 MAVLink Companion Computer - Full Web Control Interface
 * 
 * Expanded web control interface for Pixhawk:
 * - ESP32 in AP mode (creates WiFi network)
 * - Web interface with live telemetry display
 * - Attitude data (roll, pitch, yaw) with visual horizon
 * - Arm/Disarm/Force Arm controls
 * - Flight mode selection using SET_MODE message
 * - RC channel override: Throttle, Yaw, Pitch, Roll sliders
 * - GPS data display (lat, lon, alt, fix, satellites, HDOP)
 * - Barometer data (pressure, temperature, altitude)
 * - VFR HUD data (groundspeed, heading, climb rate, throttle)
 * - Setup: disable pre-arm checks via PARAM_SET
 * - Status logs display with verbose diagnostics
 * 
 * Key fixes applied:
 * - COMPANION_SYSTEM_ID = 200 (avoids Pixhawk sysid=1 conflict)
 * - Heartbeat as MAV_TYPE_GCS(6) for full command authority
 * - CRC recomputed AFTER setting sequence number
 * - target_component=0 (broadcast) per ArduPilot wiki
 * 
 * Configuration:
 * - WiFi AP SSID: Pixhawk-ESP32
 * - WiFi AP Password: pixhawk123
 * - Web interface: http://192.168.4.1
 * - MAVLink: TX GPIO 16, RX GPIO 17, 57600 baud
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "mavespstm.h"

// ============================================================================
// Configuration
// ============================================================================

#define WIFI_AP_SSID        "Pixhawk-ESP32"
#define WIFI_AP_PASS        "pixhawk123"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4

#define PIXHAWK_UART_NUM     UART_NUM_2
#define PIXHAWK_TX_PIN       GPIO_NUM_16
#define PIXHAWK_RX_PIN       GPIO_NUM_17
#define PIXHAWK_BAUD_RATE    57600
#define UART_BUF_SIZE        1024

// MAVLink Configuration
#define COMPANION_SYSTEM_ID    200
#define COMPANION_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER
#define PIXHAWK_SYSTEM_ID      1
#define PIXHAWK_COMPONENT_ID   1

// Log buffer
#define LOG_BUFFER_SIZE        24
#define LOG_MSG_SIZE           120

static const char *TAG = "WebIface";

// ============================================================================
// Global State
// ============================================================================

// MAVLink state
static mavlink_status_t mav_status;
static mavlink_message_t mav_msg;
static uint8_t tx_seq = 0;
static SemaphoreHandle_t uart_mutex;

// Attitude data
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;
static uint32_t attitude_count = 0;

// Flight state
static uint32_t heartbeat_count = 0;
static uint8_t pixhawk_base_mode = 0;
static uint32_t pixhawk_custom_mode = 0;
static uint8_t pixhawk_system_status = 0;
static bool is_armed = false;
static bool is_connected = false;
static uint32_t last_heartbeat_time = 0;

// GPS data
static int32_t gps_lat = 0;           // degE7
static int32_t gps_lon = 0;           // degE7
static int32_t gps_alt = 0;           // mm MSL
static uint8_t gps_fix_type = 0;
static uint8_t gps_satellites = 0;
static uint16_t gps_eph = 9999;       // HDOP * 100
static uint16_t gps_vel = 0;          // cm/s
static bool gps_has_data = false;

// Global position (fused)
static int32_t global_lat = 0;
static int32_t global_lon = 0;
static int32_t global_alt = 0;        // mm MSL
static int32_t global_rel_alt = 0;    // mm above home
static uint16_t global_hdg = 0;       // cdeg

// Barometer data
static float baro_press_abs = 0.0f;   // hPa
static float baro_press_diff = 0.0f;  // hPa
static int16_t baro_temperature = 0;  // cdegC
static bool baro_has_data = false;

// Battery data (from SYS_STATUS msg)
static uint16_t batt_voltage = 0;      // mV
static int16_t batt_current = -1;      // cA (10*mA), -1=unknown
static int8_t batt_remaining = -1;     // %, -1=unknown
static bool batt_has_data = false;

// VFR HUD data
static float vfr_airspeed = 0.0f;     // m/s
static float vfr_groundspeed = 0.0f;  // m/s
static float vfr_alt = 0.0f;          // m MSL
static float vfr_climb = 0.0f;        // m/s
static int16_t vfr_heading = 0;       // deg
static uint16_t vfr_throttle = 0;     // %
static bool vfr_has_data = false;

// RC Override state
static uint16_t rc_chan1 = 0;  // Roll (0=no override)
static uint16_t rc_chan2 = 0;  // Pitch
static uint16_t rc_chan3 = 0;  // Throttle
static uint16_t rc_chan4 = 0;  // Yaw
static bool rc_override_active = false;
static uint32_t rc_last_web_time = 0;  // Last time browser polled /api/data

// Safety: auto-release RC override if browser stops polling (WiFi lost, tab closed, etc.)
#define RC_OVERRIDE_TIMEOUT_MS 2000

// ============================================================================
// Auto-flight sequence state
// ============================================================================
typedef enum {
    SEQ_IDLE = 0,
    SEQ_PREFLIGHT,     // Setting fence, checking GPS
    SEQ_ARMING,        // Switching to GUIDED + arming
    SEQ_TAKEOFF,       // MAV_CMD_NAV_TAKEOFF sent, climbing
    SEQ_HOVERING,      // At target altitude, counting down
    SEQ_LANDING,       // LAND mode set, descending
    SEQ_COMPLETE,      // Done
    SEQ_ABORTED        // User aborted or safety triggered
} auto_seq_state_t;

static auto_seq_state_t seq_state = SEQ_IDLE;
static uint32_t seq_timer = 0;          // Timestamp for current phase
static uint32_t seq_phase_start = 0;    // Start time for arming/takeoff phase
static float seq_target_alt = 5.0f;     // Target altitude in meters
static int seq_hover_seconds = 10;      // Hover duration
static int seq_hover_remaining = 0;     // Countdown
static bool seq_abort_flag = false;     // Set by abort button
static uint32_t seq_last_tkoff_send = 0; // Last takeoff cmd send time

#define SEQ_TAKEOFF_TIMEOUT_MS  30000   // Max time to reach altitude
#define SEQ_ALT_TOLERANCE       1.0f    // Within 1m of target = arrived
#define SEQ_ARM_TIMEOUT_MS      15000   // Max time to arm
#define FENCE_ALT_MAX           10.0f   // Geofence max altitude
#define FENCE_RADIUS            30.0f   // Geofence max radius from home
#define MAV_CMD_NAV_TAKEOFF     22

// Log buffer (circular)
static char log_buffer[LOG_BUFFER_SIZE][LOG_MSG_SIZE];
static int log_head = 0;
static int log_count = 0;
static SemaphoreHandle_t log_mutex;

// ============================================================================
// ArduPilot Flight Modes (ArduCopter)
// ============================================================================

typedef struct {
    uint32_t mode_num;
    const char *name;
} flight_mode_t;

static const flight_mode_t copter_modes[] = {
    {0,  "STABILIZE"},
    {1,  "ACRO"},
    {2,  "ALT_HOLD"},
    {3,  "AUTO"},
    {4,  "GUIDED"},
    {5,  "LOITER"},
    {6,  "RTL"},
    {7,  "CIRCLE"},
    {9,  "LAND"},
    {16, "POSHOLD"},
    {17, "BRAKE"},
    {21, "SMART_RTL"},
};
#define NUM_COPTER_MODES (sizeof(copter_modes) / sizeof(copter_modes[0]))

static const char* get_mode_name(uint32_t mode) {
    for (int i = 0; i < NUM_COPTER_MODES; i++) {
        if (copter_modes[i].mode_num == mode) {
            return copter_modes[i].name;
        }
    }
    return "UNKNOWN";
}

static const char* get_status_name(uint8_t status) {
    switch (status) {
        case 0: return "UNINIT";
        case 1: return "BOOT";
        case 2: return "CALIBRATING";
        case 3: return "STANDBY";
        case 4: return "ACTIVE";
        case 5: return "CRITICAL";
        case 6: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Logging
// ============================================================================

static void add_log(const char *fmt, ...) {
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(log_buffer[log_head], LOG_MSG_SIZE, fmt, args);
        va_end(args);
        
        log_head = (log_head + 1) % LOG_BUFFER_SIZE;
        if (log_count < LOG_BUFFER_SIZE) log_count++;
        
        xSemaphoreGive(log_mutex);
    }
}

// ============================================================================
// UART / MAVLink Core
// ============================================================================

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = PIXHAWK_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(PIXHAWK_UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PIXHAWK_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PIXHAWK_UART_NUM, PIXHAWK_TX_PIN, PIXHAWK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", PIXHAWK_TX_PIN, PIXHAWK_RX_PIN, PIXHAWK_BAUD_RATE);
}

/**
 * @brief Send MAVLink message with CRC fix
 * Sets sequence number THEN recomputes CRC to avoid the silent-drop bug.
 */
static void send_mavlink_message(uint8_t *buf, uint16_t len) {
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buf[4] = tx_seq++;
        
        // Recompute CRC over header + payload (bytes 1 through 9+payload_len)
        uint8_t payload_len = buf[1];
        uint32_t msgid = buf[7] | ((uint32_t)buf[8] << 8) | ((uint32_t)buf[9] << 16);
        
        uint16_t crc;
        crc_init(&crc);
        for (int i = 1; i < 10 + payload_len; i++) {
            crc_accumulate(buf[i], &crc);
        }
        crc_accumulate(mavlink_get_crc_extra(msgid), &crc);
        
        buf[10 + payload_len] = crc & 0xFF;
        buf[10 + payload_len + 1] = (crc >> 8) & 0xFF;
        
        int written = uart_write_bytes(PIXHAWK_UART_NUM, buf, len);
        if (written != len) {
            ESP_LOGW(TAG, "UART write incomplete: %d/%d bytes", written, len);
        }
        xSemaphoreGive(uart_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire UART mutex");
    }
}

// ============================================================================
// MAVLink Command Functions
// ============================================================================

static void send_heartbeat(void) {
    uint8_t buf[32];
    uint16_t len = mavlink_msg_heartbeat_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_GENERIC,
        0, 0,
        MAV_STATE_ACTIVE
    );
    send_mavlink_message(buf, len);
}

static void send_arm_command(bool arm, bool force) {
    uint8_t buf[48];
    float arm_param = arm ? 1.0f : 0.0f;
    float force_param = force ? (float)MAV_ARM_FORCE_MAGIC : 0.0f;
    
    uint16_t len = mavlink_msg_command_long_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        0,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        arm_param,
        force_param,
        0, 0, 0, 0, 0
    );
    
    send_mavlink_message(buf, len);
    
    const char *cmd = arm ? (force ? "FORCE ARM" : "ARM") : "DISARM";
    add_log("Sent %s (sysid=%d->%d)", cmd, COMPANION_SYSTEM_ID, PIXHAWK_SYSTEM_ID);
    ESP_LOGI(TAG, "Sent %s command", cmd);
}

static void send_mode_command(uint32_t mode) {
    uint8_t buf[32];
    
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (is_armed) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    
    uint16_t len = mavlink_msg_set_mode_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        base_mode,
        mode
    );
    
    send_mavlink_message(buf, len);
    add_log("Set mode: %s (base=0x%02X)", get_mode_name(mode), base_mode);
    ESP_LOGI(TAG, "Sent SET_MODE: %s (%lu)", get_mode_name(mode), mode);
}

static void send_param_set(const char *param_id, float value) {
    uint8_t buf[48];
    
    uint16_t len = mavlink_msg_param_set_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        0,
        param_id,
        value,
        MAV_PARAM_TYPE_REAL32
    );
    
    send_mavlink_message(buf, len);
    add_log("PARAM_SET: %s = %.0f", param_id, value);
    ESP_LOGI(TAG, "Sent PARAM_SET: %s = %.1f", param_id, value);
}

/**
 * @brief Send RC_CHANNELS_OVERRIDE with current slider values
 * Channel mapping: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
 * Value 0 = release channel back to RC radio
 * Value UINT16_MAX (65535) = ignore this channel (leave unchanged)
 */
static void send_rc_override(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
    uint8_t buf[32];
    
    uint16_t len = mavlink_msg_rc_channels_override_pack(
        COMPANION_SYSTEM_ID,
        COMPANION_COMPONENT_ID,
        buf,
        PIXHAWK_SYSTEM_ID,
        0,
        ch1, ch2, ch3, ch4,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX
    );
    
    send_mavlink_message(buf, len);
}

static void send_rc_override_throttle_low(void) {
    send_rc_override(0, 0, 1000, 0);
    ESP_LOGI(TAG, "Sent RC override: throttle low");
}

static void disable_prearm_checks(void) {
    ESP_LOGI(TAG, "Disabling pre-arm checks...");
    add_log("Disabling checks & failsafes...");
    
    send_param_set("ARMING_CHECK", 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    send_param_set("FS_THR_ENABLE", 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    send_param_set("FS_GCS_ENABLE", 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Disable auto-disarm on landing detection (default=10s).
    // On a bench the drone never "takes off" so ArduPilot thinks it's landed
    // and auto-disarms after 10 seconds.
    send_param_set("DISARM_DELAY", 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // CRITICAL: ArduPilot ignores RC_CHANNELS_OVERRIDE unless the sender's
    // system ID matches SYSID_MYGCS. Default is 255 (Mission Planner).
    // Set it to our ESP32's system ID so overrides are accepted.
    send_param_set("SYSID_MYGCS", (float)COMPANION_SYSTEM_ID);
    add_log("Set SYSID_MYGCS=%d (RC override)", COMPANION_SYSTEM_ID);
}

// ============================================================================
// Auto-flight sequence helpers
// ============================================================================

static const char *seq_state_name(auto_seq_state_t s) {
    switch (s) {
        case SEQ_IDLE:       return "IDLE";
        case SEQ_PREFLIGHT:  return "PREFLIGHT";
        case SEQ_ARMING:     return "ARMING";
        case SEQ_TAKEOFF:    return "TAKEOFF";
        case SEQ_HOVERING:   return "HOVERING";
        case SEQ_LANDING:    return "LANDING";
        case SEQ_COMPLETE:   return "COMPLETE";
        case SEQ_ABORTED:    return "ABORTED";
        default:             return "UNKNOWN";
    }
}

static void setup_geofence(void) {
    ESP_LOGI(TAG, "Setting up geofence: alt=%.0fm radius=%.0fm", FENCE_ALT_MAX, FENCE_RADIUS);
    add_log("Geofence: alt=%.0fm rad=%.0fm", FENCE_ALT_MAX, FENCE_RADIUS);

    send_param_set("FENCE_TYPE", 3);          // Altitude + Circle
    vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ALT_MAX", FENCE_ALT_MAX);
    vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_RADIUS", FENCE_RADIUS);
    vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ACTION", 2);        // Always LAND on breach
    vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ENABLE", 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void send_takeoff_command(float altitude) {
    ESP_LOGI(TAG, "Sending MAV_CMD_NAV_TAKEOFF alt=%.1f", altitude);
    add_log("Takeoff cmd: %.1fm", altitude);

    uint8_t buf[64];
    int len = mavlink_msg_command_long_pack(
        COMPANION_SYSTEM_ID, COMPANION_COMPONENT_ID, buf,
        PIXHAWK_SYSTEM_ID, 0,         // target sys, target comp
        MAV_CMD_NAV_TAKEOFF, 0,        // command, confirmation
        0, 0, 0, 0, 0, 0, altitude);  // params 1-7 (alt in param7)
    send_mavlink_message(buf, len);
}

/**
 * @brief State-machine tick for the automated takeoff sequence.
 *        Called at 4 Hz from the heartbeat task.
 *
 *  PREFLIGHT  → check GPS, set geofence, disable prearm
 *  ARMING     → switch GUIDED, arm (retry every 2 s, max 5 tries)
 *  TAKEOFF    → MAV_CMD_NAV_TAKEOFF, monitor rel-alt
 *  HOVERING   → countdown hover_seconds
 *  LANDING    → LAND mode, wait for disarm
 *  COMPLETE / ABORTED
 */
static void auto_sequence_tick(void) {
    if (seq_state == SEQ_IDLE || seq_state == SEQ_COMPLETE || seq_state == SEQ_ABORTED)
        return;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* ---- global abort check ---- */
    if (seq_abort_flag) {
        seq_abort_flag = false;
        ESP_LOGW(TAG, "AUTO-FLIGHT ABORTED by user");
        add_log("!! ABORT: switching to LAND !!");
        send_mode_command(9);   // LAND
        seq_state = SEQ_ABORTED;
        return;
    }

    switch (seq_state) {

    /* ---- PREFLIGHT: GPS + geofence + disable prearm ---- */
    case SEQ_PREFLIGHT: {
        if (gps_fix_type < 3 || gps_satellites < 6) {
            /* log once per 5 s */
            if ((now - seq_timer) % 5000 < 250)
                add_log("Waiting GPS (fix=%d sats=%d)...", gps_fix_type, gps_satellites);
            if ((now - seq_timer) > 30000) {
                add_log("GPS timeout - aborting");
                seq_state = SEQ_ABORTED;
            }
            return;
        }

        add_log("GPS OK (fix=%d sats=%d)", gps_fix_type, gps_satellites);
        disable_prearm_checks();
        setup_geofence();

        seq_state = SEQ_ARMING;
        seq_timer = now;
        seq_phase_start = now;
        add_log("Seq: PREFLIGHT -> ARMING");
        ESP_LOGI(TAG, "Seq: PREFLIGHT -> ARMING");
        break;
    }

    /* ---- ARMING: GUIDED mode + arm ---- */
    case SEQ_ARMING: {
        if (is_armed) {
            seq_state = SEQ_TAKEOFF;
            seq_timer = now;
            seq_last_tkoff_send = 0;
            add_log("ARMED -> sending TAKEOFF");
            ESP_LOGI(TAG, "Seq: ARMED -> TAKEOFF");
            send_takeoff_command(seq_target_alt);
            return;
        }

        /* give up after 15 s from start of arming phase */
        if ((now - seq_phase_start) > SEQ_ARM_TIMEOUT_MS) {
            add_log("Failed to arm after 15s - aborting");
            seq_state = SEQ_ABORTED;
            break;
        }

        /* ensure GUIDED */
        if (pixhawk_custom_mode != 4) {
            send_mode_command(4);
            add_log("Setting GUIDED mode...");
            return;   /* come back next tick */
        }

        /* try arming every 2 s */
        if ((now - seq_timer) > 2000) {
            send_rc_override_throttle_low();
            send_arm_command(true, false);
            add_log("Arm attempt...");
            seq_timer = now;
        }
        break;
    }

    /* ---- TAKEOFF: monitor altitude ---- */
    case SEQ_TAKEOFF: {
        float rel_alt_m = global_rel_alt / 1000.0f;

        /* resend takeoff command every 3 s in case first was missed */
        if (seq_last_tkoff_send == 0) seq_last_tkoff_send = now;
        if ((now - seq_last_tkoff_send) > 3000) {
            send_takeoff_command(seq_target_alt);
            seq_last_tkoff_send = now;
        }

        if (rel_alt_m >= (seq_target_alt - SEQ_ALT_TOLERANCE)) {
            seq_state = SEQ_HOVERING;
            seq_timer = now;
            seq_hover_remaining = seq_hover_seconds;
            add_log("Reached %.1fm - hovering %ds", rel_alt_m, seq_hover_seconds);
            ESP_LOGI(TAG, "Seq: HOVERING for %ds", seq_hover_seconds);
            return;
        }

        if ((now - seq_timer) > SEQ_TAKEOFF_TIMEOUT_MS) {
            add_log("Takeoff timeout (%.1fm) - LAND", rel_alt_m);
            send_mode_command(9);
            seq_state = SEQ_ABORTED;
        }
        break;
    }

    /* ---- HOVERING: count down ---- */
    case SEQ_HOVERING: {
        int elapsed = (int)((now - seq_timer) / 1000);
        seq_hover_remaining = seq_hover_seconds - elapsed;
        if (seq_hover_remaining <= 0) {
            seq_hover_remaining = 0;
            add_log("Hover done - LANDING");
            ESP_LOGI(TAG, "Seq: LANDING");
            send_mode_command(9);
            seq_state = SEQ_LANDING;
            seq_timer = now;
        }
        break;
    }

    /* ---- LANDING: wait for disarm ---- */
    case SEQ_LANDING: {
        if (pixhawk_custom_mode != 9)
            send_mode_command(9);

        if (!is_armed) {
            seq_state = SEQ_COMPLETE;
            add_log("Landed & disarmed - COMPLETE!");
            ESP_LOGI(TAG, "Seq: COMPLETE");
            return;
        }
        /* patience message every 30 s */
        if ((now - seq_timer) > 30000) {
            add_log("Still landing...");
            seq_timer = now;
        }
        break;
    }

    default: break;
    }
}

// ============================================================================
// MAVLink Message Handlers
// ============================================================================

static void handle_heartbeat(const mavlink_message_t *msg) {
    if (msg->sysid == 255) return;
    if (msg->sysid == COMPANION_SYSTEM_ID) return;
    
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);
    
    if (heartbeat_count > 0 && pixhawk_custom_mode != hb.custom_mode) {
        ESP_LOGI(TAG, "MODE CHANGED: %s -> %s", 
                 get_mode_name(pixhawk_custom_mode), get_mode_name(hb.custom_mode));
        add_log("Mode: %s -> %s", get_mode_name(pixhawk_custom_mode), get_mode_name(hb.custom_mode));
    }
    
    bool new_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    if (heartbeat_count > 0 && is_armed != new_armed) {
        ESP_LOGI(TAG, "ARM STATE: %s", new_armed ? "ARMED" : "DISARMED");
        add_log("%s", new_armed ? ">>> ARMED <<<" : ">>> DISARMED <<<");
    }
    
    heartbeat_count++;
    pixhawk_base_mode = hb.base_mode;
    pixhawk_custom_mode = hb.custom_mode;
    pixhawk_system_status = hb.system_status;
    is_armed = new_armed;
    is_connected = true;
    last_heartbeat_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (heartbeat_count % 10 == 1) {
        ESP_LOGI(TAG, "PIXHAWK: Mode=%s Status=%s %s",
                 get_mode_name(pixhawk_custom_mode),
                 get_status_name(pixhawk_system_status),
                 is_armed ? "ARMED" : "DISARMED");
    }
}

static void handle_attitude(const mavlink_message_t *msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);
    
    current_roll = mavlink_rad_to_deg(attitude.roll);
    current_pitch = mavlink_rad_to_deg(attitude.pitch);
    current_yaw = mavlink_rad_to_deg(attitude.yaw);
    attitude_count++;
}

static void handle_gps_raw_int(const mavlink_message_t *msg) {
    mavlink_gps_raw_int_t gps;
    mavlink_msg_gps_raw_int_decode(msg, &gps);
    
    gps_lat = gps.lat;
    gps_lon = gps.lon;
    gps_alt = gps.alt;
    gps_fix_type = gps.fix_type;
    gps_satellites = gps.satellites_visible;
    gps_eph = gps.eph;
    gps_vel = gps.vel;
    gps_has_data = true;
}

static void handle_global_position_int(const mavlink_message_t *msg) {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(msg, &pos);
    
    global_lat = pos.lat;
    global_lon = pos.lon;
    global_alt = pos.alt;
    global_rel_alt = pos.relative_alt;
    global_hdg = pos.hdg;
}

static void handle_scaled_pressure(const mavlink_message_t *msg) {
    mavlink_scaled_pressure_t press;
    mavlink_msg_scaled_pressure_decode(msg, &press);
    
    baro_press_abs = press.press_abs;
    baro_press_diff = press.press_diff;
    baro_temperature = press.temperature;
    baro_has_data = true;
}

static void handle_vfr_hud(const mavlink_message_t *msg) {
    mavlink_vfr_hud_t hud;
    mavlink_msg_vfr_hud_decode(msg, &hud);
    
    vfr_airspeed = hud.airspeed;
    vfr_groundspeed = hud.groundspeed;
    vfr_alt = hud.alt;
    vfr_climb = hud.climb;
    vfr_heading = hud.heading;
    vfr_throttle = hud.throttle;
    vfr_has_data = true;
}

static void handle_sys_status(const mavlink_message_t *msg) {
    mavlink_sys_status_t sys;
    mavlink_msg_sys_status_decode(msg, &sys);
    
    batt_voltage = sys.voltage_battery;
    batt_current = sys.current_battery;
    batt_remaining = sys.battery_remaining;
    batt_has_data = true;
}

static void handle_command_ack(const mavlink_message_t *msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(msg, &ack);
    
    const char *result_str = mavlink_result_to_string(ack.result);
    
    if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            add_log("ARM/DISARM: %s", result_str);
        } else {
            add_log("ARM/DISARM FAILED: %s", result_str);
        }
    } else if (ack.command == MAV_CMD_DO_SET_MODE) {
        if (ack.result == MAV_RESULT_ACCEPTED) {
            add_log("MODE CHANGE: %s", result_str);
        } else {
            add_log("MODE FAILED: %s", result_str);
        }
    } else {
        add_log("CMD %d: %s", ack.command, result_str);
    }
    
    ESP_LOGI(TAG, "COMMAND_ACK: cmd=%d result=%s", ack.command, result_str);
}

static void handle_param_value(const mavlink_message_t *msg) {
    mavlink_param_value_t param;
    mavlink_msg_param_value_decode(msg, &param);
    
    char safe_id[17];
    memcpy(safe_id, param.param_id, 16);
    safe_id[16] = '\0';
    
    add_log("PARAM: %s = %.2f", safe_id, param.param_value);
    ESP_LOGI(TAG, "PARAM_VALUE: %s = %.4f", safe_id, param.param_value);
}

static void handle_statustext(const mavlink_message_t *msg) {
    mavlink_statustext_t text;
    mavlink_msg_statustext_decode(msg, &text);
    
    char safe_text[51];
    int len = 0;
    for (int i = 0; i < 50 && text.text[i] != '\0'; i++) {
        if (text.text[i] < 32 || text.text[i] > 126) break;
        safe_text[len++] = text.text[i];
    }
    safe_text[len] = '\0';
    
    add_log("[%s] %s", mavlink_severity_to_string(text.severity), safe_text);
    ESP_LOGI(TAG, "STATUSTEXT: [%s] %s", mavlink_severity_to_string(text.severity), safe_text);
}

static void process_mavlink_message(const mavlink_message_t *msg) {
    static uint32_t msg_counts[256] = {0};
    if (msg->msgid < 256) {
        msg_counts[msg->msgid]++;
        if (msg_counts[msg->msgid] == 1) {
            ESP_LOGI(TAG, "First MSG_ID: %lu from SYS:%d COMP:%d", 
                     msg->msgid, msg->sysid, msg->compid);
        }
    }
    
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_heartbeat(msg);
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            handle_sys_status(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            handle_attitude(msg);
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            handle_gps_raw_int(msg);
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            handle_scaled_pressure(msg);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            handle_global_position_int(msg);
            break;
        case MAVLINK_MSG_ID_VFR_HUD:
            handle_vfr_hud(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handle_command_ack(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
            handle_param_value(msg);
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
            handle_statustext(msg);
            break;
    }
}

// ============================================================================
// FreeRTOS Tasks
// ============================================================================

static void mavlink_rx_task(void *pvParameters) {
    uint8_t data[128];
    
    ESP_LOGI(TAG, "MAVLink RX task started");
    
    while (1) {
        int len = uart_read_bytes(PIXHAWK_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t result = mavlink_parse_char(0, data[i], &mav_msg, &mav_status);
                if (result == MAVLINK_FRAMING_OK) {
                    process_mavlink_message(&mav_msg);
                }
            }
        }
        
        // Check connection timeout
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (is_connected && (now - last_heartbeat_time > 3000)) {
            is_connected = false;
            add_log("Connection lost!");
            ESP_LOGW(TAG, "Pixhawk connection lost");
        }
    }
}

static void mavlink_heartbeat_task(void *pvParameters) {
    ESP_LOGI(TAG, "Heartbeat task started (sysid=%d compid=%d)", 
             COMPANION_SYSTEM_ID, COMPANION_COMPONENT_ID);
    
    while (1) {
        send_heartbeat();
        
        // Safety: auto-release RC override if web client stopped communicating
        // This handles: WiFi out of range, browser tab closed, phone disconnected
        if (rc_override_active) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((now - rc_last_web_time) > RC_OVERRIDE_TIMEOUT_MS) {
                rc_override_active = false;
                rc_chan1 = 0; rc_chan2 = 0; rc_chan3 = 0; rc_chan4 = 0;
                send_rc_override(0, 0, 0, 0);
                add_log("RC SAFETY: auto-released (no web client)");
                ESP_LOGW(TAG, "RC override auto-released: web client timeout");
            } else {
                // Resend RC override at ~4Hz (Pixhawk reverts after its own timeout)
                send_rc_override(rc_chan1, rc_chan2, rc_chan3, rc_chan4);
            }
        }
        
        // Tick the auto-flight state machine (~4 Hz)
        auto_sequence_tick();
        
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ============================================================================
// WiFi AP
// ============================================================================

static void wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s", WIFI_AP_SSID);
    add_log("WiFi AP: %s", WIFI_AP_SSID);
}

// ============================================================================
// HTTP Server - HTML Page
// ============================================================================

static const char HTML_PAGE_PART1[] =
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>Pixhawk Control</title>\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{font-family:'Segoe UI',Arial,sans-serif;background:#0d1117;color:#e6edf3;padding:8px}\n"
".container{max-width:640px;margin:0 auto}\n"
"h1{text-align:center;color:#58a6ff;margin-bottom:16px;font-size:22px;letter-spacing:1px}\n"
".card{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:14px;margin-bottom:12px}\n"
".card h2{color:#58a6ff;margin-bottom:10px;font-size:14px;text-transform:uppercase;letter-spacing:1px;border-bottom:1px solid #21262d;padding-bottom:6px}\n"
/* Status row */
".status{display:flex;justify-content:space-between;gap:8px}\n"
".status-item{text-align:center;flex:1;background:#0d1117;border-radius:8px;padding:8px 4px}\n"
".status-value{font-size:20px;font-weight:700}\n"
".status-label{font-size:11px;color:#8b949e;margin-top:2px}\n"
".armed{color:#f85149}.disarmed{color:#3fb950}\n"
".connected{color:#3fb950}.disconnected{color:#f85149}\n"
/* Attitude */
".att-row{display:flex;justify-content:space-around;text-align:center;gap:8px}\n"
".att-item{flex:1;background:#0d1117;border-radius:8px;padding:10px 4px}\n"
".att-value{font-size:26px;font-weight:700;color:#e6edf3}\n"
".att-label{font-size:11px;color:#8b949e;margin-top:2px}\n"
/* Buttons */
".controls{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap}\n"
".btn{flex:1;padding:12px 8px;border:none;border-radius:8px;font-size:13px;font-weight:700;cursor:pointer;min-width:70px;transition:opacity .2s;text-transform:uppercase;letter-spacing:.5px}\n"
".btn:hover{opacity:.85}\n"
".btn:active{transform:scale(.97)}\n"
".btn-arm{background:#da3633;color:#fff}\n"
".btn-force{background:#d29922;color:#fff}\n"
".btn-disarm{background:#238636;color:#fff}\n"
".btn-setup{background:#1f6feb;color:#fff}\n"
"select{width:100%;padding:10px;font-size:14px;border-radius:8px;border:1px solid #30363d;background:#0d1117;color:#e6edf3;margin-top:8px;cursor:pointer}\n"
"select:focus{border-color:#58a6ff;outline:none}\n"
/* Data grid */
".data-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}\n"
".data-grid.cols3{grid-template-columns:1fr 1fr 1fr}\n"
".data-item{background:#0d1117;border-radius:8px;padding:8px;text-align:center}\n"
".data-val{font-size:18px;font-weight:700;color:#e6edf3}\n"
".data-lbl{font-size:10px;color:#8b949e;margin-top:2px;text-transform:uppercase}\n"
".data-val.good{color:#3fb950}\n"
".data-val.warn{color:#d29922}\n"
".data-val.bad{color:#f85149}\n"
/* RC sliders */
".rc-group{margin-bottom:8px}\n"
".rc-label{display:flex;justify-content:space-between;font-size:12px;color:#8b949e;margin-bottom:4px}\n"
".rc-label span:last-child{color:#e6edf3;font-weight:700;font-family:monospace}\n"
"input[type=range]{-webkit-appearance:none;width:100%;height:16px;background:#21262d;border-radius:8px;outline:none;margin:6px 0}\n"
"input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:40px;height:40px;background:#58a6ff;border-radius:50%;cursor:pointer;border:3px solid #1f6feb}\n"
".rc-btns{display:flex;gap:8px;margin-top:8px}\n"
".btn-rc{background:#238636;color:#fff}\n"
".btn-rc-stop{background:#da3633;color:#fff}\n"
".btn-takeoff{background:#6e40c9;color:#fff;flex:2}\n"
".btn-abort{background:#da3633;color:#fff;flex:1}\n"
".seq-box{background:#0d1117;border-radius:8px;padding:10px;margin-bottom:8px;text-align:center}\n"
".seq-state{font-size:20px;font-weight:700}\n"
/* Logs */
".logs{background:#010409;border:1px solid #21262d;border-radius:8px;padding:8px;height:160px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:11px;line-height:1.5}\n"
".log-entry{padding:1px 0;border-bottom:1px solid #161b22;color:#8b949e}\n"
/* Footer */
".footer{text-align:center;color:#484f58;font-size:10px;margin-top:8px}\n"
".card-collapse .card-hdr{cursor:pointer;display:flex;justify-content:space-between;align-items:center;user-select:none;-webkit-user-select:none}\n"
".card-collapse .card-hdr::after{content:'\\25BC';font-size:10px;color:#8b949e;transition:transform .2s}\n"
".card-collapse.collapsed .card-hdr::after{transform:rotate(-90deg)}\n"
".card-collapse.collapsed .card-body{display:none}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class='container'>\n"
"<h1>&#9992; PIXHAWK CONTROL</h1>\n"
"\n"
"<div class='card'>\n"
"<h2>Status</h2>\n"
"<div class='status'>\n"
"<div class='status-item'><div id='conn' class='status-value disconnected'>---</div><div class='status-label'>Connection</div></div>\n"
"<div class='status-item'><div id='armed' class='status-value'>---</div><div class='status-label'>Armed</div></div>\n"
"<div class='status-item'><div id='mode' class='status-value' style='font-size:16px'>---</div><div class='status-label'>Mode</div></div>\n"
"<div class='status-item'><div id='sysstat' class='status-value' style='font-size:14px'>---</div><div class='status-label'>State</div></div>\n"
"</div>\n"
"<div class='status' style='margin-top:8px'>\n"
"<div class='status-item'><div id='batt_v' class='status-value' style='font-size:18px'>---</div><div class='status-label'>Battery (V)</div></div>\n"
"<div class='status-item'><div id='batt_a' class='status-value' style='font-size:18px'>---</div><div class='status-label'>Current (A)</div></div>\n"
"<div class='status-item'><div id='batt_pct' class='status-value' style='font-size:18px'>---</div><div class='status-label'>Remaining</div></div>\n"
"</div>\n"
"</div>\n"
"\n"
"<div class='card card-collapse collapsed'>\n"
"<h2 class='card-hdr' onclick='toggleCard(this)'>Attitude</h2>\n"
"<div class='card-body'>\n"
"<div class='att-row'>\n"
"<div class='att-item'><div id='roll' class='att-value'>0.0&deg;</div><div class='att-label'>ROLL</div></div>\n"
"<div class='att-item'><div id='pitch' class='att-value'>0.0&deg;</div><div class='att-label'>PITCH</div></div>\n"
"<div class='att-item'><div id='yaw' class='att-value'>0.0&deg;</div><div class='att-label'>YAW</div></div>\n"
"</div>\n"
"</div>\n"
"</div>\n"
"\n";

static const char HTML_PAGE_PART2[] =
"<div class='card card-collapse collapsed'>\n"
"<h2 class='card-hdr' onclick='toggleCard(this)'>GPS</h2>\n"
"<div class='card-body'>\n"
"<div class='data-grid cols3'>\n"
"<div class='data-item'><div id='gps_fix' class='data-val bad'>---</div><div class='data-lbl'>Fix</div></div>\n"
"<div class='data-item'><div id='gps_sats' class='data-val'>0</div><div class='data-lbl'>Satellites</div></div>\n"
"<div class='data-item'><div id='gps_hdop' class='data-val'>---</div><div class='data-lbl'>HDOP</div></div>\n"
"<div class='data-item'><div id='gps_lat' class='data-val' style='font-size:14px'>---</div><div class='data-lbl'>Latitude</div></div>\n"
"<div class='data-item'><div id='gps_lon' class='data-val' style='font-size:14px'>---</div><div class='data-lbl'>Longitude</div></div>\n"
"<div class='data-item'><div id='gps_alt' class='data-val'>---</div><div class='data-lbl'>GPS Alt (m)</div></div>\n"
"</div>\n"
"</div>\n"
"</div>\n"
"\n"
"<div class='card card-collapse collapsed'>\n"
"<h2 class='card-hdr' onclick='toggleCard(this)'>Flight Data</h2>\n"
"<div class='card-body'>\n"
"<div class='data-grid cols3'>\n"
"<div class='data-item'><div id='vfr_alt' class='data-val'>---</div><div class='data-lbl'>Alt MSL (m)</div></div>\n"
"<div class='data-item'><div id='rel_alt' class='data-val'>---</div><div class='data-lbl'>Rel Alt (m)</div></div>\n"
"<div class='data-item'><div id='vfr_gspd' class='data-val'>---</div><div class='data-lbl'>GndSpd (m/s)</div></div>\n"
"<div class='data-item'><div id='vfr_hdg' class='data-val'>---</div><div class='data-lbl'>Heading (&deg;)</div></div>\n"
"<div class='data-item'><div id='vfr_climb' class='data-val'>---</div><div class='data-lbl'>Climb (m/s)</div></div>\n"
"<div class='data-item'><div id='vfr_thr' class='data-val'>---</div><div class='data-lbl'>Throttle (%)</div></div>\n"
"</div>\n"
"</div>\n"
"</div>\n"
"\n"
"<div class='card card-collapse collapsed'>\n"
"<h2 class='card-hdr' onclick='toggleCard(this)'>Barometer</h2>\n"
"<div class='card-body'>\n"
"<div class='data-grid cols3'>\n"
"<div class='data-item'><div id='baro_press' class='data-val'>---</div><div class='data-lbl'>Pressure (hPa)</div></div>\n"
"<div class='data-item'><div id='baro_temp' class='data-val'>---</div><div class='data-lbl'>Temp (&deg;C)</div></div>\n"
"<div class='data-item'><div id='baro_alt' class='data-val'>---</div><div class='data-lbl'>Baro Alt (m)</div></div>\n"
"</div>\n"
"</div>\n"
"</div>\n"
"\n"
"<div class='card'>\n"
"<h2>Controls</h2>\n"
"<div class='controls'>\n"
"<button class='btn btn-setup' onclick='doSetup()'>Setup</button>\n"
"<button class='btn btn-arm' onclick='doArm()'>Arm</button>\n"
"<button class='btn btn-force' onclick='doForceArm()'>Force Arm</button>\n"
"<button class='btn btn-disarm' onclick='doDisarm()'>Disarm</button>\n"
"</div>\n"
"<select id='modeSelect' onchange='doSetMode()'>\n"
"<option value=''>-- Select Mode --</option>\n"
"<option value='0'>STABILIZE</option>\n"
"<option value='1'>ACRO</option>\n"
"<option value='2'>ALT_HOLD</option>\n"
"<option value='3'>AUTO</option>\n"
"<option value='4'>GUIDED</option>\n"
"<option value='5'>LOITER</option>\n"
"<option value='6'>RTL</option>\n"
"<option value='7'>CIRCLE</option>\n"
"<option value='9'>LAND</option>\n"
"<option value='16'>POSHOLD</option>\n"
"<option value='17'>BRAKE</option>\n"
"<option value='21'>SMART_RTL</option>\n"
"</select>\n"
"</div>\n"
"\n"
"<div class='card'>\n"
"<h2>&#128640; Auto Flight</h2>\n"
"<div class='seq-box'>\n"
"<div class='seq-state' id='seq_st' style='color:#8b949e'>IDLE</div>\n"
"<div id='seq_cd' style='display:none;font-size:16px;color:#d29922;margin-top:4px'>Hovering: <span id='seq_cd_val'>10</span>s</div>\n"
"</div>\n"
"<div style='display:flex;gap:8px'>\n"
"<button class='btn btn-takeoff' id='btnTakeoff' onclick='doTakeoff()'>TAKEOFF SEQUENCE</button>\n"
"<button class='btn btn-abort' id='btnAbort' onclick='doAbort()' style='opacity:0.35;pointer-events:none'>&#9888; ABORT</button>\n"
"</div>\n"
"<div style='margin-top:8px;font-size:11px;color:#8b949e;line-height:1.6'>\n"
"GUIDED &rarr; ARM &rarr; Takeoff 5 m &rarr; Hover 10 s &rarr; LAND<br>\n"
"Geofence: 10 m altitude / 30 m radius &bull; LAND on breach\n"
"</div>\n"
"</div>\n"
"\n";

static const char HTML_PAGE_PART3[] =
"<div class='card'>\n"
"<h2>RC Override</h2>\n"
"<div class='rc-group'>\n"
"<div class='rc-label'><span>Throttle (CH3)</span><span id='thr_val'>1000</span></div>\n"
"<input type='range' id='thr' min='1000' max='2000' value='1000' step='10'>\n"
"</div>\n"
"<div class='rc-group'>\n"
"<div class='rc-label'><span>Yaw (CH4)</span><span id='yaw_val'>1500</span></div>\n"
"<input type='range' id='yaw_rc' min='1000' max='2000' value='1500' step='10'>\n"
"</div>\n"
"<div class='rc-group'>\n"
"<div class='rc-label'><span>Pitch (CH2)</span><span id='pitch_val'>1500</span></div>\n"
"<input type='range' id='pitch_rc' min='1000' max='2000' value='1500' step='10'>\n"
"</div>\n"
"<div class='rc-group'>\n"
"<div class='rc-label'><span>Roll (CH1)</span><span id='roll_val'>1500</span></div>\n"
"<input type='range' id='roll_rc' min='1000' max='2000' value='1500' step='10'>\n"
"</div>\n"
"<div class='rc-btns'>\n"
"<button class='btn btn-rc' onclick='doRcSend()'>Send RC</button>\n"
"<button class='btn btn-rc-stop' onclick='doRcStop()'>Release RC</button>\n"
"</div>\n"
"</div>\n"
"\n"
"<div class='card'>\n"
"<h2>Logs</h2>\n"
"<div id='logs' class='logs'></div>\n"
"</div>\n"
"\n"
"<div class='footer'>ESP32 Companion &bull; SysID " "200" " &bull; MAVLink v2</div>\n"
"</div>\n"
"\n";

static const char HTML_PAGE_PART4[] =
"<script>\n"
"var rcActive=false;\n"
"\n"
"function update(){\n"
"fetch('/api/data').then(r=>r.json()).then(d=>{\n"
"document.getElementById('conn').textContent=d.connected?'OK':'LOST';\n"
"document.getElementById('conn').className='status-value '+(d.connected?'connected':'disconnected');\n"
"document.getElementById('armed').textContent=d.armed?'ARMED':'SAFE';\n"
"document.getElementById('armed').className='status-value '+(d.armed?'armed':'disarmed');\n"
"document.getElementById('mode').textContent=d.mode;\n"
"document.getElementById('sysstat').textContent=d.status;\n"
/* Battery */
"if(d.batt_has){\n"
"var bv=d.batt_v/1000;document.getElementById('batt_v').textContent=bv.toFixed(2);\n"
"document.getElementById('batt_v').className='status-value '+(bv>11.1?'connected':(bv>10.5?'':'armed'));\n"
"document.getElementById('batt_a').textContent=d.batt_a>=0?(d.batt_a/100).toFixed(1):'---';\n"
"document.getElementById('batt_pct').textContent=d.batt_pct>=0?d.batt_pct+'%':'---';\n"
"document.getElementById('batt_pct').className='status-value '+(d.batt_pct>25?'connected':(d.batt_pct>10?'':'armed'));\n"
"}\n"
"document.getElementById('roll').innerHTML=d.roll.toFixed(1)+'&deg;';\n"
"document.getElementById('pitch').innerHTML=d.pitch.toFixed(1)+'&deg;';\n"
"document.getElementById('yaw').innerHTML=d.yaw.toFixed(1)+'&deg;';\n"
/* GPS */
"var fc=d.gps_fix>=3?'good':(d.gps_fix>=2?'warn':'bad');\n"
"document.getElementById('gps_fix').textContent=d.gps_fix_str;\n"
"document.getElementById('gps_fix').className='data-val '+fc;\n"
"document.getElementById('gps_sats').textContent=d.gps_sats;\n"
"document.getElementById('gps_sats').className='data-val '+(d.gps_sats>=6?'good':(d.gps_sats>=4?'warn':'bad'));\n"
"var hd=d.gps_hdop/100;\n"
"document.getElementById('gps_hdop').textContent=hd<99?hd.toFixed(1):'---';\n"
"document.getElementById('gps_lat').textContent=d.gps_has?(d.gps_lat/1e7).toFixed(7):'---';\n"
"document.getElementById('gps_lon').textContent=d.gps_has?(d.gps_lon/1e7).toFixed(7):'---';\n"
"document.getElementById('gps_alt').textContent=d.gps_has?(d.gps_alt/1000).toFixed(1):'---';\n"
/* Flight data */
"document.getElementById('vfr_alt').textContent=d.vfr_has?d.vfr_alt.toFixed(1):'---';\n"
"document.getElementById('rel_alt').textContent=(d.rel_alt/1000).toFixed(1);\n"
"document.getElementById('vfr_gspd').textContent=d.vfr_has?d.vfr_gspd.toFixed(1):'---';\n"
"document.getElementById('vfr_hdg').textContent=d.vfr_has?d.vfr_hdg:'---';\n"
"document.getElementById('vfr_climb').textContent=d.vfr_has?d.vfr_climb.toFixed(2):'---';\n"
"document.getElementById('vfr_thr').textContent=d.vfr_has?d.vfr_thr:'---';\n"
/* Baro */
"document.getElementById('baro_press').textContent=d.baro_has?d.baro_press.toFixed(1):'---';\n"
"document.getElementById('baro_temp').textContent=d.baro_has?(d.baro_temp/100).toFixed(1):'---';\n"
"if(d.baro_has){var ba=44330*(1-Math.pow(d.baro_press/1013.25,0.1903));document.getElementById('baro_alt').textContent=ba.toFixed(1);}else{document.getElementById('baro_alt').textContent='---';}\n"
/* Logs */
"var ld=document.getElementById('logs');\n"
"ld.innerHTML=d.logs.map(l=>'<div class=\"log-entry\">'+l+'</div>').join('');\n"
"ld.scrollTop=ld.scrollHeight;\n"
/* Auto flight sequence UI */
"var ss=d.seq_state||0;\n"
"document.getElementById('seq_st').textContent=d.seq_name||'IDLE';\n"
"document.getElementById('seq_st').style.color=ss==0?'#8b949e':(ss==7?'#f85149':(ss==6?'#3fb950':'#d29922'));\n"
"document.getElementById('seq_cd').style.display=ss==4?'block':'none';\n"
"document.getElementById('seq_cd_val').textContent=d.seq_hover||0;\n"
"document.getElementById('btnAbort').style.opacity=(ss>0&&ss<6)?'1':'0.35';\n"
"document.getElementById('btnAbort').style.pointerEvents=(ss>0&&ss<6)?'auto':'none';\n"
"document.getElementById('btnTakeoff').style.display=(ss>0&&ss<6)?'none':'block';\n"
"}).catch(e=>console.error(e));}\n"
"\n"
"function doSetup(){fetch('/api/setup',{method:'POST'}).then(()=>update());}\n"
"function doArm(){fetch('/api/arm',{method:'POST'}).then(()=>update());}\n"
"function doForceArm(){fetch('/api/forcearm',{method:'POST'}).then(()=>update());}\n"
"function doDisarm(){fetch('/api/disarm',{method:'POST'}).then(()=>update());}\n"
"function doSetMode(){var m=document.getElementById('modeSelect').value;if(m)fetch('/api/mode?m='+m,{method:'POST'}).then(()=>update());document.getElementById('modeSelect').value='';}\n"
"function doTakeoff(){if(!confirm('Start auto takeoff?\\nGUIDED > ARM > 5m > hover 10s > LAND\\n\\nAre props clear?'))return;fetch('/api/takeoff',{method:'POST'}).then(()=>update());}\n"
"function doAbort(){fetch('/api/abort',{method:'POST'}).then(()=>update());}\n"
"\n"
/* RC slider value display */
"['thr','yaw_rc','pitch_rc','roll_rc'].forEach(function(id){\n"
"document.getElementById(id).addEventListener('input',function(){\n"
"var lbl={'thr':'thr_val','yaw_rc':'yaw_val','pitch_rc':'pitch_val','roll_rc':'roll_val'};\n"
"document.getElementById(lbl[id]).textContent=this.value;\n"
"});});\n"
"\n"
"function doRcSend(){\n"
"rcActive=true;\n"
"sendRc();\n"
"}\n"
"function sendRc(){\n"
"var t=document.getElementById('thr').value;\n"
"var y=document.getElementById('yaw_rc').value;\n"
"var p=document.getElementById('pitch_rc').value;\n"
"var r=document.getElementById('roll_rc').value;\n"
"fetch('/api/rc?r='+r+'&p='+p+'&t='+t+'&y='+y,{method:'POST'});\n"
"}\n"
"function doRcStop(){\n"
"rcActive=false;\n"
"document.getElementById('thr').value=1000;document.getElementById('thr_val').textContent='1000';\n"
"document.getElementById('yaw_rc').value=1500;document.getElementById('yaw_val').textContent='1500';\n"
"document.getElementById('pitch_rc').value=1500;document.getElementById('pitch_val').textContent='1500';\n"
"document.getElementById('roll_rc').value=1500;document.getElementById('roll_val').textContent='1500';\n"
"fetch('/api/rc/stop',{method:'POST'});\n"
"}\n"
"\n"
/* Yaw/pitch/roll sliders snap back to center on release */
"['yaw_rc','pitch_rc','roll_rc'].forEach(function(id){\n"
"document.getElementById(id).addEventListener('mouseup',function(){this.value=1500;document.getElementById({'yaw_rc':'yaw_val','pitch_rc':'pitch_val','roll_rc':'roll_val'}[id]).textContent='1500';if(rcActive)sendRc();});\n"
"document.getElementById(id).addEventListener('touchend',function(){this.value=1500;document.getElementById({'yaw_rc':'yaw_val','pitch_rc':'pitch_val','roll_rc':'roll_val'}[id]).textContent='1500';if(rcActive)sendRc();});\n"
"});\n"
"\n"
/* Auto-send RC while active and sliders are being moved */
"var rcTimer=null;\n"
"['thr','yaw_rc','pitch_rc','roll_rc'].forEach(function(id){\n"
"document.getElementById(id).addEventListener('input',function(){\n"
"if(rcActive)sendRc();\n"
"});});\n"
"\n"
"function toggleCard(el){el.parentElement.classList.toggle('collapsed');}\n"
"\n"
"setInterval(update,500);update();\n"
"</script>\n"
"</body>\n"
"</html>";

// ============================================================================
// HTTP Handlers
// ============================================================================

static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, HTML_PAGE_PART1, sizeof(HTML_PAGE_PART1) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE_PART2, sizeof(HTML_PAGE_PART2) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE_PART3, sizeof(HTML_PAGE_PART3) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE_PART4, sizeof(HTML_PAGE_PART4) - 1);
    httpd_resp_send_chunk(req, NULL, 0); // End chunked response
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    char json[2048];
    char logs_json[600] = "[";
    
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < log_count; i++) {
            int idx = (log_head - log_count + i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (i > 0) strcat(logs_json, ",");
            strcat(logs_json, "\"");
            char *p = log_buffer[idx];
            char *d = logs_json + strlen(logs_json);
            while (*p && (d - logs_json) < 580) {
                if (*p == '"') { *d++ = '\\'; }
                if (*p == '\\' && *(p+1) != '"') { *d++ = '\\'; }
                *d++ = *p++;
            }
            *d = '\0';
            strcat(logs_json, "\"");
        }
        xSemaphoreGive(log_mutex);
    }
    strcat(logs_json, "]");
    
    // Estimate barometric altitude from pressure (ISA formula)
    // Alt = 44330 * (1 - (P/P0)^0.1903)
    float baro_alt_est = 0.0f;
    if (baro_has_data && baro_press_abs > 0) {
        baro_alt_est = 44330.0f * (1.0f - powf(baro_press_abs / 1013.25f, 0.1903f));
    }
    
    snprintf(json, sizeof(json),
        "{\"connected\":%s,\"armed\":%s,\"mode\":\"%s\",\"status\":\"%s\","
        "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
        "\"gps_has\":%s,\"gps_fix\":%d,\"gps_fix_str\":\"%s\",\"gps_sats\":%d,"
        "\"gps_hdop\":%d,\"gps_lat\":%ld,\"gps_lon\":%ld,\"gps_alt\":%ld,"
        "\"rel_alt\":%ld,"
        "\"baro_has\":%s,\"baro_press\":%.2f,\"baro_temp\":%d,\"baro_alt\":%.1f,"
        "\"vfr_has\":%s,\"vfr_alt\":%.2f,\"vfr_gspd\":%.2f,\"vfr_hdg\":%d,"
        "\"vfr_climb\":%.2f,\"vfr_thr\":%d,"
        "\"batt_has\":%s,\"batt_v\":%d,\"batt_a\":%d,\"batt_pct\":%d,"
        "\"seq_state\":%d,\"seq_hover\":%d,\"seq_name\":\"%s\","
        "\"rc_active\":%s,"
        "\"logs\":%s}",
        is_connected ? "true" : "false",
        is_armed ? "true" : "false",
        get_mode_name(pixhawk_custom_mode),
        get_status_name(pixhawk_system_status),
        current_roll, current_pitch, current_yaw,
        gps_has_data ? "true" : "false",
        gps_fix_type,
        mavlink_gps_fix_type_string(gps_fix_type),
        gps_satellites,
        gps_eph,
        (long)gps_lat, (long)gps_lon, (long)gps_alt,
        (long)global_rel_alt,
        baro_has_data ? "true" : "false",
        baro_press_abs,
        baro_temperature,
        baro_alt_est,
        vfr_has_data ? "true" : "false",
        vfr_alt, vfr_groundspeed, vfr_heading,
        vfr_climb, vfr_throttle,
        batt_has_data ? "true" : "false",
        batt_voltage, batt_current, batt_remaining,
        (int)seq_state, seq_hover_remaining, seq_state_name(seq_state),
        rc_override_active ? "true" : "false",
        logs_json
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    
    // Update web activity timestamp (browser is still polling)
    rc_last_web_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    return ESP_OK;
}

static esp_err_t setup_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== SETUP: Disabling pre-arm checks ===");
    disable_prearm_checks();
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t arm_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== ARM requested ===");
    // Always disable auto-disarm (bench testing: landing detector triggers after 10s)
    send_param_set("DISARM_DELAY", 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_rc_override_throttle_low();
    add_log("RC Override: throttle low");
    send_arm_command(true, false);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t force_arm_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== FORCE ARM requested ===");
    send_param_set("DISARM_DELAY", 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_rc_override_throttle_low();
    add_log("RC Override: throttle low");
    send_arm_command(true, true);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t disarm_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== DISARM requested ===");
    send_arm_command(false, false);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char mode_str[8];
        if (httpd_query_key_value(buf, "m", mode_str, sizeof(mode_str)) == ESP_OK) {
            uint32_t mode = atoi(mode_str);
            ESP_LOGI(TAG, "Setting mode to %lu (%s)", mode, get_mode_name(mode));
            send_mode_command(mode);
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/**
 * @brief RC Override handler - sets RC channel values
 * Query params: r=roll(ch1), p=pitch(ch2), t=throttle(ch3), y=yaw(ch4)
 * Values 1000-2000 us
 */
static esp_err_t rc_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(buf, "r", val, sizeof(val)) == ESP_OK) rc_chan1 = atoi(val);
        if (httpd_query_key_value(buf, "p", val, sizeof(val)) == ESP_OK) rc_chan2 = atoi(val);
        if (httpd_query_key_value(buf, "t", val, sizeof(val)) == ESP_OK) rc_chan3 = atoi(val);
        if (httpd_query_key_value(buf, "y", val, sizeof(val)) == ESP_OK) rc_chan4 = atoi(val);
    }
    
    rc_override_active = true;
    rc_last_web_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    send_rc_override(rc_chan1, rc_chan2, rc_chan3, rc_chan4);
    
    ESP_LOGI(TAG, "RC Override: R=%d P=%d T=%d Y=%d", rc_chan1, rc_chan2, rc_chan3, rc_chan4);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/**
 * @brief Stop RC Override - sends 0 for all channels (release control)
 */
static esp_err_t rc_stop_handler(httpd_req_t *req) {
    rc_override_active = false;
    rc_chan1 = 0; rc_chan2 = 0; rc_chan3 = 0; rc_chan4 = 0;
    send_rc_override(0, 0, 0, 0);
    add_log("RC Override released");
    ESP_LOGI(TAG, "RC Override stopped");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t takeoff_handler(httpd_req_t *req) {
    if (seq_state != SEQ_IDLE && seq_state != SEQ_COMPLETE && seq_state != SEQ_ABORTED) {
        httpd_resp_send(req, "BUSY", 4);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "=== AUTO TAKEOFF SEQUENCE START ===");
    add_log(">>> AUTO TAKEOFF SEQUENCE <<<");
    seq_abort_flag = false;
    seq_hover_remaining = seq_hover_seconds;
    seq_state = SEQ_PREFLIGHT;
    seq_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t abort_handler(httpd_req_t *req) {
    ESP_LOGW(TAG, "=== ABORT requested ===");
    if (seq_state > SEQ_IDLE && seq_state < SEQ_COMPLETE) {
        seq_abort_flag = true;
        add_log("ABORT requested by user");
    } else {
        /* If not in sequence, just switch to LAND as a safety measure */
        send_mode_command(9);
        add_log("LAND mode (no sequence active)");
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 14;
    config.stack_size = 8192;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root     = { .uri = "/",              .method = HTTP_GET,  .handler = root_handler };
        httpd_uri_t data     = { .uri = "/api/data",      .method = HTTP_GET,  .handler = data_handler };
        httpd_uri_t setup    = { .uri = "/api/setup",     .method = HTTP_POST, .handler = setup_handler };
        httpd_uri_t arm      = { .uri = "/api/arm",       .method = HTTP_POST, .handler = arm_handler };
        httpd_uri_t forcearm = { .uri = "/api/forcearm",  .method = HTTP_POST, .handler = force_arm_handler };
        httpd_uri_t disarm   = { .uri = "/api/disarm",    .method = HTTP_POST, .handler = disarm_handler };
        httpd_uri_t mode     = { .uri = "/api/mode",      .method = HTTP_POST, .handler = mode_handler };
        httpd_uri_t rc       = { .uri = "/api/rc",        .method = HTTP_POST, .handler = rc_handler };
        httpd_uri_t rcstop   = { .uri = "/api/rc/stop",   .method = HTTP_POST, .handler = rc_stop_handler };
        httpd_uri_t takeoff  = { .uri = "/api/takeoff",   .method = HTTP_POST, .handler = takeoff_handler };
        httpd_uri_t abort_f  = { .uri = "/api/abort",     .method = HTTP_POST, .handler = abort_handler };
        
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &data);
        httpd_register_uri_handler(server, &setup);
        httpd_register_uri_handler(server, &arm);
        httpd_register_uri_handler(server, &forcearm);
        httpd_register_uri_handler(server, &disarm);
        httpd_register_uri_handler(server, &mode);
        httpd_register_uri_handler(server, &rc);
        httpd_register_uri_handler(server, &rcstop);
        httpd_register_uri_handler(server, &takeoff);
        httpd_register_uri_handler(server, &abort_f);
        
        ESP_LOGI(TAG, "HTTP server started (11 endpoints)");
    }
    
    return server;
}

// ============================================================================
// Main
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "   ESP32 Pixhawk Web Interface");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Companion SysID: %d  Pixhawk SysID: %d", COMPANION_SYSTEM_ID, PIXHAWK_SYSTEM_ID);
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    uart_mutex = xSemaphoreCreateMutex();
    log_mutex = xSemaphoreCreateMutex();
    
    mavlink_status_init(&mav_status);
    uart_init();
    wifi_init_ap();
    start_webserver();
    
    // Heartbeat task runs at 4Hz (also resends RC override)
    xTaskCreate(mavlink_rx_task, "mav_rx", 4096, NULL, 10, NULL);
    xTaskCreate(mavlink_heartbeat_task, "mav_hb", 4096, NULL, 5, NULL);
    
    add_log("System ready (sysid=%d)", COMPANION_SYSTEM_ID);
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "WiFi: %s / %s", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "Web: http://192.168.4.1");
    ESP_LOGI(TAG, "=========================================");
}
