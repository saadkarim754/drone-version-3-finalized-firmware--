/* ==========================================================================
 * ESP32-CAM Drone Fire Detection + Pixhawk Companion Computer
 *
 * Merged firmware combining:
 *   1. Edge Impulse FOMO fire detection with live camera feed
 *   2. MAVLink companion computer for Pixhawk drone control
 *
 * Features:
 *   - Live camera feed with AI fire detection bounding box overlay
 *   - MAVLink v2 communication with Pixhawk autopilot
 *   - Unified web dashboard: camera + drone controls + telemetry
 *   - Arm / Disarm / Force-Arm / Flight mode selection
 *   - RC channel override (Throttle, Yaw, Pitch, Roll sliders)
 *   - Automated takeoff sequence (GUIDED > ARM > Takeoff > Hover > LAND)
 *   - GPS, Barometer, Battery, Attitude, VFR HUD telemetry
 *   - Geofence safety (altitude + radius)
 *
 * Hardware (two boards supported, auto-detected at compile time):
 *
 *   Board A: ESP32-CAM AI-Thinker   (CONFIG_IDF_TARGET_ESP32)
 *     MAVLink UART2  TX=GPIO13  RX=GPIO14
 *     LED GPIO 33 (active LOW)
 *     Camera: standard AI-Thinker pinout
 *
 *   Board B: GOOUUU ESP32-S3-CAM    (CONFIG_IDF_TARGET_ESP32S3)
 *     MAVLink UART1  TX=GPIO47  RX=GPIO21
 *     LED GPIO 48 (active HIGH)
 *     Camera: S3-CAM pinout (XCLK=15, SIOD=4, SIOC=5, etc.)
 *
 *   Both boards: Pixhawk flight controller connected via UART
 *
 * Configuration:
 *   - WiFi AP SSID: DroneFireDetect  Password: drone12345
 *   - Web interface: http://192.168.4.1
 *   - MAVLink UART: 57600 baud, 8N1
 *   - Companion System ID: 200 (avoids Pixhawk sysid=1 conflict)
 *
 * License: BSD-3-Clause-Clear (Edge Impulse SDK)
 * ========================================================================== */

/* ========================== INCLUDES ====================================== */
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"

/* Edge Impulse headers */
#include "ei_device_espressif_esp32.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"
#include "ei_analogsensor.h"
#include "ei_inertial_sensor.h"
#include "ei_camera.h"

/* MAVLink library (lightweight, header-only) */
#include "mavespstm.h"

/* ========================== CONFIGURATION ================================= */

/* --- WiFi AP --- */
#define WIFI_SSID           "DroneFireDetect"
#define WIFI_PASS           "drone12345"
#define WIFI_CHANNEL        1
#define MAX_CONNECTIONS     4

/* --- MAVLink UART to Pixhawk ---
 * Board-specific GPIO selection:
 *   ESP32 (AI-Thinker):  UART2  TX=13  RX=14   (free pins, camera uses others)
 *   ESP32-S3 (GOOUUU):   UART1  TX=47  RX=21   (free pins, camera uses 4-18)
 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define MAV_UART_NUM      UART_NUM_1
  #define MAV_TX_PIN        GPIO_NUM_47
  #define MAV_RX_PIN        GPIO_NUM_21
#else
  #define MAV_UART_NUM      UART_NUM_2
  #define MAV_TX_PIN        GPIO_NUM_13
  #define MAV_RX_PIN        GPIO_NUM_14
#endif
#define MAV_BAUD_RATE       57600
#define UART_BUF_SIZE       1024

/* --- MAVLink IDs --- */
#define COMPANION_SYSID     200
#define COMPANION_COMPID    MAV_COMP_ID_ONBOARD_COMPUTER
#define PIXHAWK_SYSID       1
#define PIXHAWK_COMPID      1

/* --- Indicator LED ---
 *   ESP32 (AI-Thinker):  GPIO 33 onboard red LED, active LOW
 *   ESP32-S3 (GOOUUU):   GPIO 48 user LED, active HIGH
 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define INDICATOR_LED_PIN GPIO_NUM_48
  #define LED_ACTIVE_LOW    0          /* active HIGH */
#else
  #define INDICATOR_LED_PIN GPIO_NUM_33
  #define LED_ACTIVE_LOW    1          /* active LOW  */
#endif

/* --- Logging --- */
#define LOG_BUFFER_SIZE     32
#define LOG_MSG_SIZE        120

/* --- RC Override Safety --- */
#define RC_OVERRIDE_TIMEOUT_MS 2000

/* --- Auto-flight --- */
#define SEQ_TAKEOFF_TIMEOUT_MS  30000
#define SEQ_ALT_TOLERANCE       1.0f
#define SEQ_ARM_TIMEOUT_MS      15000
#define FENCE_ALT_MAX           10.0f
#define FENCE_RADIUS            30.0f
#define MAV_CMD_NAV_TAKEOFF     22

static const char *TAG = "DroneFire";

/* ========================== FORWARD DECLARATIONS ========================== */

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceESP32::get_device());
static ATServer *at;
static EiDeviceESP32 *dev = NULL;
static httpd_handle_t http_server = NULL;

/* FreeRTOS tasks */
void serial_task(void *pvParameters);
void led_task(void *pvParameters);
void webserver_task(void *pvParameters);
void inference_task(void *pvParameters);
static void mavlink_rx_task(void *pvParameters);
static void mavlink_heartbeat_task(void *pvParameters);

/* ========================== GLOBAL STATE ================================== */

/* --- Mutexes --- */
static SemaphoreHandle_t detection_mutex = NULL;
static SemaphoreHandle_t uart_mutex = NULL;
static SemaphoreHandle_t log_mutex = NULL;

/* --- Cached camera frame (written by camera_pump_task, read by inference & HTTP) --- */
static SemaphoreHandle_t frame_cache_mutex = NULL;
static uint8_t *cached_jpg_buf  = NULL;
static size_t   cached_jpg_len  = 0;
/* Signaled by camera_pump_task every time a fresh JPEG lands in the cache.
 * Inference blocks on this instead of calling esp_camera_fb_get() directly. */
SemaphoreHandle_t camera_ready_sem = NULL;

/*
 * Called from ei_camera.cpp after each JPEG capture so the HTTP
 * frame_handler can serve the latest image without touching the
 * camera hardware (avoids DMA contention when WiFi is active).
 */
extern "C" void camera_cache_frame(const uint8_t *buf, size_t len) {
    if (!frame_cache_mutex || !buf || len == 0) return;
    if (xSemaphoreTake(frame_cache_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (cached_jpg_buf) free(cached_jpg_buf);
        // Force PSRAM to avoid fragmenting the precious internal SRAM for variable-size JPEGs
        cached_jpg_buf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
        if (cached_jpg_buf) {
            memcpy(cached_jpg_buf, buf, len);
            cached_jpg_len = len;
        } else {
            cached_jpg_len = 0;
        }
        xSemaphoreGive(frame_cache_mutex);
    }
}

/*
 * camera_get_cached_jpeg – called by ei_camera.cpp instead of esp_camera_fb_get().
 * Returns a heap-allocated copy of the latest cached JPEG.  Caller must free().
 */
extern "C" bool camera_get_cached_jpeg(uint8_t **buf_out, size_t *len_out)
{
    if (!frame_cache_mutex || !buf_out || !len_out) return false;
    if (xSemaphoreTake(frame_cache_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGE("cam_cache", "Mutex timeout in camera_get_cached_jpeg");
        return false;
    }
    bool ok = false;
    if (cached_jpg_buf && cached_jpg_len > 0) {
        *buf_out = (uint8_t *)malloc(cached_jpg_len);
        if (*buf_out) {
            memcpy(*buf_out, cached_jpg_buf, cached_jpg_len);
            *len_out = cached_jpg_len;
            ok = true;
        }
    }
    xSemaphoreGive(frame_cache_mutex);
    return ok;
}

/* --------------------------------------------------------------------------
 * camera_pump_task
 * The ONLY task that ever calls esp_camera_fb_get() / esp_camera_fb_return().
 * Runs at priority 15 on Core 1 (same core as the camera DMA ISR) so it
 * services hardware frames immediately – inference never starves the pipeline.
 * Inference blocks on camera_ready_sem then reads from the shared cache.
 * -------------------------------------------------------------------------- */
/*
 * camera_reinit – deinit + reinit the camera hardware to recover from a stuck
 * DMA state (typically caused by WiFi Block-ACK bursts starving the GDMA channel).
 * Returns true on success.
 */
static bool camera_reinit(void)
{
    ESP_LOGW("cam_pump", ">>> Camera reinit: deinit...");
    esp_camera_deinit();
    vTaskDelay(pdMS_TO_TICKS(200));  /* let DMA / bus settle */

    /* Re-read the current camera_config from ei_camera.cpp (it's a file-scope
     * static, but esp_camera_init uses the last config).  We just call
     * esp_camera_init with a minimal QQVGA config identical to what was used
     * at first boot. */
    ESP_LOGW("cam_pump", ">>> Camera reinit: init...");
    EiCameraESP32 *cam = static_cast<EiCameraESP32*>(EiCameraESP32::get_camera());
    /* init(96,96) selects QQVGA which is the smallest resolution */
    bool ok = cam->init(96, 96);
    if (ok) {
        ESP_LOGW("cam_pump", ">>> Camera reinit SUCCESS");
    } else {
        ESP_LOGE("cam_pump", ">>> Camera reinit FAILED – will retry");
    }
    return ok;
}

static void camera_pump_task(void *pvParameters)
{
    ESP_LOGI("cam_pump", "Pump task started – waiting for camera init...");
    /* Block until inference_task has called esp_camera_init() */
    while (esp_camera_sensor_get() == NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI("cam_pump", "Camera ready, pump running");

    int consecutive_fails = 0;
    const int REINIT_THRESHOLD = 3;  /* reinit camera after this many consecutive failures */

    while (1) {
        /* Yield briefly BEFORE fb_get so any pending WiFi DMA burst can finish
         * first – this dramatically reduces the chance of bus collision. */
        vTaskDelay(pdMS_TO_TICKS(10));

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            if (consecutive_fails > 0) {
                ESP_LOGI("cam_pump", "Camera recovered after %d failures", consecutive_fails);
            }
            consecutive_fails = 0;
            camera_cache_frame(fb->buf, fb->len);
            esp_camera_fb_return(fb);
            /* Notify inference that a fresh frame is in the cache. */
            xSemaphoreGive(camera_ready_sem);
        } else {
            consecutive_fails++;
            ESP_LOGW("cam_pump", "fb_get() returned NULL (%d consecutive)", consecutive_fails);

            if (consecutive_fails >= REINIT_THRESHOLD) {
                ESP_LOGE("cam_pump", "DMA appears stuck – reinitializing camera...");
                if (camera_reinit()) {
                    consecutive_fails = 0;
                } else {
                    /* reinit failed, wait longer before next attempt */
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            } else {
                /* Short back-off to let WiFi finish its burst */
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            continue;
        }
        /* ~15fps cap */
        vTaskDelay(pdMS_TO_TICKS(55));
    }
}

/* --- Fire Detection Data (shared with Edge Impulse inference) --- */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
    float confidence;
    char label[32];
    bool valid;
} detection_data_t;

static detection_data_t latest_detection = {0};
static bool alarm_active = false;

/* --- Camera settings --- */
static bool camera_vflip = false;
static bool camera_hflip = false;

/* --- MAVLink parser state --- */
static mavlink_status_t mav_status;
static mavlink_message_t mav_msg;
static uint8_t tx_seq = 0;

/* --- Attitude data --- */
static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;

/* --- Flight state --- */
static uint32_t heartbeat_count = 0;
static uint8_t pixhawk_base_mode = 0;
static uint32_t pixhawk_custom_mode = 0;
static uint8_t pixhawk_system_status = 0;
static bool is_armed = false;
static bool is_connected = false;
static uint32_t last_heartbeat_time = 0;

/* --- GPS data --- */
static int32_t gps_lat = 0;
static int32_t gps_lon = 0;
static int32_t gps_alt = 0;
static uint8_t gps_fix_type = 0;
static uint8_t gps_satellites = 0;
static uint16_t gps_eph = 9999;
static bool gps_has_data = false;

/* --- Global position (fused) --- */
static int32_t global_rel_alt = 0;

/* --- Barometer data --- */
static float baro_press_abs = 0.0f;
static int16_t baro_temperature = 0;
static bool baro_has_data = false;

/* --- Battery data --- */
static uint16_t batt_voltage = 0;
static int16_t batt_current = -1;
static int8_t batt_remaining = -1;
static bool batt_has_data = false;

/* --- VFR HUD data --- */
static float vfr_groundspeed = 0.0f;
static float vfr_alt = 0.0f;
static float vfr_climb = 0.0f;
static int16_t vfr_heading = 0;
static uint16_t vfr_throttle = 0;
static bool vfr_has_data = false;

/* --- RC Override state --- */
static uint16_t rc_chan1 = 0;
static uint16_t rc_chan2 = 0;
static uint16_t rc_chan3 = 0;
static uint16_t rc_chan4 = 0;
static bool rc_override_active = false;
static uint32_t rc_last_web_time = 0;

/* --- Auto-flight sequence --- */
typedef enum {
    SEQ_IDLE = 0,
    SEQ_PREFLIGHT,
    SEQ_ARMING,
    SEQ_TAKEOFF,
    SEQ_HOVERING,
    SEQ_LANDING,
    SEQ_COMPLETE,
    SEQ_ABORTED
} auto_seq_state_t;

static auto_seq_state_t seq_state = SEQ_IDLE;
static uint32_t seq_timer = 0;
static uint32_t seq_phase_start = 0;
static float seq_target_alt = 5.0f;
static int seq_hover_seconds = 10;
static int seq_hover_remaining = 0;
static bool seq_abort_flag = false;
static uint32_t seq_last_tkoff_send = 0;

/* --- Circular Log Buffer --- */
static char log_entries[LOG_BUFFER_SIZE][LOG_MSG_SIZE];
static int log_head = 0;
static int log_count = 0;

/* ========================== FLIGHT MODE TABLE ============================= */

typedef struct {
    uint32_t mode_num;
    const char *name;
} flight_mode_t;

static const flight_mode_t copter_modes[] = {
    {0, "STABILIZE"}, {1, "ACRO"},      {2, "ALT_HOLD"}, {3, "AUTO"},
    {4, "GUIDED"},    {5, "LOITER"},    {6, "RTL"},      {7, "CIRCLE"},
    {9, "LAND"},      {16, "POSHOLD"},  {17, "BRAKE"},   {21, "SMART_RTL"},
};
#define NUM_COPTER_MODES (sizeof(copter_modes) / sizeof(copter_modes[0]))

static const char *get_mode_name(uint32_t mode) {
    for (int i = 0; i < (int)NUM_COPTER_MODES; i++) {
        if (copter_modes[i].mode_num == mode) return copter_modes[i].name;
    }
    return "UNKNOWN";
}

static const char *get_status_name(uint8_t status) {
    switch (status) {
        case 0: return "UNINIT";    case 1: return "BOOT";
        case 2: return "CALIBRATING"; case 3: return "STANDBY";
        case 4: return "ACTIVE";    case 5: return "CRITICAL";
        case 6: return "EMERGENCY"; default: return "UNKNOWN";
    }
}

static const char *seq_state_name(auto_seq_state_t s) {
    switch (s) {
        case SEQ_IDLE: return "IDLE";         case SEQ_PREFLIGHT: return "PREFLIGHT";
        case SEQ_ARMING: return "ARMING";     case SEQ_TAKEOFF: return "TAKEOFF";
        case SEQ_HOVERING: return "HOVERING"; case SEQ_LANDING: return "LANDING";
        case SEQ_COMPLETE: return "COMPLETE";  case SEQ_ABORTED: return "ABORTED";
        default: return "UNKNOWN";
    }
}

/* ========================== LOGGING ======================================= */

void add_log(const char *fmt, ...) {
    if (!log_mutex) return;
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(log_entries[log_head], LOG_MSG_SIZE, fmt, args);
        va_end(args);
        log_head = (log_head + 1) % LOG_BUFFER_SIZE;
        if (log_count < LOG_BUFFER_SIZE) log_count++;
        xSemaphoreGive(log_mutex);
    }
}

/* ========================== MAVLINK UART ================================== */

static void mav_uart_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = MAV_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(MAV_UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART_NUM, MAV_TX_PIN, MAV_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "MAVLink UART: TX=%d RX=%d Baud=%d", MAV_TX_PIN, MAV_RX_PIN, MAV_BAUD_RATE);
}

/**
 * Send MAVLink message with CRC recomputed AFTER setting sequence number.
 * This avoids the silent-drop bug where Pixhawk rejects mis-CRC'd packets.
 */
static void send_mavlink_message(uint8_t *buf, uint16_t len) {
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buf[4] = tx_seq++;

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

        uart_write_bytes(MAV_UART_NUM, buf, len);
        xSemaphoreGive(uart_mutex);
    }
}

/* ========================== MAVLINK COMMANDS =============================== */

static void send_heartbeat(void) {
    uint8_t buf[32];
    uint16_t len = mavlink_msg_heartbeat_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE);
    send_mavlink_message(buf, len);
}

static void send_arm_command(bool arm, bool force) {
    uint8_t buf[48];
    float arm_param = arm ? 1.0f : 0.0f;
    float force_param = force ? (float)MAV_ARM_FORCE_MAGIC : 0.0f;

    uint16_t len = mavlink_msg_command_long_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        PIXHAWK_SYSID, 0,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        arm_param, force_param, 0, 0, 0, 0, 0);
    send_mavlink_message(buf, len);

    const char *cmd = arm ? (force ? "FORCE ARM" : "ARM") : "DISARM";
    add_log("Sent %s command", cmd);
    ESP_LOGI(TAG, "Sent %s command", cmd);
}

static void send_mode_command(uint32_t mode) {
    uint8_t buf[32];
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (is_armed) base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint16_t len = mavlink_msg_set_mode_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        PIXHAWK_SYSID, base_mode, mode);
    send_mavlink_message(buf, len);
    add_log("Set mode: %s", get_mode_name(mode));
}

static void send_param_set(const char *param_id, float value) {
    uint8_t buf[48];
    uint16_t len = mavlink_msg_param_set_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        PIXHAWK_SYSID, 0, param_id, value, MAV_PARAM_TYPE_REAL32);
    send_mavlink_message(buf, len);
    add_log("PARAM_SET: %s = %.0f", param_id, value);
}

static void send_rc_override(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
    uint8_t buf[32];
    uint16_t len = mavlink_msg_rc_channels_override_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        PIXHAWK_SYSID, 0,
        ch1, ch2, ch3, ch4,
        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
    send_mavlink_message(buf, len);
}

static void send_rc_override_throttle_low(void) {
    send_rc_override(0, 0, 1000, 0);
}

static void disable_prearm_checks(void) {
    add_log("Disabling checks & failsafes...");
    send_param_set("ARMING_CHECK", 0);   vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FS_THR_ENABLE", 0);  vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FS_GCS_ENABLE", 0);  vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("DISARM_DELAY", 0);   vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("SYSID_MYGCS", (float)COMPANION_SYSID);
    add_log("Set SYSID_MYGCS=%d", COMPANION_SYSID);
}

static void setup_geofence(void) {
    add_log("Geofence: alt=%.0fm rad=%.0fm", FENCE_ALT_MAX, FENCE_RADIUS);
    send_param_set("FENCE_TYPE", 3);          vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ALT_MAX", FENCE_ALT_MAX); vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_RADIUS", FENCE_RADIUS);   vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ACTION", 2);        vTaskDelay(pdMS_TO_TICKS(100));
    send_param_set("FENCE_ENABLE", 1);        vTaskDelay(pdMS_TO_TICKS(100));
}

static void send_takeoff_command(float altitude) {
    add_log("Takeoff cmd: %.1fm", altitude);
    uint8_t buf[64];
    uint16_t len = mavlink_msg_command_long_pack(
        COMPANION_SYSID, COMPANION_COMPID, buf,
        PIXHAWK_SYSID, 0,
        MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude);
    send_mavlink_message(buf, len);
}

/* ========================== AUTO-FLIGHT SEQUENCE ========================== */

static void auto_sequence_tick(void) {
    if (seq_state == SEQ_IDLE || seq_state == SEQ_COMPLETE || seq_state == SEQ_ABORTED)
        return;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (seq_abort_flag) {
        seq_abort_flag = false;
        add_log("!! ABORT: switching to LAND !!");
        send_mode_command(9);
        seq_state = SEQ_ABORTED;
        return;
    }

    switch (seq_state) {

    case SEQ_PREFLIGHT:
        if (gps_fix_type < 3 || gps_satellites < 6) {
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
        add_log("PREFLIGHT -> ARMING");
        break;

    case SEQ_ARMING:
        if (is_armed) {
            seq_state = SEQ_TAKEOFF;
            seq_timer = now;
            seq_last_tkoff_send = 0;
            add_log("ARMED -> TAKEOFF");
            send_takeoff_command(seq_target_alt);
            return;
        }
        if ((now - seq_phase_start) > SEQ_ARM_TIMEOUT_MS) {
            add_log("Failed to arm - aborting");
            seq_state = SEQ_ABORTED;
            break;
        }
        if (pixhawk_custom_mode != 4) {
            send_mode_command(4);
            add_log("Setting GUIDED mode...");
            return;
        }
        if ((now - seq_timer) > 2000) {
            send_rc_override_throttle_low();
            send_arm_command(true, false);
            add_log("Arm attempt...");
            seq_timer = now;
        }
        break;

    case SEQ_TAKEOFF: {
        float rel_alt_m = global_rel_alt / 1000.0f;
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
            return;
        }
        if ((now - seq_timer) > SEQ_TAKEOFF_TIMEOUT_MS) {
            add_log("Takeoff timeout (%.1fm) - LAND", rel_alt_m);
            send_mode_command(9);
            seq_state = SEQ_ABORTED;
        }
        break;
    }

    case SEQ_HOVERING: {
        int elapsed = (int)((now - seq_timer) / 1000);
        seq_hover_remaining = seq_hover_seconds - elapsed;
        if (seq_hover_remaining <= 0) {
            seq_hover_remaining = 0;
            add_log("Hover done - LANDING");
            send_mode_command(9);
            seq_state = SEQ_LANDING;
            seq_timer = now;
        }
        break;
    }

    case SEQ_LANDING:
        if (pixhawk_custom_mode != 9) send_mode_command(9);
        if (!is_armed) {
            seq_state = SEQ_COMPLETE;
            add_log("Landed & disarmed - COMPLETE!");
            return;
        }
        if ((now - seq_timer) > 30000) {
            add_log("Still landing...");
            seq_timer = now;
        }
        break;

    default: break;
    }
}

/* ========================== MAVLINK MESSAGE HANDLERS ====================== */

static void handle_heartbeat(const mavlink_message_t *msg) {
    if (msg->sysid == 255 || msg->sysid == COMPANION_SYSID) return;

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);

    bool new_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

    if (heartbeat_count > 0 && pixhawk_custom_mode != hb.custom_mode)
        add_log("Mode: %s -> %s", get_mode_name(pixhawk_custom_mode), get_mode_name(hb.custom_mode));
    if (heartbeat_count > 0 && is_armed != new_armed)
        add_log("%s", new_armed ? ">>> ARMED <<<" : ">>> DISARMED <<<");

    heartbeat_count++;
    pixhawk_base_mode = hb.base_mode;
    pixhawk_custom_mode = hb.custom_mode;
    pixhawk_system_status = hb.system_status;
    is_armed = new_armed;
    is_connected = true;
    last_heartbeat_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void handle_attitude(const mavlink_message_t *msg) {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(msg, &att);
    current_roll = mavlink_rad_to_deg(att.roll);
    current_pitch = mavlink_rad_to_deg(att.pitch);
    current_yaw = mavlink_rad_to_deg(att.yaw);
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
    gps_has_data = true;
}

static void handle_global_position_int(const mavlink_message_t *msg) {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(msg, &pos);
    global_rel_alt = pos.relative_alt;
}

static void handle_scaled_pressure(const mavlink_message_t *msg) {
    mavlink_scaled_pressure_t press;
    mavlink_msg_scaled_pressure_decode(msg, &press);
    baro_press_abs = press.press_abs;
    baro_temperature = press.temperature;
    baro_has_data = true;
}

static void handle_vfr_hud(const mavlink_message_t *msg) {
    mavlink_vfr_hud_t hud;
    mavlink_msg_vfr_hud_decode(msg, &hud);
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
    const char *res = mavlink_result_to_string(ack.result);
    add_log("CMD %d: %s", ack.command, res);
}

static void handle_statustext(const mavlink_message_t *msg) {
    mavlink_statustext_t text;
    mavlink_msg_statustext_decode(msg, &text);
    char safe[51];
    int len = 0;
    for (int i = 0; i < 50 && text.text[i] >= 32 && text.text[i] <= 126; i++)
        safe[len++] = text.text[i];
    safe[len] = '\0';
    add_log("[%s] %s", mavlink_severity_to_string(text.severity), safe);
}

static void handle_param_value(const mavlink_message_t *msg) {
    mavlink_param_value_t param;
    mavlink_msg_param_value_decode(msg, &param);
    char id[17];
    memcpy(id, param.param_id, 16);
    id[16] = '\0';
    add_log("PARAM: %s = %.2f", id, param.param_value);
}

static void process_mavlink_message(const mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:           handle_heartbeat(msg); break;
        case MAVLINK_MSG_ID_SYS_STATUS:          handle_sys_status(msg); break;
        case MAVLINK_MSG_ID_ATTITUDE:            handle_attitude(msg); break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:         handle_gps_raw_int(msg); break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:     handle_scaled_pressure(msg); break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: handle_global_position_int(msg); break;
        case MAVLINK_MSG_ID_VFR_HUD:             handle_vfr_hud(msg); break;
        case MAVLINK_MSG_ID_COMMAND_ACK:         handle_command_ack(msg); break;
        case MAVLINK_MSG_ID_PARAM_VALUE:         handle_param_value(msg); break;
        case MAVLINK_MSG_ID_STATUSTEXT:          handle_statustext(msg); break;
    }
}

/* ========================== DETECTION UPDATE (EI interface) ================ */

void update_detection_data(uint32_t x, uint32_t y, uint32_t width, uint32_t height,
                           float confidence, const char *label)
{
    ei_printf("[Detection] %s (%.2f) at (%u,%u) %ux%u\r\n",
              label, confidence, (unsigned)x, (unsigned)y, (unsigned)width, (unsigned)height);

    if (detection_mutex && xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(100))) {
        latest_detection.x = x;
        latest_detection.y = y;
        latest_detection.width = width;
        latest_detection.height = height;
        latest_detection.confidence = confidence;
        strncpy(latest_detection.label, label, sizeof(latest_detection.label) - 1);
        latest_detection.label[sizeof(latest_detection.label) - 1] = '\0';
        latest_detection.valid = (confidence > 0.0f);

        if (confidence > 0.5f && !alarm_active) {
            alarm_active = true;
            add_log("[FIRE] Detected: %s %.0f%%", label, confidence * 100.0f);
        } else if (confidence == 0.0f) {
            alarm_active = false;
        }

        xSemaphoreGive(detection_mutex);
    }
}

/* ========================== WIFI AP ======================================= */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
        add_log("[WiFi] Client connected");
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
        add_log("[WiFi] Client disconnected");
}

static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, WIFI_PASS);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    wifi_config.ap.channel = WIFI_CHANNEL;
    wifi_config.ap.max_connection = MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP: %s / %s", WIFI_SSID, WIFI_PASS);
    add_log("WiFi AP: %s", WIFI_SSID);
}

/* ========================== HTML DASHBOARD ================================ */

static const char HTML_PAGE1[] =
"<!DOCTYPE html>\n"
"<html><head><meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>Drone Fire Detection</title>\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{font-family:'Segoe UI',Arial,sans-serif;background:#0d1117;color:#e6edf3;padding:8px}\n"
".ct{max-width:640px;margin:0 auto}\n"
"h1{text-align:center;color:#58a6ff;margin-bottom:12px;font-size:20px}\n"
".cd{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:12px;margin-bottom:10px}\n"
".cd h2{color:#58a6ff;margin-bottom:8px;font-size:13px;text-transform:uppercase;letter-spacing:1px;"
"border-bottom:1px solid #21262d;padding-bottom:5px}\n"
".sb{display:flex;flex-wrap:wrap;gap:6px}\n"
".si{flex:1;min-width:55px;text-align:center;background:#0d1117;border-radius:8px;padding:6px}\n"
".sv{font-size:15px;font-weight:700}.sl{font-size:10px;color:#8b949e;margin-top:2px}\n"
".gd{color:#3fb950}.bd{color:#f85149}.wd{color:#d29922}\n"
".cam{position:relative;width:100%}#frm{width:100%;display:block;border-radius:8px}\n"
"#ovl{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}\n"
".fa{background:#da3633;color:#fff;text-align:center;padding:8px;border-radius:8px;"
"margin:8px 0;font-weight:700;display:none;animation:pulse 1s infinite}\n"
"@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}\n"
".br{display:flex;gap:6px;flex-wrap:wrap;margin:6px 0}\n"
".bt{flex:1;padding:10px;border:none;border-radius:8px;font-size:12px;font-weight:700;"
"cursor:pointer;min-width:55px;text-transform:uppercase}\n"
".bt:active{transform:scale(.97)}\n"
".b-arm{background:#da3633;color:#fff}.b-frc{background:#d29922;color:#fff}\n"
".b-dis{background:#238636;color:#fff}.b-set{background:#1f6feb;color:#fff}\n"
".b-rc{background:#238636;color:#fff}.b-rs{background:#da3633;color:#fff}\n"
".b-tk{background:#6e40c9;color:#fff;flex:2}.b-ab{background:#da3633;color:#fff;flex:1}\n"
".b-cm{background:#30363d;color:#e6edf3}\n"
"select{width:100%;padding:8px;font-size:13px;border-radius:8px;border:1px solid #30363d;"
"background:#0d1117;color:#e6edf3;margin-top:6px}\n"
".rg{margin-bottom:6px}\n"
".rl{display:flex;justify-content:space-between;font-size:11px;color:#8b949e;margin-bottom:2px}\n"
".rl span:last-child{color:#e6edf3;font-weight:700;font-family:monospace}\n"
"input[type=range]{-webkit-appearance:none;width:100%;height:14px;background:#21262d;"
"border-radius:7px;outline:none;margin:4px 0}\n"
"input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:32px;height:32px;"
"background:#58a6ff;border-radius:50%;cursor:pointer;border:2px solid #1f6feb}\n"
".dg{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}\n"
".di{background:#0d1117;border-radius:6px;padding:6px;text-align:center}\n"
".dv{font-size:14px;font-weight:700}.dl{font-size:9px;color:#8b949e;margin-top:1px;text-transform:uppercase}\n"
".co .ch{cursor:pointer;display:flex;justify-content:space-between;align-items:center;user-select:none}\n"
".co .ch::after{content:'\\25BC';font-size:9px;color:#8b949e;transition:transform .2s}\n"
".co.cl .ch::after{transform:rotate(-90deg)}.co.cl .cb{display:none}\n"
".lg{background:#010409;border:1px solid #21262d;border-radius:6px;padding:6px;"
"height:140px;overflow-y:auto;font-family:monospace;font-size:10px;line-height:1.5}\n"
".le{padding:1px 0;border-bottom:1px solid #161b22;color:#8b949e}\n"
".sq{background:#0d1117;border-radius:8px;padding:8px;text-align:center;margin-bottom:6px}\n"
".ss{font-size:18px;font-weight:700}\n"
".ft{text-align:center;color:#484f58;font-size:9px;margin-top:8px}\n"
"</style></head><body><div class='ct'>\n"
"<h1>&#128681; DRONE FIRE DETECTION</h1>\n";

static const char HTML_PAGE2[] =
"<div class='cd'><div class='sb'>\n"
"<div class='si'><div id='conn' class='sv bd'>---</div><div class='sl'>Link</div></div>\n"
"<div class='si'><div id='armed' class='sv'>---</div><div class='sl'>Armed</div></div>\n"
"<div class='si'><div id='mode' class='sv' style='font-size:12px'>---</div><div class='sl'>Mode</div></div>\n"
"<div class='si'><div id='bv' class='sv'>---</div><div class='sl'>Volts</div></div>\n"
"<div class='si'><div id='bp' class='sv'>---</div><div class='sl'>Batt%</div></div>\n"
"</div></div>\n"
"<div class='cd'><h2>Camera Feed</h2>\n"
"<div class='cam'><img id='frm'><canvas id='ovl'></canvas></div>\n"
"<div id='fireAlert' class='fa'>&#128293; FIRE DETECTED!</div>\n"
"<div style='margin-top:6px;font-size:11px;color:#8b949e' id='det'>Waiting...</div>\n"
"<div class='br' style='margin-top:6px'>\n"
"<button class='bt b-cm' onclick='toggleVF()'>V-Flip</button>\n"
"<button class='bt b-cm' onclick='toggleHF()'>H-Flip</button></div></div>\n"
"<div class='cd'><h2>Controls</h2>\n"
"<div class='br'>\n"
"<button class='bt b-set' onclick='doSetup()'>Setup</button>\n"
"<button class='bt b-arm' onclick='doArm()'>Arm</button>\n"
"<button class='bt b-frc' onclick='doForce()'>Force</button>\n"
"<button class='bt b-dis' onclick='doDisarm()'>Disarm</button></div>\n"
"<select id='ms' onchange='doMode()'>\n"
"<option value=''>-- Flight Mode --</option>\n"
"<option value='0'>STABILIZE</option><option value='1'>ACRO</option>\n"
"<option value='2'>ALT_HOLD</option><option value='3'>AUTO</option>\n"
"<option value='4'>GUIDED</option><option value='5'>LOITER</option>\n"
"<option value='6'>RTL</option><option value='7'>CIRCLE</option>\n"
"<option value='9'>LAND</option><option value='16'>POSHOLD</option>\n"
"<option value='17'>BRAKE</option><option value='21'>SMART_RTL</option>\n"
"</select></div>\n";

static const char HTML_PAGE3[] =
"<div class='cd co cl'><h2 class='ch' onclick='tc(this)'>RC Override</h2><div class='cb'>\n"
"<div class='rg'><div class='rl'><span>Throttle (CH3)</span><span id='tv'>1000</span></div>\n"
"<input type='range' id='thr' min='1000' max='2000' value='1000' step='10'></div>\n"
"<div class='rg'><div class='rl'><span>Yaw (CH4)</span><span id='yv'>1500</span></div>\n"
"<input type='range' id='yrc' min='1000' max='2000' value='1500' step='10'></div>\n"
"<div class='rg'><div class='rl'><span>Pitch (CH2)</span><span id='pv'>1500</span></div>\n"
"<input type='range' id='prc' min='1000' max='2000' value='1500' step='10'></div>\n"
"<div class='rg'><div class='rl'><span>Roll (CH1)</span><span id='rv'>1500</span></div>\n"
"<input type='range' id='rrc' min='1000' max='2000' value='1500' step='10'></div>\n"
"<div class='br'>\n"
"<button class='bt b-rc' onclick='doRcS()'>Send RC</button>\n"
"<button class='bt b-rs' onclick='doRcX()'>Release</button></div></div></div>\n"
"<div class='cd'><h2>&#128640; Auto Flight</h2>\n"
"<div class='sq'><div class='ss' id='sqs' style='color:#8b949e'>IDLE</div>\n"
"<div id='sqc' style='display:none;font-size:14px;color:#d29922;margin-top:4px'>"
"Hover: <span id='sqv'>10</span>s</div></div>\n"
"<div class='br'>\n"
"<button class='bt b-tk' id='btk' onclick='doTk()'>TAKEOFF</button>\n"
"<button class='bt b-ab' id='bab' onclick='doAb()' style='opacity:.35;pointer-events:none'>ABORT</button>\n"
"</div></div>\n";

static const char HTML_PAGE4[] =
"<div class='cd co cl'><h2 class='ch' onclick='tc(this)'>GPS &amp; Telemetry</h2><div class='cb'>\n"
"<div class='dg'>\n"
"<div class='di'><div id='gf' class='dv bd'>---</div><div class='dl'>Fix</div></div>\n"
"<div class='di'><div id='gs' class='dv'>0</div><div class='dl'>Sats</div></div>\n"
"<div class='di'><div id='gh' class='dv'>---</div><div class='dl'>HDOP</div></div>\n"
"<div class='di'><div id='gl' class='dv' style='font-size:11px'>---</div><div class='dl'>Lat</div></div>\n"
"<div class='di'><div id='go' class='dv' style='font-size:11px'>---</div><div class='dl'>Lon</div></div>\n"
"<div class='di'><div id='ga' class='dv'>---</div><div class='dl'>GPS Alt</div></div>\n"
"</div>\n"
"<div class='dg' style='margin-top:6px'>\n"
"<div class='di'><div id='ar' class='dv'>---</div><div class='dl'>Roll</div></div>\n"
"<div class='di'><div id='ap' class='dv'>---</div><div class='dl'>Pitch</div></div>\n"
"<div class='di'><div id='ay' class='dv'>---</div><div class='dl'>Yaw</div></div>\n"
"<div class='di'><div id='va' class='dv'>---</div><div class='dl'>Alt MSL</div></div>\n"
"<div class='di'><div id='ra' class='dv'>---</div><div class='dl'>Rel Alt</div></div>\n"
"<div class='di'><div id='vg' class='dv'>---</div><div class='dl'>GndSpd</div></div>\n"
"<div class='di'><div id='vh' class='dv'>---</div><div class='dl'>Heading</div></div>\n"
"<div class='di'><div id='vc' class='dv'>---</div><div class='dl'>Climb</div></div>\n"
"<div class='di'><div id='vt' class='dv'>---</div><div class='dl'>Thr%</div></div>\n"
"</div></div></div>\n"
"<div class='cd'><h2>Logs</h2><div id='logs' class='lg'></div></div>\n"
"<div class='ft'>ESP32-CAM Companion &bull; SysID 200 &bull; MAVLink v2</div>\n"
"</div>\n";

static const char HTML_PAGE5[] =
"<script>\n"
"var rcA=false,fetching=false;\n"
"var c=document.getElementById('ovl'),ctx=c.getContext('2d'),img=document.getElementById('frm');\n"
"\n"
"function update(){\n"
"fetch('/api/data').then(r=>r.json()).then(d=>{\n"
"document.getElementById('conn').textContent=d.cn?'OK':'LOST';\n"
"document.getElementById('conn').className='sv '+(d.cn?'gd':'bd');\n"
"document.getElementById('armed').textContent=d.ar?'ARMED':'SAFE';\n"
"document.getElementById('armed').className='sv '+(d.ar?'bd':'gd');\n"
"document.getElementById('mode').textContent=d.md;\n"
"if(d.bh){var bv=d.bV/1000;document.getElementById('bv').textContent=bv.toFixed(1);\n"
"document.getElementById('bv').className='sv '+(bv>11.1?'gd':(bv>10.5?'wd':'bd'));\n"
"document.getElementById('bp').textContent=d.bP>=0?d.bP+'%':'---';\n"
"document.getElementById('bp').className='sv '+(d.bP>25?'gd':(d.bP>10?'wd':'bd'));}\n"
"var fc=d.gx>=3?'gd':(d.gx>=2?'wd':'bd');\n"
"document.getElementById('gf').textContent=d.gfs;document.getElementById('gf').className='dv '+fc;\n"
"document.getElementById('gs').textContent=d.gn;\n"
"document.getElementById('gs').className='dv '+(d.gn>=6?'gd':(d.gn>=4?'wd':'bd'));\n"
"var hd=d.ge/100;document.getElementById('gh').textContent=hd<99?hd.toFixed(1):'---';\n"
"document.getElementById('gl').textContent=d.gD?(d.gla/1e7).toFixed(7):'---';\n"
"document.getElementById('go').textContent=d.gD?(d.glo/1e7).toFixed(7):'---';\n"
"document.getElementById('ga').textContent=d.gD?(d.gal/1000).toFixed(1):'---';\n"
"document.getElementById('ar').innerHTML=d.rl.toFixed(1)+'&deg;';\n"
"document.getElementById('ap').innerHTML=d.pt.toFixed(1)+'&deg;';\n"
"document.getElementById('ay').innerHTML=d.yw.toFixed(1)+'&deg;';\n"
"document.getElementById('va').textContent=d.vH?d.vA.toFixed(1):'---';\n"
"document.getElementById('ra').textContent=(d.rA/1000).toFixed(1);\n"
"document.getElementById('vg').textContent=d.vH?d.vG.toFixed(1):'---';\n"
"document.getElementById('vh').textContent=d.vH?d.vD:'---';\n"
"document.getElementById('vc').textContent=d.vH?d.vC.toFixed(2):'---';\n"
"document.getElementById('vt').textContent=d.vH?d.vT:'---';\n"
"var fa=document.getElementById('fireAlert');\n"
"fa.style.display=d.fv?'block':'none';\n"
"document.getElementById('det').innerHTML=d.fv?"
"'<span style=\"color:#f85149\">FIRE: '+d.fl+' '+(d.fc*100).toFixed(0)+'%</span>':'Monitoring (no fire)';\n"
"var ss=d.sS||0;\n"
"document.getElementById('sqs').textContent=d.sN||'IDLE';\n"
"document.getElementById('sqs').style.color=ss==0?'#8b949e':(ss>=6?(ss==6?'#3fb950':'#f85149'):'#d29922');\n"
"document.getElementById('sqc').style.display=ss==4?'block':'none';\n"
"document.getElementById('sqv').textContent=d.sH||0;\n"
"document.getElementById('bab').style.opacity=(ss>0&&ss<6)?'1':'0.35';\n"
"document.getElementById('bab').style.pointerEvents=(ss>0&&ss<6)?'auto':'none';\n"
"document.getElementById('btk').style.display=(ss>0&&ss<6)?'none':'block';\n"
"var ld=document.getElementById('logs');\n"
"ld.innerHTML=d.lg.map(l=>'<div class=\"le\">'+l+'</div>').join('');\n"
"ld.scrollTop=ld.scrollHeight;\n"
"}).catch(e=>console.error(e));}\n"
"\n"
"function fetchFrame(){\n"
"if(fetching)return;fetching=true;\n"
"fetch('/api/frame',{cache:'no-store'}).then(r=>r.json()).then(d=>{\n"
"if(d.image){img.onload=function(){c.width=img.width;c.height=img.height;\n"
"ctx.drawImage(img,0,0,c.width,c.height);\n"
"if(d.valid&&d.confidence>0){var sx=c.width/96,sy=c.height/96;\n"
"var x=d.x*sx,y=d.y*sy,w=d.width*sx,h=d.height*sy;\n"
"ctx.fillStyle='rgba(255,0,0,0.3)';ctx.fillRect(x,y,w,h);\n"
"ctx.strokeStyle='#f00';ctx.lineWidth=3;ctx.strokeRect(x,y,w,h);\n"
"ctx.fillStyle='#f00';ctx.font='bold 14px Arial';\n"
"ctx.fillText(d.label+' '+(d.confidence*100).toFixed(0)+'%',x,y>15?y-4:y+h+14);}\n"
"fetching=false;setTimeout(fetchFrame,2000);};\n"
"img.src='data:image/jpeg;base64,'+d.image;}\n"
"else{fetching=false;setTimeout(fetchFrame,2000);}\n"
"}).catch(e=>{fetching=false;setTimeout(fetchFrame,2000);});}\n"
"\n"
"function doSetup(){fetch('/api/setup',{method:'POST'});}\n"
"function doArm(){fetch('/api/arm',{method:'POST'});}\n"
"function doForce(){fetch('/api/forcearm',{method:'POST'});}\n"
"function doDisarm(){fetch('/api/disarm',{method:'POST'});}\n"
"function doMode(){var m=document.getElementById('ms').value;"
"if(m)fetch('/api/mode?m='+m,{method:'POST'});document.getElementById('ms').value='';}\n"
"function doTk(){if(!confirm('Start auto takeoff?\\nGUIDED>ARM>5m>Hover 10s>LAND'))return;"
"fetch('/api/takeoff',{method:'POST'});}\n"
"function doAb(){fetch('/api/abort',{method:'POST'});}\n"
"function toggleVF(){fetch('/api/camera/vflip');}\n"
"function toggleHF(){fetch('/api/camera/hflip');}\n"
"\n"
"['thr','yrc','prc','rrc'].forEach(function(id){\n"
"document.getElementById(id).addEventListener('input',function(){\n"
"var m={'thr':'tv','yrc':'yv','prc':'pv','rrc':'rv'};\n"
"document.getElementById(m[id]).textContent=this.value;\n"
"if(rcA)sendRc();});});\n"
"function doRcS(){rcA=true;sendRc();}\n"
"function sendRc(){\n"
"var t=document.getElementById('thr').value,y=document.getElementById('yrc').value,\n"
"p=document.getElementById('prc').value,r=document.getElementById('rrc').value;\n"
"fetch('/api/rc?r='+r+'&p='+p+'&t='+t+'&y='+y,{method:'POST'});}\n"
"function doRcX(){rcA=false;\n"
"document.getElementById('thr').value=1000;document.getElementById('tv').textContent='1000';\n"
"document.getElementById('yrc').value=1500;document.getElementById('yv').textContent='1500';\n"
"document.getElementById('prc').value=1500;document.getElementById('pv').textContent='1500';\n"
"document.getElementById('rrc').value=1500;document.getElementById('rv').textContent='1500';\n"
"fetch('/api/rc/stop',{method:'POST'});}\n"
"['yrc','prc','rrc'].forEach(function(id){\n"
"var m={'yrc':'yv','prc':'pv','rrc':'rv'};\n"
"function snap(){document.getElementById(id).value=1500;document.getElementById(m[id]).textContent='1500';if(rcA)sendRc();}\n"
"document.getElementById(id).addEventListener('mouseup',snap);\n"
"document.getElementById(id).addEventListener('touchend',snap);});\n"
"\n"
"function tc(el){el.parentElement.classList.toggle('cl');}\n"
"setInterval(update,500);update();fetchFrame();\n"
"</script></body></html>";

/* ========================== HTTP HANDLERS ================================= */

/* --- Serve HTML dashboard --- */
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, HTML_PAGE1, sizeof(HTML_PAGE1) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE2, sizeof(HTML_PAGE2) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE3, sizeof(HTML_PAGE3) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE4, sizeof(HTML_PAGE4) - 1);
    httpd_resp_send_chunk(req, HTML_PAGE5, sizeof(HTML_PAGE5) - 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* --- JSON telemetry + detection + logs --- */
static esp_err_t data_handler(httpd_req_t *req) {
    char json[2560];
    char logs_json[800] = "[";

    /* Build logs JSON array */
    if (xSemaphoreTake(log_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < log_count; i++) {
            int idx = (log_head - log_count + i + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
            if (i > 0) strcat(logs_json, ",");
            strcat(logs_json, "\"");
            char *p = log_entries[idx];
            char *d = logs_json + strlen(logs_json);
            while (*p && (d - logs_json) < 780) {
                if (*p == '"') { *d++ = '\\'; }
                if (*p == '\\' && *(p + 1) != '"') { *d++ = '\\'; }
                *d++ = *p++;
            }
            *d = '\0';
            strcat(logs_json, "\"");
        }
        xSemaphoreGive(log_mutex);
    }
    strcat(logs_json, "]");

    /* Get detection data */
    bool fv = false; float fc = 0; char fl[32] = "";
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(50))) {
        fv = latest_detection.valid;
        fc = latest_detection.confidence;
        strncpy(fl, latest_detection.label, 31);
        xSemaphoreGive(detection_mutex);
    }

    snprintf(json, sizeof(json),
        "{\"cn\":%s,\"ar\":%s,\"md\":\"%s\","
        "\"rl\":%.2f,\"pt\":%.2f,\"yw\":%.2f,"
        "\"gD\":%s,\"gx\":%d,\"gfs\":\"%s\",\"gn\":%d,\"ge\":%d,"
        "\"gla\":%ld,\"glo\":%ld,\"gal\":%ld,\"rA\":%ld,"
        "\"bh\":%s,\"bV\":%d,\"bA\":%d,\"bP\":%d,"
        "\"vH\":%s,\"vA\":%.2f,\"vG\":%.2f,\"vD\":%d,\"vC\":%.2f,\"vT\":%d,"
        "\"fv\":%s,\"fc\":%.2f,\"fl\":\"%s\","
        "\"sS\":%d,\"sH\":%d,\"sN\":\"%s\","
        "\"lg\":%s}",
        is_connected ? "true" : "false",
        is_armed ? "true" : "false",
        get_mode_name(pixhawk_custom_mode),
        current_roll, current_pitch, current_yaw,
        gps_has_data ? "true" : "false", gps_fix_type,
        mavlink_gps_fix_type_string(gps_fix_type), gps_satellites, gps_eph,
        (long)gps_lat, (long)gps_lon, (long)gps_alt, (long)global_rel_alt,
        batt_has_data ? "true" : "false", batt_voltage, batt_current, batt_remaining,
        vfr_has_data ? "true" : "false", vfr_alt, vfr_groundspeed, vfr_heading, vfr_climb, vfr_throttle,
        fv ? "true" : "false", fc, fl,
        (int)seq_state, seq_hover_remaining, seq_state_name(seq_state),
        logs_json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, json, strlen(json));

    rc_last_web_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return ESP_OK;
}

/* --- Base64 helpers --- */
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t base64_encode(const uint8_t *src, size_t src_len, char *dst, size_t dst_len) {
    size_t needed = ((src_len + 2) / 3) * 4 + 1;
    if (dst_len < needed) return 0;
    size_t i, j;
    for (i = 0, j = 0; i < src_len; i += 3, j += 4) {
        uint32_t n = ((uint32_t)src[i]) << 16;
        if (i + 1 < src_len) n |= ((uint32_t)src[i + 1]) << 8;
        if (i + 2 < src_len) n |= src[i + 2];
        dst[j] = b64_table[(n >> 18) & 0x3F];
        dst[j + 1] = b64_table[(n >> 12) & 0x3F];
        dst[j + 2] = (i + 1 < src_len) ? b64_table[(n >> 6) & 0x3F] : '=';
        dst[j + 3] = (i + 2 < src_len) ? b64_table[n & 0x3F] : '=';
    }
    dst[j] = '\0';
    return j;
}

/* --- Camera frame + detection JSON --- */
static esp_err_t frame_handler(httpd_req_t *req) {
    /*
     * Serve the last JPEG cached by the inference task instead of calling
     * esp_camera_fb_get() here.  This avoids DMA contention between the
     * inference capture and the HTTP handler when WiFi traffic is active.
     */
    uint8_t *jpg_copy = NULL;
    size_t   jpg_len  = 0;

    if (frame_cache_mutex && xSemaphoreTake(frame_cache_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (cached_jpg_buf && cached_jpg_len) {
            jpg_copy = (uint8_t *)heap_caps_malloc(cached_jpg_len, MALLOC_CAP_SPIRAM);
            if (jpg_copy) {
                memcpy(jpg_copy, cached_jpg_buf, cached_jpg_len);
                jpg_len = cached_jpg_len;
                ESP_LOGD("frame_h", "[DBG AREA 7] Serving cached frame: %u bytes", (unsigned)jpg_len);
            } else {
                ESP_LOGE("frame_h", "[DBG AREA 7] heap_caps_malloc(%u, SPIRAM) FAILED – free_psram=%u",
                         (unsigned)cached_jpg_len, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            }
        } else {
            ESP_LOGW("frame_h", "[DBG AREA 7] Cache EMPTY when HTTP requested frame (buf=%p len=%u)",
                     (void*)cached_jpg_buf, (unsigned)cached_jpg_len);
        }
        xSemaphoreGive(frame_cache_mutex);
    } else {
        ESP_LOGE("frame_h", "[DBG AREA 7] frame_cache_mutex TIMEOUT in frame_handler – inference holding lock?");
    }

    if (!jpg_copy) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No frame available yet");
        return ESP_FAIL;
    }

    size_t b64_len = ((jpg_len + 2) / 3) * 4 + 1;
    char *b64_buf = (char *)heap_caps_malloc(b64_len, MALLOC_CAP_SPIRAM);
    if (!b64_buf) {
        free(jpg_copy);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    base64_encode(jpg_copy, jpg_len, b64_buf, b64_len);
    free(jpg_copy);

    size_t json_size = b64_len + 256;
    char *json = (char *)heap_caps_malloc(json_size, MALLOC_CAP_SPIRAM);
    if (!json) {
        free(b64_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(50))) {
        snprintf(json, json_size,
                 "{\"image\":\"%s\",\"valid\":%s,\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u,"
                 "\"confidence\":%.2f,\"label\":\"%s\"}",
                 b64_buf,
                 latest_detection.valid ? "true" : "false",
                 (unsigned)latest_detection.x, (unsigned)latest_detection.y,
                 (unsigned)latest_detection.width, (unsigned)latest_detection.height,
                 latest_detection.confidence, latest_detection.label);
        xSemaphoreGive(detection_mutex);
    } else {
        snprintf(json, json_size,
                 "{\"image\":\"%s\",\"valid\":false,\"x\":0,\"y\":0,\"width\":0,\"height\":0,"
                 "\"confidence\":0,\"label\":\"\"}", b64_buf);
    }

    free(b64_buf);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t err = httpd_resp_send(req, json, strlen(json));
    free(json);
    return err;
}

/* --- Camera flip handlers --- */
static esp_err_t vflip_handler(httpd_req_t *req) {
    camera_vflip = !camera_vflip;
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_vflip(s, camera_vflip ? 1 : 0);
    add_log("Camera V-Flip: %s", camera_vflip ? "ON" : "OFF");
    char json[32];
    snprintf(json, sizeof(json), "{\"vflip\":%s}", camera_vflip ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

static esp_err_t hflip_handler(httpd_req_t *req) {
    camera_hflip = !camera_hflip;
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_hmirror(s, camera_hflip ? 1 : 0);
    add_log("Camera H-Flip: %s", camera_hflip ? "ON" : "OFF");
    char json[32];
    snprintf(json, sizeof(json), "{\"hflip\":%s}", camera_hflip ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* --- Drone control handlers --- */
static esp_err_t setup_handler(httpd_req_t *req) {
    disable_prearm_checks();
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t arm_handler(httpd_req_t *req) {
    send_param_set("DISARM_DELAY", 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_rc_override_throttle_low();
    send_arm_command(true, false);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t forcearm_handler(httpd_req_t *req) {
    send_param_set("DISARM_DELAY", 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_rc_override_throttle_low();
    send_arm_command(true, true);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t disarm_handler(httpd_req_t *req) {
    send_arm_command(false, false);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(buf, "m", val, sizeof(val)) == ESP_OK) {
            uint32_t mode = atoi(val);
            send_mode_command(mode);
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

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
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t rc_stop_handler(httpd_req_t *req) {
    rc_override_active = false;
    rc_chan1 = 0; rc_chan2 = 0; rc_chan3 = 0; rc_chan4 = 0;
    send_rc_override(0, 0, 0, 0);
    add_log("RC Override released");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t takeoff_handler(httpd_req_t *req) {
    if (seq_state != SEQ_IDLE && seq_state != SEQ_COMPLETE && seq_state != SEQ_ABORTED) {
        httpd_resp_send(req, "BUSY", 4);
        return ESP_OK;
    }
    add_log(">>> AUTO TAKEOFF SEQUENCE <<<");
    seq_abort_flag = false;
    seq_hover_remaining = seq_hover_seconds;
    seq_state = SEQ_PREFLIGHT;
    seq_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t abort_handler(httpd_req_t *req) {
    if (seq_state > SEQ_IDLE && seq_state < SEQ_COMPLETE) {
        seq_abort_flag = true;
        add_log("ABORT requested");
    } else {
        send_mode_command(9);
        add_log("LAND mode (no sequence active)");
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

/* ========================== WEB SERVER ==================================== */

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);

    if (httpd_start(&http_server, &config) == ESP_OK) {
        /* Page */
        httpd_uri_t u_root = { .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_root);

        /* Data APIs */
        httpd_uri_t u_data = { .uri = "/api/data", .method = HTTP_GET, .handler = data_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_data);
        httpd_uri_t u_frame = { .uri = "/api/frame", .method = HTTP_GET, .handler = frame_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_frame);

        /* Camera controls */
        httpd_uri_t u_vf = { .uri = "/api/camera/vflip", .method = HTTP_GET, .handler = vflip_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_vf);
        httpd_uri_t u_hf = { .uri = "/api/camera/hflip", .method = HTTP_GET, .handler = hflip_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_hf);

        /* Drone controls */
        httpd_uri_t u_setup = { .uri = "/api/setup", .method = HTTP_POST, .handler = setup_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_setup);
        httpd_uri_t u_arm = { .uri = "/api/arm", .method = HTTP_POST, .handler = arm_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_arm);
        httpd_uri_t u_frc = { .uri = "/api/forcearm", .method = HTTP_POST, .handler = forcearm_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_frc);
        httpd_uri_t u_dis = { .uri = "/api/disarm", .method = HTTP_POST, .handler = disarm_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_dis);
        httpd_uri_t u_mode = { .uri = "/api/mode", .method = HTTP_POST, .handler = mode_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_mode);

        /* RC Override */
        httpd_uri_t u_rc = { .uri = "/api/rc", .method = HTTP_POST, .handler = rc_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_rc);
        httpd_uri_t u_rcx = { .uri = "/api/rc/stop", .method = HTTP_POST, .handler = rc_stop_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_rcx);

        /* Auto-flight */
        httpd_uri_t u_tk = { .uri = "/api/takeoff", .method = HTTP_POST, .handler = takeoff_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_tk);
        httpd_uri_t u_ab = { .uri = "/api/abort", .method = HTTP_POST, .handler = abort_handler, .user_ctx = NULL };
        httpd_register_uri_handler(http_server, &u_ab);

        add_log("[Web] Server started (14 endpoints)");
        return http_server;
    }

    ESP_LOGE(TAG, "Failed to start web server");
    return NULL;
}

/* ========================== FREERTOS TASKS ================================ */

/* --- Edge Impulse serial AT commands --- */
void serial_task(void *pvParameters) {
    while (1) {
        char data = ei_get_serial_byte();
        while (data != 0xFF) {
            at->handle(data);
            data = ei_get_serial_byte();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); /* yield to prevent watchdog timeout */
    }
}

/* --- LED indicator task --- */
void led_task(void *pvParameters) {
    bool led_state = false;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_rom_gpio_pad_select_gpio(INDICATOR_LED_PIN);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    gpio_pad_select_gpio(INDICATOR_LED_PIN);
#endif
    gpio_set_direction(INDICATOR_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(INDICATOR_LED_PIN, LED_ACTIVE_LOW ? 1 : 0); /* OFF */

    /* LED helper: respects active-high vs active-low boards */
    #define LED_ON_LEVEL   (LED_ACTIVE_LOW ? 0 : 1)
    #define LED_OFF_LEVEL  (LED_ACTIVE_LOW ? 1 : 0)

    while (1) {
        led_state = !led_state;

        /* Blink fast if fire detected, slow otherwise */
        bool fire = false;
        if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(10))) {
            fire = latest_detection.valid && latest_detection.confidence > 0.5f;
            xSemaphoreGive(detection_mutex);
        }

        if (fire) {
            gpio_set_level(INDICATOR_LED_PIN, led_state ? LED_ON_LEVEL : LED_OFF_LEVEL);
        } else if (is_connected) {
            gpio_set_level(INDICATOR_LED_PIN, led_state ? LED_ON_LEVEL : LED_OFF_LEVEL);
        } else {
            gpio_set_level(INDICATOR_LED_PIN, LED_OFF_LEVEL);
        }

        vTaskDelay(fire ? 200 / portTICK_PERIOD_MS : 1000 / portTICK_PERIOD_MS);
    }
}

/* --- Web server task --- */
void webserver_task(void *pvParameters) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    start_webserver();
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/* --- Edge Impulse inference task --- */
void inference_task(void *pvParameters) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    add_log("[AI] Starting fire detection inference");
    ei_printf("\r\n=== Drone Fire Detection System Ready ===\r\n");
    ei_printf("Web: http://192.168.4.1  WiFi: %s / %s\r\n", WIFI_SSID, WIFI_PASS);

    extern void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed);
    ei_start_impulse(true, false, false);

    add_log("[AI] Inference stopped unexpectedly!");
    while (1) { vTaskDelay(10000 / portTICK_PERIOD_MS); }
}

/* --- MAVLink RX task: parse incoming bytes from Pixhawk --- */
static void mavlink_rx_task(void *pvParameters) {
    uint8_t data[128];
    ESP_LOGI(TAG, "MAVLink RX task started");

    while (1) {
        int len = uart_read_bytes(MAV_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t result = mavlink_parse_char(0, data[i], &mav_msg, &mav_status);
                if (result == MAVLINK_FRAMING_OK) {
                    process_mavlink_message(&mav_msg);
                }
            }
        }

        /* Connection timeout */
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (is_connected && (now - last_heartbeat_time > 3000)) {
            is_connected = false;
            add_log("Pixhawk connection lost!");
        }
    }
}

/* --- MAVLink heartbeat task: send heartbeat, RC override, auto-sequence --- */
static void mavlink_heartbeat_task(void *pvParameters) {
    ESP_LOGI(TAG, "Heartbeat task started (sysid=%d)", COMPANION_SYSID);

    while (1) {
        send_heartbeat();

        /* Safety: auto-release RC if web client disconnected */
        if (rc_override_active) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((now - rc_last_web_time) > RC_OVERRIDE_TIMEOUT_MS) {
                rc_override_active = false;
                rc_chan1 = 0; rc_chan2 = 0; rc_chan3 = 0; rc_chan4 = 0;
                send_rc_override(0, 0, 0, 0);
                add_log("RC SAFETY: auto-released (no web client)");
            } else {
                send_rc_override(rc_chan1, rc_chan2, rc_chan3, rc_chan4);
            }
        }

        /* Tick auto-flight state machine at ~4 Hz */
        auto_sequence_tick();

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ========================== APP_MAIN ====================================== */

extern "C" int app_main() {
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  Drone Fire Detection + Pixhawk Control ");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Companion SysID: %d  Pixhawk SysID: %d", COMPANION_SYSID, PIXHAWK_SYSID);

    /* NVS for WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create mutexes */
    detection_mutex = xSemaphoreCreateMutex();
    uart_mutex = xSemaphoreCreateMutex();
    log_mutex = xSemaphoreCreateMutex();
    frame_cache_mutex = xSemaphoreCreateMutex();
    camera_ready_sem  = xSemaphoreCreateBinary();

    if (!detection_mutex || !uart_mutex || !log_mutex || !frame_cache_mutex || !camera_ready_sem) {
        ESP_LOGE(TAG, "Failed to create mutexes/semaphores");
        return -1;
    }

    add_log("[System] Starting...");

    /* Initialize MAVLink UART */
    mavlink_status_init(&mav_status);
    mav_uart_init();

    /* Initialize WiFi AP */
    wifi_init_softap();

    /* Initialize Edge Impulse */
    dev = static_cast<EiDeviceESP32 *>(EiDeviceESP32::get_device());
    ei_printf("Edge Impulse SDK initialized. Compiled %s %s\r\n", __DATE__, __TIME__);
    add_log("[EI] SDK initialized");

    /* Inertial sensor (LIS3DHTR) not present on this board – skip init
     * to avoid the noisy i2c_set_pin / "Failed to connect" errors. */
    // if (ei_inertial_init() == false) {
    //     ei_printf("Inertial sensor init failed\r\n");
    // }
    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor init failed\r\n");
    }

    at = ei_at_init(dev);
    dev->set_state(eiStateFinished);
    add_log("[System] All systems initialized");

    /* Create FreeRTOS tasks 
     * IMPORTANT: Pin heavy tasks to Core 1 (APP_CPU). WiFi & LwIP run on Core 0.
     * Splitting them prevents severe DMA/IRQ starvation that causes camera crash. 
     */
    /*
     * Core Assignment:
     *  Core 0: Wi-Fi (prio 23), lwIP, webserver_task, mavlink tasks
     *          HTTP/TCP work co-located with Wi-Fi to share the same bus context.
     *
     *  Core 1: inference_task ONLY (+ camera ISR auto-registered here)
     *          esp_camera_init() is called from inference_task, which causes the
     *          LCD_CAM DMA interrupt to be registered to Core 1.  Keeping inference
     *          on Core 1 ensures Wi-Fi's block-ACK bursts on Core 0 can NEVER
     *          starve the camera interrupt, preventing the 4s 'Failed to get the
     *          frame on time' timeout.
     */
    /* Core 1: camera_pump_task (prio 15) + inference_task (prio 4)
     *   camera_pump_task owns ALL hardware fb_get/fb_return calls.
     *   Its high priority guarantees DMA frames are serviced instantly,
     *   even during the 620ms inference computation window.
     *   inference_task blocks on camera_ready_sem (never touches hw).
     * Core 0: Wi-Fi (prio 23) + all network/serial tasks. */
    xTaskCreatePinnedToCore(camera_pump_task,       "cam_pump", 4096,  NULL, 15, NULL, 1);
    xTaskCreatePinnedToCore(inference_task,         "infer",    16384, NULL, 4,  NULL, 1);
    xTaskCreatePinnedToCore(serial_task,            "serial",   4096,  NULL, 5,  NULL, 0);
    xTaskCreatePinnedToCore(mavlink_rx_task,        "mav_rx",   4096,  NULL, 7,  NULL, 0);
    xTaskCreatePinnedToCore(mavlink_heartbeat_task, "mav_hb",   4096,  NULL, 3,  NULL, 0);
    xTaskCreatePinnedToCore(webserver_task,         "webserv",  8192,  NULL, 2,  NULL, 0);
    xTaskCreatePinnedToCore(led_task,               "led",      2048,  NULL, 1,  NULL, 0);

    add_log("[System] 6 tasks created, scheduler running");

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "WiFi: %s / %s", WIFI_SSID, WIFI_PASS);
    ESP_LOGI(TAG, "Web:  http://192.168.4.1");
    ESP_LOGI(TAG, "MAV:  TX=%d RX=%d @ %d baud", MAV_TX_PIN, MAV_RX_PIN, MAV_BAUD_RATE);
    ESP_LOGI(TAG, "=========================================");

    vTaskDelete(NULL);
    return 0;
}
