/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"

#include <stdio.h>
#include <string.h>

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
#include "img_converters.h"

#include "ei_device_espressif_esp32.h"

#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_run_impulse.h"

#include "ei_analogsensor.h"
#include "ei_inertial_sensor.h"

#define RED_LED_PIN GPIO_NUM_21
#define WHITE_LED_PIN GPIO_NUM_22
#define BUZZER_PIN GPIO_NUM_13
#define ACK_BUTTON_PIN GPIO_NUM_14

#define WIFI_SSID "ESP32-FireDetect"
#define WIFI_PASS "fire12345"
#define WIFI_CHANNEL 1
#define MAX_CONNECTIONS 4

static const char *TAG = "FireDetect";

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceESP32::get_device());
static ATServer *at;

/* Private variables ------------------------------------------------------- */
static EiDeviceESP32* dev = NULL;
static httpd_handle_t server = NULL;
static SemaphoreHandle_t detection_mutex = NULL;

/* Shared detection data */
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
static char log_buffer[4096] = {0};
static size_t log_index = 0;

/* Alarm state */
static bool alarm_active = false;
static bool alarm_acknowledged = false;

/* Detection mode: 0 = Forest, 1 = Drone */
static int detection_mode = 0;

/* Camera settings */
static bool camera_vflip = false;
static bool camera_hflip = false;

/* Drone mode buzzer auto-stop */
static uint32_t drone_buzzer_start_time = 0;
static bool drone_buzzer_running = false;
#define DRONE_BUZZER_DURATION_MS 15000

/* Button interrupt */
static volatile bool button_pressed = false;
static volatile uint32_t last_button_press_time = 0;

/* Button ISR Handler */
static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t now = xTaskGetTickCountFromISR();
    /* Debounce: ignore if pressed within last 200ms */
    if ((now - last_button_press_time) > pdMS_TO_TICKS(200)) {
        last_button_press_time = now;
        button_pressed = true;
    }
}

/* FreeRTOS Task Declarations ---------------------------------------------- */
void serial_task(void *pvParameters);
void led_task(void *pvParameters);
void webserver_task(void *pvParameters);
void inference_task(void *pvParameters);

/* Helper functions -------------------------------------------------------- */

void add_log(const char* message) {
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(50))) {
        size_t msg_len = strlen(message);
        if (log_index + msg_len + 2 < sizeof(log_buffer)) {
            strcpy(&log_buffer[log_index], message);
            log_index += msg_len;
            log_buffer[log_index++] = '\n';
            log_buffer[log_index] = '\0';
        } else {
            // Buffer full, reset
            log_index = 0;
            log_buffer[0] = '\0';
        }
        xSemaphoreGive(detection_mutex);
    }
    // If mutex not available, silently skip - don't block
}

/* Update detection data from inference results */
void update_detection_data(uint32_t x, uint32_t y, uint32_t width, uint32_t height, float confidence, const char* label) {
    /* Debug: Always print when this function is called */
    ei_printf("[WiFi Update] Sending data: %s (%.2f) at (%u,%u) size=%ux%u\r\n",
              label, confidence, (unsigned int)x, (unsigned int)y, 
              (unsigned int)width, (unsigned int)height);
    
    bool trigger_alarm_log = false;
    
    if (detection_mutex && xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(100))) {
        latest_detection.x = x;
        latest_detection.y = y;
        latest_detection.width = width;
        latest_detection.height = height;
        latest_detection.confidence = confidence;
        strncpy(latest_detection.label, label, sizeof(latest_detection.label) - 1);
        latest_detection.label[sizeof(latest_detection.label) - 1] = '\0';
        latest_detection.valid = (confidence > 0.0f);
        
        ei_printf("[WiFi Update] Updated: valid=%d, label=%s, conf=%.2f\r\n", 
                  latest_detection.valid, latest_detection.label, latest_detection.confidence);
        
        /* Trigger alarm if fire detected with high confidence */
        if (confidence > 0.5f) {
            if (!alarm_active && alarm_acknowledged) {
                /* Reset acknowledgment for new fire after previous was cleared */
                alarm_acknowledged = false;
            }
            if (!alarm_acknowledged) {
                alarm_active = true;
                ei_printf("[ALARM] Fire detected! Alarm activated!\r\n");
                trigger_alarm_log = true;
            }
        } else if (confidence == 0.0f && alarm_acknowledged) {
            /* Reset alarm state when no fire and previous alarm was acknowledged */
            alarm_acknowledged = false;
            alarm_active = false;
        }
        
        xSemaphoreGive(detection_mutex);
    } else {
        ei_printf("[WiFi Update] ERROR: Failed to acquire mutex!\r\n");
    }
    
    /* Add log AFTER releasing mutex to avoid deadlock */
    if (trigger_alarm_log) {
        add_log("[ALARM] Fire detected!");
    }
}

/* WiFi AP Mode Setup ------------------------------------------------------ */

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        add_log("[WiFi] Client connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Station disconnected");
        add_log("[WiFi] Client disconnected");
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                                ESP_EVENT_ANY_ID,
                                                &wifi_event_handler,
                                                NULL));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_PASS);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    wifi_config.ap.channel = WIFI_CHANNEL;
    wifi_config.ap.max_connection = MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s Password:%s", WIFI_SSID, WIFI_PASS);
    ESP_LOGI(TAG, "Connect to http://192.168.4.1");
    add_log("[WiFi] AP Mode started - Connect to ESP32-FireDetect");
    add_log("[WiFi] Web interface: http://192.168.4.1");
}

/* Web Server Handlers ----------------------------------------------------- */

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char* html_page = 
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<meta charset='UTF-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>Fire Detection</title>\n"
"<style>\n"
"body{font-family:Arial;background:#222;color:#fff;padding:20px;margin:0}\n"
"h1{color:#f44;text-align:center}\n"
".box{background:#333;padding:15px;margin:10px 0;border-radius:8px}\n"
".video-container{position:relative;width:640px;max-width:100%;margin:10px auto}\n"
"#frame{width:100%;height:auto;border:2px solid #555;display:block}\n"
"canvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}\n"
".logs{background:#111;padding:10px;height:150px;overflow-y:scroll;font-family:monospace;font-size:12px}\n"
".controls{display:flex;flex-wrap:wrap;gap:10px;margin:10px 0}\n"
".btn{padding:10px 15px;border:none;border-radius:5px;cursor:pointer;font-size:14px;font-weight:bold}\n"
".btn-blue{background:#2196F3;color:#fff}.btn-blue:hover{background:#1976D2}\n"
".btn-green{background:#4CAF50;color:#fff}.btn-green:hover{background:#388E3C}\n"
".btn-orange{background:#FF9800;color:#fff}.btn-orange:hover{background:#F57C00}\n"
".btn-active{box-shadow:0 0 10px #fff}\n"
".mode-box{display:flex;gap:10px;align-items:center}\n"
".mode-indicator{padding:5px 15px;border-radius:5px;font-weight:bold}\n"
".mode-forest{background:#4CAF50;color:#fff}\n"
".mode-drone{background:#2196F3;color:#fff}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<h1>Fire Detection Monitor</h1>\n"
"<div class='box'>\n"
"<h3>Detection Mode</h3>\n"
"<div class='mode-box'>\n"
"<button class='btn btn-green' id='btnForest' onclick='setMode(0)'>Forest Mode</button>\n"
"<button class='btn btn-blue' id='btnDrone' onclick='setMode(1)'>Drone Mode</button>\n"
"<span class='mode-indicator' id='modeIndicator'>Forest</span>\n"
"</div>\n"
"<p id='modeDesc' style='margin:10px 0;font-size:13px;color:#aaa'>Forest: Button acknowledges alarm | Drone: Buzzer auto-stops after 15s</p>\n"
"</div>\n"
"<div class='box'>\n"
"<h3>Camera Controls</h3>\n"
"<div class='controls'>\n"
"<button class='btn btn-orange' id='btnVFlip' onclick='toggleVFlip()'>Flip Vertical</button>\n"
"<button class='btn btn-blue' id='btnHFlip' onclick='toggleHFlip()'>Flip Horizontal</button>\n"
"</div>\n"
"</div>\n"
"<div class='box'>\n"
"<div class='video-container'>\n"
"<img id='frame'>\n"
"<canvas id='c'></canvas>\n"
"</div>\n"
"</div>\n"
"<div class='box' id='info'>Waiting for detection...</div>\n"
"<div class='box'><h3>Logs</h3><div class='logs' id='logs'></div></div>\n"
"<script>\n"
"let c=document.getElementById('c'),ctx=c.getContext('2d'),img=document.getElementById('frame');\n"
"let lastDetection=null,fetching=false,currentMode=0,vflip=false,hflip=false;\n"
"function updateModeUI(mode){\n"
"currentMode=mode;\n"
"document.getElementById('btnForest').classList.toggle('btn-active',mode==0);\n"
"document.getElementById('btnDrone').classList.toggle('btn-active',mode==1);\n"
"let ind=document.getElementById('modeIndicator');\n"
"ind.textContent=mode==0?'Forest':'Drone';\n"
"ind.className='mode-indicator '+(mode==0?'mode-forest':'mode-drone');\n"
"}\n"
"function setMode(m){fetch('/api/mode?mode='+m).then(r=>r.json()).then(d=>updateModeUI(d.mode)).catch(e=>console.error(e));}\n"
"function toggleVFlip(){fetch('/api/camera/vflip').then(r=>r.json()).then(d=>{vflip=d.vflip;document.getElementById('btnVFlip').classList.toggle('btn-active',vflip);}).catch(e=>console.error(e));}\n"
"function toggleHFlip(){fetch('/api/camera/hflip').then(r=>r.json()).then(d=>{hflip=d.hflip;document.getElementById('btnHFlip').classList.toggle('btn-active',hflip);}).catch(e=>console.error(e));}\n"
"function drawOverlay(){\n"
"if(!c.width||!c.height)return;\n"
"ctx.clearRect(0,0,c.width,c.height);\n"
"if(lastDetection && lastDetection.valid && lastDetection.confidence>0){\n"
"let scaleX=c.width/96,scaleY=c.height/96;\n"
"let x=lastDetection.x*scaleX,y=lastDetection.y*scaleY,w=lastDetection.width*scaleX,h=lastDetection.height*scaleY;\n"
"ctx.fillStyle='rgba(255,0,0,0.3)';ctx.fillRect(x,y,w,h);\n"
"ctx.strokeStyle='#f00';ctx.lineWidth=3;ctx.strokeRect(x,y,w,h);\n"
"ctx.fillStyle='#f00';ctx.font='bold 16px Arial';\n"
"ctx.fillText(lastDetection.label+' '+(lastDetection.confidence*100).toFixed(0)+'%',x,y>20?y-5:y+h+15);\n"
"ctx.fillStyle='rgba(0,0,0,0.7)';ctx.fillRect(5,c.height-60,200,55);\n"
"ctx.fillStyle='#0f0';ctx.font='12px monospace';\n"
"ctx.fillText('Detection Coords:',10,c.height-45);\n"
"ctx.fillText('X: '+lastDetection.x+' Y: '+lastDetection.y,10,c.height-30);\n"
"ctx.fillText('W: '+lastDetection.width+' H: '+lastDetection.height,10,c.height-15);\n"
"document.getElementById('info').innerHTML='<h3 style=\"color:#f00\">FIRE DETECTED</h3>Label: '+lastDetection.label+'<br>Confidence: '+(lastDetection.confidence*100).toFixed(1)+'%<br>Position: ('+lastDetection.x+','+lastDetection.y+') Size: '+lastDetection.width+'x'+lastDetection.height;\n"
"}else{\n"
"ctx.fillStyle='rgba(0,0,0,0.7)';ctx.fillRect(5,c.height-30,150,25);\n"
"ctx.fillStyle='#0f0';ctx.font='12px monospace';\n"
"ctx.fillText('No detection',10,c.height-12);\n"
"document.getElementById('info').innerHTML='<span style=\"color:#0f0\">Monitoring... (no fire)</span>';\n"
"}\n"
"}\n"
"function fetchFrame(){\n"
"if(fetching)return;\n"
"fetching=true;\n"
"fetch('/api/frame',{cache:'no-store'}).then(r=>r.json()).then(d=>{\n"
"if(d.image){img.onload=function(){c.width=img.width;c.height=img.height;drawOverlay();fetching=false;setTimeout(fetchFrame,100);};img.src='data:image/jpeg;base64,'+d.image;}\n"
"else{fetching=false;setTimeout(fetchFrame,100);}\n"
"lastDetection={valid:d.valid,x:d.x,y:d.y,width:d.width,height:d.height,confidence:d.confidence,label:d.label};\n"
"}).catch(e=>{console.error('Frame error:',e);fetching=false;setTimeout(fetchFrame,500);});\n"
"}\n"
"fetch('/api/settings').then(r=>r.json()).then(d=>{updateModeUI(d.mode);vflip=d.vflip;hflip=d.hflip;document.getElementById('btnVFlip').classList.toggle('btn-active',vflip);document.getElementById('btnHFlip').classList.toggle('btn-active',hflip);}).catch(e=>{});\n"
"fetchFrame();\n"
"setInterval(()=>{fetch('/api/logs',{cache:'no-store'}).then(r=>r.text()).then(l=>{let lines=l.split('\\n').filter(x=>x.trim()).slice(-15);document.getElementById('logs').innerHTML=lines.join('<br>')||'No logs yet...';}).catch(e=>{});},2000);\n"
"</script>\n"
"</body>\n"
"</html>";

esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_detection_handler(httpd_req_t *req)
{
    char json[256];
    
    ei_printf("[API] Detection request received from client\r\n");
    
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(100))) {
        snprintf(json, sizeof(json),
                 "{\"valid\":%s,\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u,\"confidence\":%.2f,\"label\":\"%s\"}",
                 latest_detection.valid ? "true" : "false",
                 (unsigned int)latest_detection.x,
                 (unsigned int)latest_detection.y,
                 (unsigned int)latest_detection.width,
                 (unsigned int)latest_detection.height,
                 latest_detection.confidence,
                 latest_detection.label);
        
        /* Debug output */
        ei_printf("[API] Sending response: %s\r\n", json);
        
        xSemaphoreGive(detection_mutex);
    } else {
        strcpy(json, "{\"valid\":false}");
        ei_printf("[API] Detection request failed - mutex timeout\r\n");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Connection", "close");  /* Close immediately to free socket */
    
    esp_err_t err = httpd_resp_send(req, json, strlen(json));
    
    if (err != ESP_OK) {
        ei_printf("[API] Failed to send detection response: %d\r\n", err);
    } else {
        ei_printf("[API] Detection response sent successfully\r\n");
    }
    
    return ESP_OK;
}

esp_err_t api_logs_handler(httpd_req_t *req)
{
    ei_printf("[API] Logs request received from client\r\n");
    
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(100))) {
        size_t log_len = strlen(log_buffer);
        ei_printf("[API] Logs buffer size: %u bytes\r\n", (unsigned int)log_len);
        
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
        httpd_resp_set_hdr(req, "Connection", "close");  /* Close immediately to free socket */
        
        esp_err_t err = httpd_resp_send(req, log_buffer, log_len);
        xSemaphoreGive(detection_mutex);
        
        if (err != ESP_OK) {
            ei_printf("[API] Failed to send logs: %d\r\n", err);
        } else {
            ei_printf("[API] Logs sent successfully (%u bytes)\r\n", (unsigned int)log_len);
        }
    } else {
        ei_printf("[API] Logs request failed - mutex timeout\r\n");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mutex timeout");
    }
    return ESP_OK;
}

/* Camera control API - flip vertical */
esp_err_t api_camera_vflip_handler(httpd_req_t *req)
{
    camera_vflip = !camera_vflip;
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, camera_vflip ? 1 : 0);
        ei_printf("[Camera] VFlip set to: %d\r\n", camera_vflip);
        add_log(camera_vflip ? "[Camera] Vertical flip: ON" : "[Camera] Vertical flip: OFF");
    }
    
    char json[64];
    snprintf(json, sizeof(json), "{\"vflip\":%s}", camera_vflip ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* Camera control API - flip horizontal */
esp_err_t api_camera_hflip_handler(httpd_req_t *req)
{
    camera_hflip = !camera_hflip;
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, camera_hflip ? 1 : 0);
        ei_printf("[Camera] HFlip set to: %d\r\n", camera_hflip);
        add_log(camera_hflip ? "[Camera] Horizontal flip: ON" : "[Camera] Horizontal flip: OFF");
    }
    
    char json[64];
    snprintf(json, sizeof(json), "{\"hflip\":%s}", camera_hflip ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* Mode control API - Forest/Drone */
esp_err_t api_mode_handler(httpd_req_t *req)
{
    char buf[32];
    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret == ESP_OK) {
        char param[8];
        if (httpd_query_key_value(buf, "mode", param, sizeof(param)) == ESP_OK) {
            int mode = atoi(param);
            if (mode == 0 || mode == 1) {
                detection_mode = mode;
                const char* mode_names[] = {"Forest", "Drone"};
                ei_printf("[Mode] Switched to: %s Detection\\r\\n", mode_names[mode]);
                char log_msg[64];
                snprintf(log_msg, sizeof(log_msg), "[Mode] Switched to %s Detection", mode_names[mode]);
                add_log(log_msg);
                
                /* Reset alarm state on mode change */
                alarm_active = false;
                alarm_acknowledged = false;
                drone_buzzer_running = false;
                gpio_set_level(BUZZER_PIN, 0);
            }
        }
    }
    
    char json[64];
    snprintf(json, sizeof(json), "{\"mode\":%d,\"modeName\":\"%s\"}", detection_mode, detection_mode == 0 ? "Forest" : "Drone");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* Get current settings API */
esp_err_t api_settings_handler(httpd_req_t *req)
{
    char json[128];
    snprintf(json, sizeof(json), 
             "{\"mode\":%d,\"modeName\":\"%s\",\"vflip\":%s,\"hflip\":%s}",
             detection_mode,
             detection_mode == 0 ? "Forest" : "Drone",
             camera_vflip ? "true" : "false",
             camera_hflip ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/* Base64 encoding table */
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t base64_encode(const uint8_t *src, size_t src_len, char *dst, size_t dst_len) {
    size_t i, j;
    size_t needed = ((src_len + 2) / 3) * 4 + 1;
    if (dst_len < needed) return 0;
    
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

/* Combined frame + detection endpoint */
esp_err_t api_frame_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera capture failed");
        return ESP_FAIL;
    }
    
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool need_free = false;
    
    if (fb->format != PIXFORMAT_JPEG) {
        if (!frame2jpg(fb, 80, &jpg_buf, &jpg_len)) {
            esp_camera_fb_return(fb);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JPEG conversion failed");
            return ESP_FAIL;
        }
        need_free = true;
    } else {
        jpg_buf = fb->buf;
        jpg_len = fb->len;
    }
    
    /* Base64 encode the image */
    size_t b64_len = ((jpg_len + 2) / 3) * 4 + 1;
    char *b64_buf = (char *)malloc(b64_len);
    if (!b64_buf) {
        if (need_free) free(jpg_buf);
        esp_camera_fb_return(fb);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    
    base64_encode(jpg_buf, jpg_len, b64_buf, b64_len);
    
    if (need_free) free(jpg_buf);
    esp_camera_fb_return(fb);
    
    /* Build JSON response with image and detection data */
    size_t json_size = b64_len + 256;
    char *json = (char *)malloc(json_size);
    if (!json) {
        free(b64_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(50))) {
        snprintf(json, json_size,
                 "{\"image\":\"%s\",\"valid\":%s,\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u,\"confidence\":%.2f,\"label\":\"%s\"}",
                 b64_buf,
                 latest_detection.valid ? "true" : "false",
                 (unsigned int)latest_detection.x,
                 (unsigned int)latest_detection.y,
                 (unsigned int)latest_detection.width,
                 (unsigned int)latest_detection.height,
                 latest_detection.confidence,
                 latest_detection.label);
        xSemaphoreGive(detection_mutex);
    } else {
        snprintf(json, json_size, "{\"image\":\"%s\",\"valid\":false,\"x\":0,\"y\":0,\"width\":0,\"height\":0,\"confidence\":0,\"label\":\"\"}", b64_buf);
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

esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[64];
    
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
        
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        
        esp_camera_fb_return(fb);
        
        if(res != ESP_OK){
            break;
        }
        
        /* Small delay to prevent overwhelming the connection */
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
    
    return res;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 16;  /* Increased for camera/mode control APIs */
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;   /* LWIP_MAX_SOCKETS=10 minus 3 internal = 7 */
    config.recv_wait_timeout = 5;  /* Shorter timeout for better responsiveness */
    config.send_wait_timeout = 5;

    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t api_detection = {
            .uri = "/api/detection",
            .method = HTTP_GET,
            .handler = api_detection_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_detection);

        httpd_uri_t api_logs = {
            .uri = "/api/logs",
            .method = HTTP_GET,
            .handler = api_logs_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_logs);

        httpd_uri_t api_frame = {
            .uri = "/api/frame",
            .method = HTTP_GET,
            .handler = api_frame_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_frame);

        httpd_uri_t api_vflip = {
            .uri = "/api/camera/vflip",
            .method = HTTP_GET,
            .handler = api_camera_vflip_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_vflip);

        httpd_uri_t api_hflip = {
            .uri = "/api/camera/hflip",
            .method = HTTP_GET,
            .handler = api_camera_hflip_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_hflip);

        httpd_uri_t api_mode = {
            .uri = "/api/mode",
            .method = HTTP_GET,
            .handler = api_mode_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_mode);

        httpd_uri_t api_settings = {
            .uri = "/api/settings",
            .method = HTTP_GET,
            .handler = api_settings_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_settings);

        httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = stream_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        add_log("[WebServer] HTTP server started successfully");
        add_log("[WebServer] Frame+detection API at /api/frame");
        return server;
    }

    ESP_LOGE(TAG, "Failed to start web server");
    return NULL;
}

/* Public functions -------------------------------------------------------- */

void setup_led() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_rom_gpio_pad_select_gpio(RED_LED_PIN);
    esp_rom_gpio_pad_select_gpio(WHITE_LED_PIN);
    esp_rom_gpio_pad_select_gpio(BUZZER_PIN);
    esp_rom_gpio_pad_select_gpio(ACK_BUTTON_PIN);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
    gpio_pad_select_gpio(RED_LED_PIN);
    gpio_pad_select_gpio(WHITE_LED_PIN);
    gpio_pad_select_gpio(BUZZER_PIN);
    gpio_pad_select_gpio(ACK_BUTTON_PIN);
#endif
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(WHITE_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ACK_BUTTON_PIN, GPIO_MODE_INPUT);
    
    /* Enable pull-up for button */
    gpio_set_pull_mode(ACK_BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    /* Configure button interrupt - trigger on falling edge (button press) */
    gpio_set_intr_type(ACK_BUTTON_PIN, GPIO_INTR_NEGEDGE);
    
    /* Install GPIO ISR service (may already be installed by camera) */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err == ESP_OK) {
        ESP_LOGI(TAG, "GPIO ISR service installed");
    } else if (isr_err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "GPIO ISR service already installed (by camera)");
    }
    
    /* Attach interrupt handler to button pin */
    gpio_isr_handler_add(ACK_BUTTON_PIN, button_isr_handler, (void*) ACK_BUTTON_PIN);
    
    /* Initialize outputs to OFF */
    gpio_set_level(RED_LED_PIN, 0);
    gpio_set_level(WHITE_LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
    
    ESP_LOGI(TAG, "Button interrupt configured on GPIO %d", ACK_BUTTON_PIN);
}

/* FreeRTOS Task Implementations ------------------------------------------- */

void serial_task(void *pvParameters)
{
    while(1){
        /* handle command coming from uart */
        char data = ei_get_serial_byte();

        while (data != 0xFF) {
            at->handle(data);
            data = ei_get_serial_byte();
        }
        
        /* Small delay to prevent task starvation */
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void led_task(void *pvParameters)
{
    bool led_state = false;
    bool buzzer_state = false;
    uint32_t buzzer_toggle_count = 0;
    
    while(1){
        /* Check interrupt flag for button press - only for Forest mode */
        if (button_pressed) {
            button_pressed = false; // Clear flag
            
            if (detection_mode == 0 && alarm_active) {  /* Forest mode only */
                alarm_acknowledged = true;
                alarm_active = false;
                gpio_set_level(BUZZER_PIN, 0);
                ei_printf("[ALARM] Acknowledged by button interrupt (Forest Mode)\r\n");
                add_log("[ALARM] Fire alarm acknowledged");
            } else if (detection_mode == 1) {
                ei_printf("[INFO] Button disabled in Drone mode\r\n");
            }
        }
        
        /* Toggle heartbeat LED */
        led_state = !led_state;
        gpio_set_level(WHITE_LED_PIN, led_state);
        
        /* Handle alarm buzzer based on mode */
        if (alarm_active && !alarm_acknowledged) {
            if (detection_mode == 1) {  /* Drone mode - 15s auto-stop */
                if (!drone_buzzer_running) {
                    drone_buzzer_start_time = xTaskGetTickCount();
                    drone_buzzer_running = true;
                    ei_printf("[ALARM] Drone mode: Buzzer started for 15 seconds\r\n");
                    add_log("[ALARM] Drone mode: 15s buzzer started");
                }
                
                uint32_t elapsed = (xTaskGetTickCount() - drone_buzzer_start_time) * portTICK_PERIOD_MS;
                if (elapsed < DRONE_BUZZER_DURATION_MS) {
                    buzzer_state = !buzzer_state;
                    gpio_set_level(BUZZER_PIN, buzzer_state);
                } else {
                    /* 15 seconds elapsed - stop buzzer */
                    gpio_set_level(BUZZER_PIN, 0);
                    buzzer_state = false;
                    alarm_acknowledged = true;  /* Auto-acknowledge */
                    alarm_active = false;
                    drone_buzzer_running = false;
                    ei_printf("[ALARM] Drone mode: Buzzer auto-stopped after 15s\r\n");
                    add_log("[ALARM] Drone mode: Buzzer auto-stopped");
                }
            } else {  /* Forest mode - continuous until button press */
                buzzer_state = !buzzer_state;
                gpio_set_level(BUZZER_PIN, buzzer_state);
                buzzer_toggle_count++;
                
                if (buzzer_toggle_count % 10 == 0) {
                    ei_printf("[ALARM] Forest mode: Press button to acknowledge\r\n");
                }
            }
        } else {
            /* Turn off buzzer if alarm is not active or acknowledged */
            gpio_set_level(BUZZER_PIN, 0);
            buzzer_state = false;
            drone_buzzer_running = false;
        }
        
        /* Red LED for current fire detection */
        if (xSemaphoreTake(detection_mutex, pdMS_TO_TICKS(10))) {
            if (latest_detection.valid && latest_detection.confidence > 0.5) {
                gpio_set_level(RED_LED_PIN, 1);
            } else {
                gpio_set_level(RED_LED_PIN, 0);
            }
            xSemaphoreGive(detection_mutex);
        }
        
        /* Delay 500ms for faster buzzer beep */
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void webserver_task(void *pvParameters)
{
    /* Wait for WiFi to be ready */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    /* Start web server */
    start_webserver();
    
    /* Keep task alive */
    while(1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void inference_task(void *pvParameters)
{
    /* Wait for system initialization - camera and sensors need time */
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    add_log("[Inference] Auto-starting continuous fire detection");
    ei_printf("\r\n=== Fire Detection System Ready ===\r\n");
    ei_printf("Auto-starting continuous inference...\r\n");
    ei_printf("Web interface: http://192.168.4.1\r\n");
    ei_printf("WiFi: SSID=%s, Password=%s\r\n", WIFI_SSID, WIFI_PASS);
    ei_printf("Press button on GPIO 14 to acknowledge fire alarm\r\n\r\n");
    
    /* Start continuous inference - this function contains its own loop */
    extern void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed);
    ei_start_impulse(true, false, false);
    
    /* This line should never be reached unless inference stops */
    add_log("[Inference] Stopped unexpectedly");
    ei_printf("[ERROR] Inference stopped! Restart system.\r\n");
    
    while(1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

extern "C" int app_main()
{
    /* Initialize NVS for WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create mutex for shared data */
    detection_mutex = xSemaphoreCreateMutex();
    if (detection_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return -1;
    }

    setup_led();
    add_log("[System] Fire Detection System starting...");

    /* Initialize WiFi AP Mode */
    wifi_init_softap();

    /* Initialize Edge Impulse sensors and commands */
    dev = static_cast<EiDeviceESP32*>(EiDeviceESP32::get_device());

    ei_printf(
        "Hello from Edge Impulse Device SDK.\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);
    add_log("[EdgeImpulse] SDK initialized");

    /* Setup the inertial sensor */
    if (ei_inertial_init() == false) {
        ei_printf("Inertial sensor initialization failed\r\n");
        add_log("[Warning] Inertial sensor failed");
    } else {
        add_log("[Sensor] Inertial sensor ready");
    }

    /* Setup the analog sensor */
    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor initialization failed\r\n");
        add_log("[Warning] ADC sensor failed");
    } else {
        add_log("[Sensor] ADC sensor ready");
    }

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();

    dev->set_state(eiStateFinished);
    add_log("[System] All systems initialized");

    /* Create FreeRTOS tasks with proper priorities */
    xTaskCreate(
        serial_task,           // Task function
        "serial_task",         // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Task parameters
        5,                     // Priority (5 = high - critical)
        NULL                   // Task handle
    );
    add_log("[Task] Serial task created (Priority: 5)");

    xTaskCreate(
        webserver_task,        // Task function
        "webserver_task",      // Task name
        8192,                  // Stack size (bytes)
        NULL,                  // Task parameters
        3,                     // Priority (3 = medium)
        NULL                   // Task handle
    );
    add_log("[Task] Web server task created (Priority: 3)");

    xTaskCreate(
        led_task,              // Task function
        "led_task",            // Task name
        2048,                  // Stack size (bytes)
        NULL,                  // Task parameters
        1,                     // Priority (1 = low)
        NULL                   // Task handle
    );
    add_log("[Task] LED task created (Priority: 1)");

    xTaskCreate(
        inference_task,        // Task function
        "inference_task",      // Task name
        16384,                 // Stack size (16KB for ML)
        NULL,                  // Task parameters
        4,                     // Priority (4 = high, below serial)
        NULL                   // Task handle
    );
    add_log("[Task] Inference task created (Priority: 4)");
    add_log("[System] FreeRTOS scheduler starting...");

    /* Delete init task - FreeRTOS tasks now run */
    vTaskDelete(NULL);
    
    return 0;
}