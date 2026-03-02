# Drone Fire Detection + Pixhawk Companion Computer

ESP32-S3 firmware combining **Edge Impulse FOMO fire detection** with a **MAVLink Pixhawk companion computer**. A single web dashboard at `http://192.168.4.1` provides live camera feed with AI fire detection overlay, drone arm/disarm/mode controls, RC override sliders, auto-flight sequence, and full telemetry.

WiFi AP: **DroneFireDetect** / **drone12345**

---

## Features

### AI Fire Detection
- **Edge Impulse FOMO** object detection model (96x96 input, ~622 ms inference)
- Real-time bounding box overlay on the camera feed
- Confidence threshold filtering (>50%)
- Web dashboard shows detection label + confidence percentage

### WS2812 RGB LED Status Indicator
Priority-based color scheme on the addressable LED (GPIO 48):

| Priority | State | Color | Condition |
|----------|-------|-------|-----------|
| 1 | **FIRE ALERT** | Red/Orange flash (5 Hz) | Fire detected consistently for 5 seconds |
| 2 | **ARMED** | Solid blue | Drone is armed |
| 3 | **GPS + STABILIZE** | Solid green | GPS 3D fix + STABILIZE mode |
| 4 | **GPS OK** | Dim cyan | GPS 3D fix, other flight mode |
| 5 | **CONNECTED** | White pulse | Pixhawk heartbeat received |
| 6 | **IDLE** | Magenta breathe | No Pixhawk connection |

Fire confirmation requires **25 consecutive positive detections** at 200 ms intervals (5 seconds total). Once confirmed, the red/orange flash sequence plays for 5 seconds. This prevents false LED alerts from single-frame glitches.

### MAVLink v2 Companion Computer
- Companion System ID 200, avoids conflict with Pixhawk SysID 1
- Bidirectional MAVLink v2 over UART (57600 baud)
- Heartbeat exchange with the flight controller
- Receives: heartbeat, attitude, GPS, barometer, battery, VFR HUD, command ACK, param values, status text
- Sends: arm/disarm, force-arm, flight mode changes, RC overrides, param sets, takeoff commands

### Web Dashboard (http://192.168.4.1)
- **Status bar**: Connection state, armed/disarmed, flight mode, battery voltage/percentage
- **Camera feed**: Live JPEG stream with fire detection bounding box overlay (canvas)
- **Controls**: Setup, Arm, Force Arm, Disarm, Flight Mode selector
- **RC Override**: Throttle, Yaw, Pitch, Roll sliders with snap-back to center
- **Auto Flight**: One-click takeoff sequence (GUIDED > ARM > Takeoff > Hover > LAND)
- **Telemetry**: GPS (fix, sats, HDOP, lat/lon/alt), attitude (roll/pitch/yaw), altitude, groundspeed, heading, climb rate, throttle
- **Logs**: Scrolling event log with timestamped entries
- **Camera controls**: V-Flip / H-Flip
- **FPS counter** in browser tab title

### Automated Takeoff Sequence
One-button sequence: Set GUIDED mode > Arm > Takeoff to 5 m > Hover 10 s > Land. Includes geofence safety (altitude + radius limits), arm timeout, takeoff timeout, and abort capability.

### Decoupled Camera Architecture
Camera pump task runs independently at high priority, continuously capturing frames and caching the latest good JPEG. The inference task and web server both read from the cache, never touching the camera hardware directly. This eliminates DMA bus contention with WiFi.

---

## Supported Boards

| Board | Chip | MAVLink UART | LED | Camera |
|-------|------|-------------|-----|--------|
| AI-Thinker ESP32-CAM | ESP32 | UART2 TX=13 RX=14 | GPIO 33 (active LOW) | AI-Thinker pinout |
| GOOUUU ESP32-S3-CAM | ESP32-S3 | UART1 TX=47 RX=21 | GPIO 48 WS2812 RGB | S3-CAM pinout |

Board selection is **automatic** via `#if defined(CONFIG_IDF_TARGET_ESP32S3)` preprocessor checks.

---

## Hardware Architecture

```
                        +------------------------------+
                        |      GOOUUU ESP32-S3-CAM     |
                        |                              |
  OV2640 Camera <------>|  Core 1: cam_pump (prio 15)  |
  (I2S/DMA)             |           inference (prio 4) |
                        |                              |
  WS2812 LED (GPIO 48) <|  Core 0: WiFi AP (prio 23)  |
                        |           webserver (prio 2) |
  Pixhawk (UART1) <---->|           mav_rx (prio 7)   |
  TX=47 RX=21           |           mav_hb (prio 3)   |
                        |           serial (prio 5)    |
                        |           led (prio 1)       |
                        +------------------------------+
```

### FreeRTOS Task Layout

| Task | Core | Priority | Stack | Function |
|------|------|----------|-------|----------|
| `cam_pump` | 1 | 15 | 16 KB | Camera DMA frame capture, JPEG validation, cache |
| `infer` | 1 | 4 | 16 KB | Edge Impulse FOMO inference |
| WiFi | 0 | 23 | (system) | ESP-IDF WiFi SoftAP |
| `mav_rx` | 0 | 7 | 4 KB | MAVLink UART receive + parse |
| `serial` | 0 | 5 | 4 KB | Edge Impulse AT command interface |
| `mav_hb` | 0 | 3 | 4 KB | MAVLink heartbeat + auto-sequence |
| `webserv` | 0 | 2 | 8 KB | HTTP server start |
| `led` | 0 | 1 | 4 KB | WS2812 RGB LED state machine |

---

## Technical Challenges & Solutions

### WiFi / Camera DMA Bus Contention

**Problem**: On ESP32-S3, both WiFi and the camera share the AHB bus for DMA transfers. When WiFi transmits Block-ACK bursts, it starves the camera's GDMA channel, causing:
- `cam_hal: Failed to get the frame on time!` -- DMA transfer never completes
- `cam_hal: FB-OVF` -- Frame buffer overflow (no free buffer to DMA into)
- `cam_hal: NO-EOI` -- End-of-image marker missing (corrupted/truncated JPEG)
- `cam_hal: NO-SOI` -- Start-of-image marker missing

**Solutions implemented**:

1. **Double-buffered with GRAB_LATEST**: `fb_count=2` ensures the camera always has a buffer to write into while the pump holds the other. `GRAB_LATEST` returns the newest completed frame.

2. **JPEG validation before caching**: The pump checks for SOI (0xFFD8) at the start and EOI (0xFFD9) at the end of every frame. Corrupt JPEGs from bus contention are silently dropped -- the cache keeps serving the last known-good frame.

3. **Automatic camera reinit**: If 3 consecutive `esp_camera_fb_get()` calls fail (DMA stuck), the pump calls `camera_reinit()` which does `esp_camera_deinit()` + `esp_camera_init()` to reset the DMA state machine. Recovery takes ~300 ms.

4. **Frame buffers in DRAM** (not PSRAM): Camera DMA targets internal SRAM via the AHB bus. Using PSRAM would add additional MSPI bus contention since WiFi also uses MSPI when SPIRAM is enabled. QVGA JPEG at quality 12 is only ~2-4 KB per buffer.

5. **Bus yield before capture**: The pump yields 5 ms (`vTaskDelay`) before each `fb_get()` call, giving WiFi DMA bursts time to finish.

### Stack Overflow in cam_pump

**Problem**: The pump task originally had only 4096 bytes of stack. Normal operation was fine, but when the DMA got stuck and `camera_reinit()` was called, `esp_camera_init()` does I2C/SCCB probing, sensor detection, and DMA channel setup -- consuming 3-4 KB of stack, exceeding the limit.

**Solution**: Stack increased to 16384 bytes. Provides headroom for the camera HAL's internal error-handling paths, the `camera_reinit()` deep call chain, and ISR register window spills.

### Watchdog Timeout on CPU1

**Problem**: The inference task on Core 1 takes ~622 ms per inference cycle. With `ei_sleep(1)` between iterations, the FreeRTOS IDLE task on Core 1 was starved, unable to feed the task watchdog within the 15-second timeout.

**Solution**: Increased inference yield to `ei_sleep(10)` (10 ms), giving IDLE1 enough time to run and reset the watchdog.

### Decoupled Camera-Inference Architecture

**Problem**: Originally, the inference task called `esp_camera_fb_get()` directly, holding the camera hardware hostage during the 622 ms inference window. This meant:
- The web server couldn't serve frames during inference
- WiFi DMA and camera DMA competed for the AHB bus during the worst possible moment

**Solution**: Implemented a producer-consumer architecture:
- **`camera_pump_task`** (Core 1, priority 15): The only task that touches camera hardware. Runs at ~30 fps, validates JPEGs, caches good frames in PSRAM via a mutex-protected buffer.
- **Inference task** (Core 1, priority 4): Polls the frame cache for new frame IDs, reads cached JPEG copies -- never blocks the camera pipeline.
- **Web frame handler** (Core 0): Reads the same cache, base64-encodes, returns JSON. Independent of camera hardware.

### JPEG Decode Failures After WiFi Connects

**Problem**: With `fb_count=3` and `GRAB_LATEST`, the camera DMA ran continuously, maximizing AHB bus contention. WiFi bursts corrupted DMA transfers, producing malformed JPEGs that all had identical size (e.g., 1778 bytes) -- indicating they were all the same corrupted frame cached and served repeatedly.

**Solution**: The pump now validates JPEG markers (SOI/EOI) before updating the cache. Corrupt frames are dropped; inference and web always get the last known-good frame. Reduced to `fb_count=2` to decrease continuous DMA pressure.

---

## Switching Boards

Two pre-configured `sdkconfig` files are provided. **Copy the right one and clean before building.**

### Build for AI-Thinker ESP32-CAM

```bash
copy sdkconfig.esp32 sdkconfig
idf.py fullclean
idf.py build
idf.py -p COM5 flash monitor
```

### Build for GOOUUU ESP32-S3-CAM

```bash
copy sdkconfig.esp32s3 sdkconfig
idf.py fullclean
idf.py build
idf.py -p COM5 flash monitor
```

> **Why `fullclean`?** The build directory caches the target chip. Switching between ESP32 and ESP32-S3 without cleaning will cause build errors.

> **Change `COM5`** to whatever port your board shows up as (check Device Manager).

### First-time S3 setup (if `sdkconfig.esp32s3` doesn't exist yet)

```bash
copy sdkconfig sdkconfig.esp32            # backup current config
idf.py set-target esp32s3                 # generates fresh S3 sdkconfig + cleans build
idf.py menuconfig                         # configure settings below
```

In menuconfig, set:
- **Component config > ESP PSRAM > Support for external SPI-connected RAM** -- Enable
- **Component config > ESP PSRAM > SPI RAM config > Mode** -- **Octal Mode (OPI)**
- **Serial flasher config > Flash size** -- **8MB** (or match your board)
- **Partition Table > Custom partition CSV file** -- `partitions.csv`

Then save and exit, and backup the S3 config:

```bash
copy sdkconfig sdkconfig.esp32s3
idf.py build
```

---

## Requirements

### Hardware

- ESP32-S3-CAM (GOOUUU) or ESP32-CAM (AI-Thinker) with OV2640 camera
- WS2812B addressable LED on GPIO 48 (ESP32-S3 board)
- Pixhawk flight controller (ArduCopter) connected via UART
- 8 MB Octal PSRAM (for frame cache and inference buffers)

### Tools

Install ESP-IDF v5.5.1, following the instructions for your OS from the [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/get-started/index.html).

---

## Building

From the firmware folder, in an ESP-IDF terminal:

```bash
idf.py build
```

### Flash & Monitor

```bash
idf.py -p COM5 flash monitor
```

Change `COM5` to your actual serial port. Serial: 115200 baud, 8N1.

---

## Web Dashboard

Connect to WiFi AP **DroneFireDetect** (password: **drone12345**), then open **http://192.168.4.1**.

---

## Project Structure

```
main/main.cpp                    -- Merged firmware (fire detection + companion + LED)
main/web-interface.c             -- Reference file (Pixhawk companion, not compiled)
main/main.cpp.bak                -- Original fire-detection-only backup
edge-impulse/                    -- EI platform code + inference engine
edge-impulse-sdk/                -- Edge Impulse C++ SDK
firmware-sdk/                    -- EI device library
model-parameters/                -- FOMO model metadata
tflite-model/                    -- Compiled TFLite model
components/esp32-camera/         -- ESP32 camera driver
components/mavespstm/            -- Lightweight MAVLink v2 library (header-only)
components/LIS3DHTR_ESP-IDF/     -- Accelerometer driver (not used on S3 board)
sdkconfig.esp32                  -- Saved config for AI-Thinker ESP32-CAM
sdkconfig.esp32s3                -- Saved config for GOOUUU ESP32-S3-CAM
partitions.csv                   -- Custom partition table (3 MB app)
```

---

## Configuration Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `WIFI_SSID` | DroneFireDetect | WiFi AP name |
| `WIFI_PASS` | drone12345 | WiFi AP password |
| `MAV_BAUD_RATE` | 57600 | MAVLink UART baud rate |
| `COMPANION_SYSID` | 200 | MAVLink system ID for the ESP32 |
| `FENCE_ALT_MAX` | 10.0 m | Geofence altitude limit |
| `FENCE_RADIUS` | 30.0 m | Geofence radius limit |
| `FIRE_CONSISTENCY_CHECKS` | 25 | Consecutive fire detections needed (x200 ms = 5 s) |
| `FIRE_ALERT_DURATION_MS` | 5000 | How long fire LED alert plays |
| `REINIT_THRESHOLD` | 3 | Consecutive camera failures before reinit |

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | HTML dashboard |
| `/api/status` | GET | JSON: telemetry, detection, flight state |
| `/api/frame` | GET | JSON: base64-encoded JPEG + detection overlay data |
| `/api/arm` | POST | Arm the drone |
| `/api/disarm` | POST | Disarm the drone |
| `/api/forcearm` | POST | Force-arm (bypasses prearm checks) |
| `/api/mode?m=N` | POST | Set flight mode (N = ArduCopter mode number) |
| `/api/setup` | POST | Disable prearm checks + configure geofence |
| `/api/rc` | POST | Send RC override (ch1-ch4 query params) |
| `/api/rc/stop` | POST | Release RC override |
| `/api/takeoff` | POST | Start auto-flight sequence |
| `/api/abort` | POST | Abort auto-flight sequence |
| `/api/vflip` | POST | Toggle camera vertical flip |
| `/api/hflip` | POST | Toggle camera horizontal flip |
| `/api/logs` | GET | JSON array of recent log entries |

---

## Memory Map

| Resource | Location | Size | Purpose |
|----------|----------|------|---------|
| Camera frame buffer x2 | Internal DRAM | ~4 KB each | DMA targets for OV2640 JPEG |
| Frame cache | PSRAM (heap) | ~2-4 KB | Mutex-protected cached JPEG for web/inference |
| Inference snapshot | PSRAM (heap) | 27 KB | 96x96x3 RGB888 buffer for FOMO model |
| TFLite arena | PSRAM (heap) | ~200 KB | TensorFlow Lite Micro tensor arena |
| RMT TX buffer | Internal DRAM | 64 symbols | WS2812 LED signal (tiny) |

---

## Future Improvements

- **MJPEG streaming**: Replace base64-JSON frame delivery with `multipart/x-mixed-replace` for much lower overhead and higher browser FPS
- **MAVLink fire reporting**: Send STATUSTEXT or custom MAVLink message to ground station when fire is confirmed
- **Multi-zone fire tracking**: Track bounding box position over time to estimate fire spread direction
- **Thermal camera integration**: Pair with an MLX90640 or FLIR Lepton for thermal confirmation of fire detections
- **OTA firmware updates**: Over-the-air update capability via the web dashboard
- **SD card logging**: Record fire detection events with timestamps and GPS coordinates
- **Video recording**: Save JPEG frames to SD card during fire events for post-flight analysis
- **Configurable thresholds**: Web UI sliders for confidence threshold, fire consistency duration, LED brightness
- **Battery voltage-based LED**: Show battery level on LED color (green > yellow > red) when drone is idle
- **Lost-link failsafe**: If Pixhawk heartbeat disappears for >30 s, trigger visual/audible alert

---

## Importing a New Edge Impulse Model

When you retrain your model on Edge Impulse and download a new **ESP-IDF** deployment, some files can be dropped in directly and others need **custom patches re-applied**. This section documents exactly what to do.

### Step 1: Replace These Folders/Files (Safe -- No Custom Code)

These are pure Edge Impulse generated files. Replace them entirely:

| Path | What It Contains |
|------|------------------|
| `model-parameters/model_metadata.h` | Model defines: input dimensions, class count, project ID, arena size, quantization, FOMO flags |
| `model-parameters/model_variables.h` | Class labels array, DSP config, learning block function references |
| `tflite-model/` (entire folder) | Compiled model weights (`.cpp`), API header (`.h`), TFLite ops defines |
| `edge-impulse-sdk/` (entire folder) | Edge Impulse C++ SDK: classifier, TFLite Micro, DSP, CMSIS, porting layer |

> **Note**: The new model's `tflite-model/` files will have a **different project/learn ID** in the filename (e.g. `tflite_learn_999999_1_compiled.cpp` instead of `tflite_learn_834117_3_compiled.cpp`). This is fine -- `model_variables.h` references them by function name, and CMakeLists.txt picks up all `.cpp` files in that folder automatically.

### Step 2: Re-Apply Custom Patches (4 Files)

We made custom modifications to 4 files that Edge Impulse's export **will overwrite**. After replacing the folders above, re-apply these patches:

---

#### Patch 1: `edge-impulse-sdk/classifier/ei_print_results.h`

**What**: Hooks `update_detection_data()` into the bounding-box print loop so the web dashboard and LED state machine receive detection results.

**Where**: Inside the `EI_CLASSIFIER_TYPE_OBJECT_DETECTION` branch (~line 85), replace the stock bounding box print loop with:

```cpp
    else if (impulse->results_type == EI_CLASSIFIER_TYPE_OBJECT_DETECTION) {
        ei_printf("Object detection bounding boxes:\n");

        extern void update_detection_data(uint32_t x, uint32_t y, uint32_t width,
                                          uint32_t height, float confidence, const char* label);
        bool detection_found = false;

        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) continue;
            ei_printf("  %s (", bb.label);
            ei_printf_float(bb.value);
            ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n",
                    (unsigned int)bb.x, (unsigned int)bb.y,
                    (unsigned int)bb.width, (unsigned int)bb.height);
            update_detection_data(bb.x, bb.y, bb.width, bb.height, bb.value, bb.label);
            detection_found = true;
        }
        if (!detection_found) {
            update_detection_data(0, 0, 0, 0, 0.0f, "none");
        }
    }
```

---

#### Patch 2: `edge-impulse/inference/ei_run_camera_impulse.cpp`

**What (a)**: Add ESP-IDF headers and the `update_detection_data` extern declaration near the top (after the existing includes):

```cpp
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void update_detection_data(uint32_t x, uint32_t y, uint32_t width,
                                  uint32_t height, float confidence, const char* label);
```

**What (b)**: In the `ei_start_impulse()` function, change the inference loop yield from `ei_sleep(100)` to `ei_sleep(10)` to prevent watchdog timeout on Core 1:

```cpp
    while(!ei_user_invoke_stop()) {
        ei_run_impulse();
        ei_sleep(10);  // yield 10ms so IDLE task on Core 1 can feed the watchdog
    }
```

---

#### Patch 3: `edge-impulse/ingestion-sdk-platform/sensors/ei_camera.cpp`

**What (a)**: Add extern declarations before the `camera_config` struct (after includes):

```cpp
extern "C" void camera_cache_frame(const uint8_t *buf, size_t len);
extern "C" bool camera_get_cached_jpeg(uint8_t **buf_out, size_t *len_out);
extern "C" uint32_t camera_get_frame_id(void);
```

**What (b)**: Replace the stock `camera_config` struct with our optimized version:

```cpp
static camera_config_t camera_config = {
    // ... pin assignments stay the same ...
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 2,                        // double-buffer to avoid FB-OVF
    .fb_location = CAMERA_FB_IN_DRAM,     // internal SRAM, not PSRAM
    .grab_mode = CAMERA_GRAB_LATEST,      // always return newest frame
    .sccb_i2c_port = 0,
};
```

**What (c)**: In `init()`, remove any warm-up frame loop and replace with a 100 ms settle delay:

```cpp
    /* No warm-up loop -- camera_pump_task handles frame acquisition */
    ei_sleep(100);
```

**What (d)**: Replace the stock `ei_camera_capture_jpeg()` with our cache-based version:

```cpp
bool EiCameraESP32::ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size)
{
    int64_t t0 = esp_timer_get_time();
    size_t len = 0;
    bool got_frame = false;
    for (int attempt = 0; attempt < 60; attempt++) {
        if (camera_get_cached_jpeg(image, &len)) { got_frame = true; break; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!got_frame) {
        ESP_LOGE(TAG, "No cached frame after 3s");
        return false;
    }
    *image_size = (uint32_t)len;
    ESP_LOGI(TAG, "Got JPEG: %u bytes in %lld us",
             (unsigned)len, (esp_timer_get_time() - t0));
    return true;
}
```

---

#### Patch 4: `firmware-sdk/ei_device_lib.cpp`

**What**: In the `#if EI_CLASSIFIER_OBJECT_DETECTION == 1` section, add the web interface hook:

```cpp
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");

    extern void update_detection_data(uint32_t x, uint32_t y, uint32_t width,
                                      uint32_t height, float confidence, const char* label);
    bool found_detection = false;
    for (uint32_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        if (bb.value > 0.3) {
            update_detection_data(bb.x, bb.y, bb.width, bb.height, bb.value, bb.label);
            found_detection = true;
        }
    }
    if (!found_detection) {
        update_detection_data(0, 0, 0, 0, 0.0f, "none");
    }
```

---

### Step 3: Verify Model Compatibility

Your new model **must** satisfy these requirements to work without code changes:

| Requirement | Current Value | Why |
|-------------|---------------|-----|
| Input type | **Image (RGB)** | Camera captures RGB frames |
| Input size | **96x96** | Hardcoded in `ei_start_impulse()` and used by `camera_pump_task` |
| Model type | **FOMO (Object Detection)** | `update_detection_data` expects bounding boxes |
| Quantization | **INT8** recommended | Fits in PSRAM arena, runs on ESP32-S3 |
| Compiled (EON) | **Yes** recommended | Lower memory footprint than full TFLite |

> **Changing input size**: If your new model uses a different resolution (e.g. 48x48 or 160x160), no code change is needed -- `EI_CLASSIFIER_INPUT_WIDTH/HEIGHT` from `model_metadata.h` propagates automatically. The camera captures at QVGA (320x240) and the SDK crops/resizes.

> **Changing model type**: If you switch from FOMO to a **classification** model (not object detection), the `update_detection_data` calls in patches 1 and 4 will be inside `#if EI_CLASSIFIER_OBJECT_DETECTION` blocks that won't compile. You'd need to adapt the web interface to show classification results instead of bounding boxes.

> **Adding new classes**: Class labels come from `model_variables.h` and are passed through `bb.label` automatically. The web dashboard, LED, and `update_detection_data` all use string comparison, not indices. If your new model detects `["fire", "smoke", "explosion"]`, the bounding box overlay and LED alert will work for all classes with confidence > 0.5.

### Quick Reference: Files You Touch vs. Files You Don't

```
REPLACE (drop-in from EI export):
  model-parameters/model_metadata.h      <-- new model defines
  model-parameters/model_variables.h     <-- new class labels + DSP config
  tflite-model/*                         <-- new compiled model
  edge-impulse-sdk/*                     <-- new SDK (then re-apply Patch 1)

PATCH AFTER REPLACING (re-apply custom code):
  edge-impulse-sdk/classifier/ei_print_results.h    <-- Patch 1
  edge-impulse/inference/ei_run_camera_impulse.cpp   <-- Patch 2
  edge-impulse/ingestion-sdk-platform/sensors/ei_camera.cpp  <-- Patch 3
  firmware-sdk/ei_device_lib.cpp                     <-- Patch 4

NEVER REPLACE (your application code):
  main/main.cpp                          <-- all drone logic, web UI, LED, pump
  main/CMakeLists.txt                    <-- build config
  components/*                           <-- camera driver, MAVLink lib
  sdkconfig / sdkconfig.esp32s3          <-- ESP-IDF config
  partitions.csv                         <-- partition table
```

---

## License

BSD-3-Clause-Clear (Edge Impulse SDK)
