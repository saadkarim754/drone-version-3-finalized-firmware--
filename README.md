# Drone Fire Detection + Pixhawk Companion Computer

ESP32-CAM firmware combining **Edge Impulse FOMO fire detection** with a **MAVLink Pixhawk companion computer**. A single web dashboard at `http://192.168.4.1` provides live camera feed with AI fire detection overlay, drone arm/disarm/mode controls, RC override sliders, auto-flight sequence, and full telemetry.

WiFi AP: **DroneFireDetect** / **drone12345**

---

## Supported Boards

| Board | Chip | MAVLink UART | LED | Camera |
|-------|------|-------------|-----|--------|
| AI-Thinker ESP32-CAM | ESP32 | UART2 TX=13 RX=14 | GPIO 33 (active LOW) | AI-Thinker pinout |
| GOOUUU ESP32-S3-CAM | ESP32-S3 | UART1 TX=47 RX=21 | GPIO 48 (active HIGH) | S3-CAM pinout |

Board selection is **automatic** — the code uses `#if defined(CONFIG_IDF_TARGET_ESP32S3)` preprocessor checks, so the correct pins, UART, and LED polarity are chosen at compile time based on which chip you're building for.

---

## Switching Boards

Two pre-configured `sdkconfig` files are provided. **You must copy the right one and clean before building.**

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

> **Why `fullclean`?** The build directory caches the target chip. Switching between ESP32 and ESP32-S3 without cleaning will cause build errors. `fullclean` deletes the build folder so everything is regenerated from the new sdkconfig.

> **Change `COM5`** to whatever port your board shows up as (check Device Manager).

### First-time S3 setup (if `sdkconfig.esp32s3` doesn't exist yet)

```bash
copy sdkconfig sdkconfig.esp32            # backup current config
idf.py set-target esp32s3                 # generates fresh S3 sdkconfig + cleans build
idf.py menuconfig                         # configure these settings:
```

In menuconfig, set:
- **Component config → ESP PSRAM → Support for external SPI-connected RAM** → Enable
- **Component config → ESP PSRAM → SPI RAM config → Mode** → **Octal Mode (OPI)**
- **Serial flasher config → Flash size** → **8MB** (or match your board)
- **Partition Table → Custom partition CSV file** → `partitions.csv`

Then save and exit, and backup the S3 config:

```bash
copy sdkconfig sdkconfig.esp32s3
idf.py build
```

Now both configs are saved and you can switch between boards with just `copy` + `fullclean` + `build`.

---

## Requirements

### Hardware

- ESP32-CAM (AI-Thinker) or ESP32-S3-CAM (GOOUUU) with OV2640 camera
- Pixhawk flight controller (ArduCopter) connected via UART

### Tools
Install ESP IDF v5.1.1, following the instructions for your OS from [this page](https://docs.espressif.com/projects/esp-idf/en/v5.1.1/esp32/get-started/index.html#installation-step-by-step).

### Building the application
From the firmware folder, in an ESP-IDF terminal:
```bash
idf.py build
```

### Flash

Connect the board to your computer and run:
```bash
idf.py -p COM5 flash monitor
```

Change `COM5` to your actual serial port.

### Serial connection

115200 baud, 8N1.

---

## Web Dashboard

Connect to WiFi AP **DroneFireDetect** (password: **drone12345**), then open **http://192.168.4.1**.

Features:
- **Status bar**: Connection, armed state, flight mode, battery voltage/percentage
- **Camera feed**: Live MJPEG with fire detection bounding box overlay
- **Controls**: Setup, Arm, Force Arm, Disarm, Flight Mode selector
- **RC Override**: Throttle, Yaw, Pitch, Roll sliders with snap-back
- **Auto Flight**: One-click takeoff sequence (GUIDED → ARM → Takeoff → Hover → LAND)
- **Telemetry**: GPS, attitude, altitude, groundspeed, heading, climb rate
- **Logs**: Scrolling event log

---

## Project Structure

```
main/main.cpp                    ← Merged firmware (fire detection + companion)
main/web-interface.c             ← Reference file (Pixhawk companion, not compiled)
main/main.cpp.bak                ← Original fire-detection-only backup
edge-impulse/                    ← EI platform code + inference engine
edge-impulse-sdk/                ← Edge Impulse C++ SDK
firmware-sdk/                    ← EI device library
model-parameters/                ← FOMO model metadata
tflite-model/                    ← Compiled TFLite model
components/esp32-camera/         ← Camera driver
components/mavespstm/            ← Lightweight MAVLink v2 library (header-only)
components/LIS3DHTR_ESP-IDF/     ← Accelerometer driver
sdkconfig.esp32                  ← Saved config for AI-Thinker ESP32-CAM
sdkconfig.esp32s3                ← Saved config for GOOUUU ESP32-S3-CAM
partitions.csv                   ← Custom partition table (3MB app)
```

Additionally, since Edge Impulse firmware is open-source and available to public, if you have made modifications/added new sensors capabilities, we encourage you to make a PR in firmware repository!
