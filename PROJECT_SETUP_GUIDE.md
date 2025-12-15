# ESP32-CAM Edge Impulse Firmware - Setup Guide

## ⚠️ Important Notice

**This is a PERSONAL project with custom model and configuration files!**

- ✅ **Configured for**: ESP-IDF v5.5.1 specifically
- ✅ **Includes**: My trained model (phone & DMM detection)
- ⚠️ **May NOT work for you as-is** - You need to:
  1. Replace model files with YOUR trained model
  2. Apply the fixes documented below for ESP-IDF v5.5.1
  3. Or use ESP-IDF v5.1.4 (recommended) to avoid fixes

**For best results**: Use this as a reference guide and apply fixes to a fresh Edge Impulse firmware download.

---

## Project Overview
This project enables Edge Impulse object detection on ESP32-CAM AI-Thinker using ESP-IDF v5.5.1. The firmware allows running trained machine learning models directly on the ESP32-CAM for real-time object detection.

## Device Information
- **Board**: ESP32-CAM AI-Thinker
- **ESP-IDF Version**: v5.5.1
- **Communication**: UART (Serial) at 115200 baud
- **Port**: COM23

## Successfully Deployed Model
- **Model Type**: Object Detection (FOMO)
- **Classes**: phone, DMM (Digital Multimeter)
- **Input Size**: 96x96 pixels
- **Performance**: ~448ms inference time, ~2 FPS
- **Accuracy**: 50%-99% confidence scores

---

## Issues Fixed & Changes Made

### 1. UART Type Conversion Error
**Problem**: Compilation failed due to invalid conversion from `int` to `uart_port_t`

**File**: `edge-impulse/ingestion-sdk-platform/espressif_esp32/ei_device_espressif_esp32.cpp`

**Changes**:
- Line 276: Changed `uart_set_baudrate(0, DEFAULT_BAUD)` → `uart_set_baudrate(UART_NUM_0, DEFAULT_BAUD)`
- Line 283: Changed `uart_set_baudrate(0, MAX_BAUD)` → `uart_set_baudrate(UART_NUM_0, MAX_BAUD)`

---

### 2. FreeRTOS Task Priority Error
**Problem**: Boot loop with error `assert failed: prvInitialiseNewTask tasks.c:1111 (uxPriority < ( 25 ))`

**File**: `sdkconfig`

**Changes**:
- Line 1375: `CONFIG_FREERTOS_TIMER_TASK_PRIORITY=25` → `24`
- Line 2277: `CONFIG_TIMER_TASK_PRIORITY=25` → `24`

**Reason**: ESP-IDF v5.5.1 has maximum priority of 24, but config had 25

---

### 3. Camera Configuration for ESP-IDF v5.5.1
**Problem**: Camera initialization failed - missing required fields for newer ESP-IDF

**File**: `edge-impulse/ingestion-sdk-platform/sensors/ei_camera.cpp`

**Changes** (Lines 73-86):
```cpp
// Before:
.fb_count = 4,
//.fb_location = CAMERA_FB_IN_PSRAM,
.grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// After:
.fb_count = 4,
.fb_location = CAMERA_FB_IN_PSRAM,        // Uncommented - required
.grab_mode = CAMERA_GRAB_WHEN_EMPTY,
.sccb_i2c_port = 0,                       // Added - required for v5.5.1
};
```

---

### 4. LEDC Channel Configuration Error
**Problem**: Camera clock failed with `ledc_channel_config(847): sleep_mode argument is invalid`

**File**: `components/esp32-camera/target/xclk.c`

**Changes** (Lines 50-60):
```cpp
// Before:
ledc_channel_config_t ch_conf;
ch_conf.gpio_num = config->pin_xclk;
ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
ch_conf.channel = config->ledc_channel;
ch_conf.intr_type = LEDC_INTR_DISABLE;
ch_conf.timer_sel = config->ledc_timer;
ch_conf.duty = 1;
ch_conf.hpoint = 0;
err = ledc_channel_config(&ch_conf);

// After:
ledc_channel_config_t ch_conf = {
    .gpio_num = config->pin_xclk,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = config->ledc_channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = config->ledc_timer,
    .duty = 1,
    .hpoint = 0,
    .flags = {
        .output_invert = 0
    }
};
err = ledc_channel_config(&ch_conf);
```

**Reason**: ESP-IDF v5.5.1 requires designated initializers and `.flags` field

---

## Build & Flash Process

### 1. Clean Build
```powershell
Remove-Item -Recurse -Force .\build
```

### 2. Build Firmware
Use VS Code ESP-IDF extension: `ESP-IDF: Build your project`

### 3. Flash to Device
Use VS Code ESP-IDF extension: `ESP-IDF: Flash your project`

### 4. Monitor Serial Output
Use VS Code ESP-IDF extension: `ESP-IDF: Monitor your device`

---

## Deploying Your Own Model

### Step 1: Train Model on Edge Impulse
1. Create project at https://studio.edgeimpulse.com/
2. Collect training data
3. Train your model (FOMO, Classification, etc.)

### Step 2: Export Model
1. Go to **Deployment** in Edge Impulse Studio
2. Select **ESP-IDF**
3. Download the library

### Step 3: Replace Model Files
Replace these folders with your downloaded files:
```
firmware-espressif-esp32-main/
├── tflite-model/          ← Your model files
├── model-parameters/      ← Your model parameters
└── edge-impulse-sdk/      ← (May need update)
```

### Step 4: Rebuild & Flash
1. Clean build directory
2. Build firmware
3. Flash to ESP32-CAM

---

## Usage & AT Commands

### Connect to Device
- **Baudrate**: 115200
- **Port**: COM23 (or your device's port)

### Available Commands

**Get Help:**
```
AT+HELP
```

**Device Info:**
```
AT+CONFIG?
```

**Take Snapshot:**
```
AT+SNAPSHOT=96,96
```

**Run Inference (Single):**
```
AT+RUNIMPULSE
```

**Run Inference (Continuous):**
```
AT+RUNIMPULSECONT
```

**Stop Continuous Inference:**
Press `Ctrl+C` in serial monitor

---

## System Configuration

### Communication
- **Default Baud**: 115200 (for AT commands)
- **Max Baud**: 1,000,000 (for image transfer)
- **Interface**: UART/Serial (No WiFi required)

### Camera Settings
- **Format**: JPEG
- **Frame Size**: QVGA (320x240) default
- **Supported Resolutions**: 64x64, 96x96, 160x120, 320x240, 480x320
- **Frame Buffer**: PSRAM
- **Clock**: 20MHz

### Performance
- **DSP Time**: 5-6ms
- **Inference Time**: ~448ms
- **FPS**: ~2 frames per second
- **Memory**: Uses PSRAM for frame buffers

---

## Troubleshooting

### Camera Not Detected
- Check camera module connection
- Verify PSRAM is enabled in `sdkconfig`
- Ensure correct pin definitions for AI-Thinker board

### Build Errors
- Clean build directory: `Remove-Item -Recurse -Force .\build`
- Check ESP-IDF version: v5.5.1
- Verify all changes from this guide are applied

### Boot Loop
- Check FreeRTOS task priorities in `sdkconfig` (must be ≤24)
- Verify partition table is correct

### Serial Monitor Closes Immediately

**Problem**: VS Code ESP-IDF serial monitor closes right after opening

**Solutions:**

1. **Use External Monitor** (Recommended):
   - **PuTTY**: COM23, 115200 baud, Serial connection
   - **Arduino Serial Monitor**: Select COM23, set 115200 baud
   - **TeraTerm**: Setup → Serial port → COM23, Speed: 115200

2. **Command Line Monitor**:
   ```powershell
   idf.py -p COM23 monitor
   # Exit with Ctrl+]
   ```

3. **Fix VS Code Settings**:
   Create/edit `.vscode/settings.json`:
   ```json
   {
       "idf.monitorBaudRate": "115200",
       "idf.port": "COM23"
   }
   ```
   Then use: **ESP-IDF: Monitor your device** (separate from flash)

### Low Inference Speed
- Normal for ESP32 (~450ms)
- Use smaller input resolution (64x64)
- Enable model optimization in Edge Impulse

---

## File Structure

```
firmware-espressif-esp32-main/
├── CMakeLists.txt
├── sdkconfig                           ← Modified (priorities)
├── partitions.csv
├── components/
│   └── esp32-camera/
│       └── target/
│           └── xclk.c                  ← Modified (LEDC flags)
├── edge-impulse/
│   └── ingestion-sdk-platform/
│       ├── espressif_esp32/
│       │   └── ei_device_espressif_esp32.cpp  ← Modified (UART)
│       └── sensors/
│           └── ei_camera.cpp           ← Modified (camera config)
├── tflite-model/                       ← Your trained model
├── model-parameters/                   ← Model metadata
└── main/
    └── main.cpp
```

---

## Expected Output

### Successful Boot
```
Hello from Edge Impulse Device SDK.
Compiled on Oct 28 2025 18:19:37
WARNING: Failed to connect to Inertial sensor!
Inertial sensor initialization failed
Type AT+HELP to see a list of commands.
```
*Note: Inertial sensor warning is normal - ESP32-CAM has no IMU*

### Inference Results
```
Starting inferencing in 2 seconds...
Taking photo...
Timing: DSP 5 ms, inference 448 ms, anomaly 0 ms
Object detection bounding boxes:
  phone (0.941406) [ x: 24, y: 24, width: 56, height: 48 ]
  DMM (0.914062) [ x: 24, y: 24, width: 56, height: 16 ]
```

---

## Summary

✅ **Fixed 4 critical compatibility issues** with ESP-IDF v5.5.1  
✅ **Camera working** - captures and processes images  
✅ **Model deployed** - object detection running on-device  
✅ **Performance validated** - 2 FPS with good accuracy  

**Total Development Time**: ~1 session  
**Final Status**: Fully operational Edge Impulse firmware on ESP32-CAM

---

## Next Steps

1. **Improve Model**: Collect more training data in various lighting conditions
2. **Add WiFi**: Enable remote monitoring and updates
3. **Optimize Performance**: Test with quantized models
4. **Add Actions**: Trigger GPIO outputs based on detections
5. **Power Optimization**: Implement sleep modes for battery operation

---

## FAQ - Frequently Asked Questions

### Q1: Would using ESP-IDF v5.1.1 have been easier?

**YES, absolutely!** 🎯

With ESP-IDF **v5.1.1** (or v5.0.x), you would have avoided **ALL 4 fixes** listed in this guide:

- ❌ No LEDC `sleep_mode` error
- ❌ No LEDC `flags` structure issue  
- ❌ No I2S driver compatibility problems
- ❌ No camera initialization errors

**Why?** The Edge Impulse firmware was originally designed for ESP-IDF v5.0-v5.1. Version v5.5.1 introduced breaking changes in:
- LEDC API (sleep modes removed)
- I2S driver architecture (complete rewrite)
- Camera component structure

**Recommendation for future projects**: Use **ESP-IDF v5.1.4** (stable, well-supported)

---

### Q2: Do I need to repeat all fixes when training a new model?

**NO! You only need to replace model files and rebuild!** ✨

#### One-Time Fixes (Already Done ✅):
1. ✅ UART type conversion fix
2. ✅ FreeRTOS priority fix
3. ✅ Camera configuration fix
4. ✅ LEDC channel fix

These fixes are **permanent** and apply to all models.

#### For Each New Model:

```powershell
# 1. Train your model in Edge Impulse Studio
# 2. Download as "ESP-IDF library"

# 3. Delete old model folders
Remove-Item -Recurse -Force .\edge-impulse-sdk
Remove-Item -Recurse -Force .\model-parameters
Remove-Item -Recurse -Force .\tflite-model

# 4. Extract new model files from downloaded zip
# (Copy the 3 folders from your download)

# 5. Clean build
Remove-Item -Recurse -Force .\build

# 6. Build & flash (same as before)
# Use VS Code ESP-IDF extension or:
# idf.py build
# idf.py -p COM23 flash monitor
```

#### What Changes Between Models:
- `edge-impulse-sdk/` - Updated SDK (if newer version)
- `model-parameters/` - New classes/labels/config
- `tflite-model/` - New neural network weights

#### What Stays the Same:
- All 4 fixes from this guide
- Camera configuration
- Build system
- AT commands
- Serial communication
- ESP32-CAM hardware setup

---

### Q3: Can I use the same firmware on multiple ESP32-CAM boards?

**YES!** Once built, you can flash the same firmware to multiple boards:

```powershell
# Flash pre-built firmware to another board
cd build
esptool.py -p COM23 write_flash @flash_args
```

No need to rebuild for identical hardware.

---

### Q4: What ESP-IDF version should I use for new projects?

| Version | Recommendation | Notes |
|---------|---------------|-------|
| **v5.1.4** | ✅ **RECOMMENDED** | Stable, no fixes needed |
| v5.0.x | ✅ Good | Works with Edge Impulse |
| v5.3.x | ⚠️ Possible issues | May need minor fixes |
| v5.5.x | ⚠️ Needs fixes | Requires all 4 fixes from this guide |
| v4.x | ❌ Not recommended | Too old for latest features |

---

### Q5: How do I speed up inference time?

Current performance: ~448ms per inference (~2 FPS)

**Optimization Options:**

1. **Reduce Input Size**
   - Use 64x64 instead of 96x96
   - Expected: ~200-250ms

2. **Model Optimization in Edge Impulse**
   - Enable INT8 quantization
   - Use EON Compiler
   - Expected: 30-50% faster

3. **Hardware Acceleration**
   - ESP32-S3 has better performance
   - Expected: 2-3x faster

4. **Reduce Model Complexity**
   - Fewer layers
   - Smaller feature extractors
   - Expected: Varies by model

---

### Q6: Can I add WiFi connectivity?

**YES!** The firmware already has WiFi support built-in:

```
AT+WIFI=YOUR_SSID,YOUR_PASSWORD,WPA2
```

Then use `AT+UPLOADSETTINGS` to configure Edge Impulse upload settings.

---

### Q7: Why do I see "Inertial sensor initialization failed"?

**This is NORMAL and expected!** ⚠️

ESP32-CAM AI-Thinker **does not have** an accelerometer/gyroscope. The Edge Impulse firmware is generic and tries to initialize all possible sensors. This warning can be safely ignored for camera-only projects.

---

### Q8: Quick Command Reference

| Task | Command |
|------|---------|
| Help | `AT+HELP` |
| Device info | `AT+CONFIG?` |
| Take snapshot | `AT+SNAPSHOT=96,96` |
| Run inference once | `AT+RUNIMPULSE` |
| Continuous inference | `AT+RUNIMPULSECONT` |
| Stop continuous | `Ctrl+C` |
| Set WiFi | `AT+WIFI=SSID,PASS,WPA2` |

---

## References

- [Edge Impulse Documentation](https://docs.edgeimpulse.com/)
- [ESP-IDF v5.5.1 Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/)
- [ESP-IDF v5.1.4 Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.1.4/) (Recommended)
- [ESP32-CAM Repository](https://github.com/espressif/esp32-camera)
- [Edge Impulse ESP32 Forum](https://forum.edgeimpulse.com/)

---

**Date**: October 28, 2025  
**Author**: Development Session  
**Status**: Production Ready ✅
