# BMI160_ESPx Library Reference & Developer Guide

`BMI160_ESPx` is a thread-safe Arduino/PlatformIO C++ library for the Bosch BMI160 6-axis Inertial Measurement Unit (IMU) on Espressif ESP32 microcontrollers.

---

## Table of Contents

1. [Architecture & Coordinate System](#1-architecture--coordinate-system)
2. [Data Structures & Enumerations](#2-data-structures--enumerations)
3. [Complete Method Reference](#3-complete-method-reference)
   - [Constructor & Destructor](#constructor--destructor)
   - [Initialization & Bus Recovery](#initialization--bus-recovery)
   - [Motion Reading & Telemetry](#motion-reading--telemetry)
   - [Direct Accessor Getters](#direct-accessor-getters)
   - [Sensor Configuration & Ranges](#sensor-configuration--ranges)
   - [Sensor Fusion & AHRS Algorithms](#sensor-fusion--ahrs-algorithms)
   - [Coordinate Frame Remapping & Axis Inversion](#coordinate-frame-remapping--axis-inversion)
   - [Tare Zeroing & Orientation Offsets](#tare-zeroing--orientation-offsets)
   - [Calibration & Bias Compensation](#calibration--bias-compensation)
   - [Digital Filtering](#digital-filtering)
   - [Diagnostics & Low-Level Register Access](#diagnostics--low-level-register-access)
4. [Internal Attributes & State Variables](#4-internal-attributes--state-variables)
5. [Practical Code Recipes](#5-practical-code-recipes)

---

## 1. Architecture & Coordinate System

### Coordinate Conventions
* **Sensor Frame:** Right-handed Cartesian coordinate system:
  * **+X:** Towards connector / nose forward
  * **+Y:** Lateral left / wing left
  * **+Z:** Normal upwards out of the sensor package (+1g when laying flat on a table)
* **Attitude Angles:**
  * **Pitch:** Rotation around lateral Y-axis (nose up/down). Range: -90.0° to +90.0° (or -π/2 to +π/2 rad).
  * **Roll:** Rotation around longitudinal X-axis (banking left/right). Range: -180.0° to +180.0° (or -π to +π rad).
  * **Yaw:** Rotation around vertical Z-axis (heading). Range: -180.0° to +180.0° (or -π to +π rad).
* **Quaternions:** Hamilton unit quaternion convention `[qw, qx, qy, qz]` where `qw² + qx² + qy² + qz² = 1.0`.

### Data Transformation Pipeline
When `read()` is invoked, raw register data flows through the following pipeline:
```text
1. Hardware Burst Read (0x0C - 0x17)
   │
2. Scale Factor Division (LSB -> g and deg/s)
   │
3. Axis Remapping (setAxisRemap)
   │
4. Stationary Gyro Bias Subtraction (calibrate / setGyroOffsets)
   │
5. Dynamic Rejection & ZUPT Deadband Gating
   │
6. Sensor Fusion (Complementary or Madgwick AHRS)
   │
7. Attitude Extraction (Raw Roll, Pitch, Yaw, Quaternions)
   │
8. Axis Sign Inversion (invertPitch / invertRoll / invertYaw)
   │
9. Tare Subtraction (tare / setTare) -> Final Output Angles
```

---

## 2. Data Structures & Enumerations

All types reside in namespace `BMI160_ESPx_Types` and are aliased into the root scope in `BMI160_ESPx.h`.

### `Motion`
Primary high-level telemetry and attitude structure.
```cpp
struct Motion {
    // Accelerometer linear acceleration in units of g (1g ≈ 9.80665 m/s²)
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    // Gyroscope angular velocity in deg/s (or rad/s if setDegrees(false))
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    // 6-DOF Orientation Euler Angles in degrees (or rad)
    float pitch = 0.0f; // Nose up (+), nose down (-)
    float roll  = 0.0f; // Bank right (+), bank left (-)
    float yaw   = 0.0f; // Heading angle integrated from gyro

    // Unit Orientation Quaternion (Hamilton convention: qw + qx*i + qy*j + qz*k)
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    // Silicon die temperature
    float temperature = 0.0f; // Sensor die temperature in °C

    // High-resolution timestamp
    uint64_t timestamp_us = 0; // esp_timer_get_time() timestamp in μs
};
```

### `RawMotion`
Low-level structure for 16-bit uncalibrated sensor readings.
```cpp
struct RawMotion {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
};
```

### `FusionMode`
```cpp
enum class FusionMode : uint8_t {
    COMPLEMENTARY = 0, // Lightweight 2-axis tilt filter
    MADGWICK      = 1  // 6-DOF quaternion gradient-descent AHRS filter
};
```

### `FilterBandwidth`
```cpp
enum class FilterBandwidth : uint8_t {
    OSR4   = 0x00, // 4× oversampling (maximum vibration damping)
    OSR2   = 0x10, // 2× oversampling (balanced response)
    NORMAL = 0x20  // Normal mode (cutoff frequency = 0.5 × ODR)
};
```

### `RangeG` & `RangeDPS`
```cpp
enum class RangeG : uint8_t { G_2 = 2, G_4 = 4, G_8 = 8, G_16 = 16 };
enum class RangeDPS : uint16_t { DPS_125 = 125, DPS_250 = 250, DPS_500 = 500, DPS_1000 = 1000, DPS_2000 = 2000 };
```

### `ErrorCode`
```cpp
enum class ErrorCode : uint8_t {
    NONE = 0,
    ERR_NO_PULLUP,
    ERR_CHIP_NOT_FOUND,
    ERR_RESET_TIMEOUT,
    ERR_I2C_NACK,
    ERR_MUTEX_TIMEOUT
};
```

---

## 3. Complete Method Reference

### Constructor & Destructor

#### `explicit BMI160_ESPx(TwoWire &wire = Wire)`
* **Description:** Creates driver instance and allocates the internal FreeRTOS recursive mutex.
* **Parameters:** `wire` - Reference to I2C peripheral.

#### `virtual ~BMI160_ESPx()`
* **Description:** Destructor. Deallocates the FreeRTOS mutex.

---

### Initialization & Bus Recovery

#### `bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0)`
* **Description:** Initializes communication with the sensor.
  1. Runs active 9-clock bus pre-check on `sda` and `scl` to clear hung slave devices.
  2. Probes addresses `0x68` and `0x69` for Chip ID `0xD1`.
  3. Soft-resets the chip (`0xB6`).
  4. Powers on Accelerometer and Gyroscope in Normal mode.
  5. Sets defaults: ±2g, ±250 °/s, 100 Hz ODR.
  6. Accelerates I2C clock to `frequency`.

---

### Motion Reading & Telemetry

#### `bool read(Motion &m)`
* **Description:** Performs an atomic 12-byte burst read across gyro and accel registers (`0x0C` - `0x17`), scales to physical units, updates sensor fusion, and writes output into `m`.
* **Returns:** `true` on success; `false` on I2C bus error or timeout.

#### `Motion read()`
* **Description:** Value-returning overload of `read(Motion &m)`.

#### `const Motion& getLastMotion() const`
* **Description:** Returns a reference to the cached `Motion` packet from the most recent read.

#### `float getTemperature()`
* **Description:** Reads silicon die temperature in °C. Rate-limited to 500 ms internally.

---

### Direct Accessor Getters

Inlined accessors for reading the latest cached telemetry:
* `float getPitch() const`
* `float getRoll() const`
* `float getYaw() const`
* `float getAccelX() const`, `float getAccelY() const`, `float getAccelZ() const`
* `float getGyroX() const`, `float getGyroY() const`, `float getGyroZ() const`

---

### Sensor Configuration & Ranges

#### `bool setAccelRange(uint8_t g = 2)` / `bool setAccelRange(RangeG range)`
* **Description:** Sets accelerometer full-scale range (2, 4, 8, 16 g).

#### `bool setGyroRange(uint16_t dps = 250)` / `bool setGyroRange(RangeDPS range)`
* **Description:** Sets gyroscope full-scale range (125, 250, 500, 1000, 2000 °/s).

#### `bool setDataRate(uint16_t hz = 100)`
* **Description:** Sets output data rate (25, 50, 100, 200, 400, 800, 1600 Hz).

#### `void setDegrees(bool enable_degrees = true)`
* **Description:** `true` for degrees (° and deg/s); `false` for radians.

---

### Sensor Fusion & AHRS Algorithms

#### `void setFusionAlgorithm(FusionMode mode = FusionMode::MADGWICK)`
* **Description:** Selects active orientation algorithm (`FusionMode::MADGWICK` or `FusionMode::COMPLEMENTARY`).

#### `void setMadgwickBeta(float beta = 0.1f)`
* **Description:** Tunes Madgwick filter divergence gain in rad/s.

#### `void setFilterWeight(float alpha = 0.98f)`
* **Description:** Tunes Complementary filter gyro weight factor.

#### `void setDynamicAccelRejection(bool enable)`
* **Description:** Enables automatic gravity gating to ignore accelerometer noise during high dynamic motions.

#### `void setZUPT(bool enable, float threshold_dps = 0.35f)`
* **Description:** Clamps tiny stationary gyro noise to zero to prevent yaw drift while resting.

#### `void resetOrientation()`
* **Description:** Resets orientation angles to 0 and re-aligns quaternion to gravity vector.

---

### Coordinate Frame Remapping & Axis Inversion

#### `void setAxisRemap(int8_t x_axis = 1, int8_t y_axis = 2, int8_t z_axis = 3)`
* **Description:** Remaps sensor physical axes (+/-1, +/-2, +/-3) to match chassis orientation.

#### `void invertPitch(bool invert = true)` / `invertRoll(bool invert = true)` / `invertYaw(bool invert = true)`
* **Description:** Toggles sign inversion for attitude angles.

---

### Tare Zeroing & Orientation Offsets

#### `void tare()`
* **Description:** Sets current attitude as zero reference. Subsequent reads will report (0, 0, 0).

#### `void setTare(float pitch_offset, float roll_offset, float yaw_offset)`
* **Description:** Manually sets tare offset angles (useful for restoring calibration from NVS).

#### `void getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const`
* **Description:** Retrieves active tare offsets.

#### `void resetTare()`
* **Description:** Clears tare offsets back to zero.

---

### Calibration & Bias Compensation

#### `void calibrate(uint16_t samples = 200)`
* **Description:** Samples stationary sensor, averages gyro zero-rate biases, and aligns attitude to gravity vector. Sensor must remain static.

#### `void setGyroOffsets(float bx, float by, float bz)`
* **Description:** Manually sets gyro zero-rate bias offsets (in deg/s).

#### `void getGyroOffsets(float &bx, float &by, float &bz) const`
* **Description:** Retrieves current gyro zero-rate bias offsets.

---

### Digital Filtering

#### `bool setDLPF(FilterBandwidth bw = FilterBandwidth::OSR4)`
* **Description:** Configures hardware DLPF oversampling mode (`OSR4`, `OSR2`, `NORMAL`).

---

### Diagnostics & Low-Level Register Access

#### `ErrorCode getLastError() const`
* **Description:** Returns the last error recorded by the driver.

#### `bool isConnected()`
* **Description:** Checks if sensor responds with chip ID `0xD1`.

#### `bool softReset()`
* **Description:** Issues software reset (`0xB6`) and waits 100 ms.

#### `bool readRaw(RawMotion &raw)`
* **Description:** Reads raw uncalibrated 16-bit integers without running fusion filters.

#### `bool writeRegister(uint8_t reg, uint8_t value)`
* **Description:** Writes an 8-bit byte to a BMI160 hardware register.

#### `bool readRegister(uint8_t reg, uint8_t &value)`
* **Description:** Reads an 8-bit byte from a BMI160 hardware register.

#### `bool readBurst(uint8_t start_reg, uint8_t *buf, size_t length)`
* **Description:** Multi-byte burst read from auto-incrementing registers.

#### `uint8_t getAddress() const`
* **Description:** Returns active 7-bit I2C address (`0x68` or `0x69`).

---

## 4. Internal Attributes & State Variables

| Variable | Type | Description |
| :--- | :--- | :--- |
| `_wire` | `TwoWire&` | Reference to active I2C peripheral instance |
| `_addr` | `uint8_t` | Active 7-bit I2C slave address (`0x68` or `0x69`) |
| `_sda`, `_scl` | `int` | Configured ESP32 GPIO pins for I2C data and clock |
| `_freq` | `uint32_t` | Configured I2C bus clock speed in Hz (default: 400,000 Hz) |
| `_use_degrees` | `bool` | Output units flag (`true` = degrees and deg/s; `false` = radians) |
| `_fusion_mode` | `FusionMode` | Active attitude fusion algorithm (`MADGWICK` or `COMPLEMENTARY`) |
| `_active_bw` | `FilterBandwidth` | Active DLPF oversampling ratio |
| `_active_odr_hz` | `uint16_t` | Cached sensor output data rate |
| `_last_error` | `ErrorCode` | Most recent diagnostic error code |
| `_dynamic_accel_rejection` | `bool` | Dynamic acceleration gating enable flag |
| `_zero_velocity_update` | `bool` | Zero-Velocity Update (ZUPT) deadband enable flag |
| `_zupt_threshold_dps` | `float` | Angular velocity threshold for ZUPT clamping (default: 0.35 deg/s) |
| `_accel_scale` | `float` | LSB/g conversion divisor (16384 for ±2g, etc.) |
| `_gyro_scale` | `float` | LSB/(deg/s) conversion divisor (131.2 for ±250 deg/s, etc.) |
| `_bias_gx`, `_bias_gy`, `_bias_gz` | `float` | Stationary gyroscope zero-rate bias offsets in deg/s |
| `_filter_alpha` | `float` | Complementary filter weighting coefficient (default: 0.98) |
| `_madgwick_beta` | `float` | Madgwick filter divergence gain in rad/s (default: 0.1) |
| `_pitch`, `_roll`, `_yaw` | `float` | Cached attitude angles |
| `_remap_x`, `_remap_y`, `_remap_z` | `int8_t` | Active axis remapping matrix codes |
| `_invert_pitch`, `_invert_roll`, `_invert_yaw` | `bool` | Axis sign inversion flags |
| `_tare_pitch`, `_tare_roll`, `_tare_yaw` | `float` | Locked zero-reference offset angles |
| `_q0`, `_q1`, `_q2`, `_q3` | `float` | Madgwick unit orientation quaternion state vector |
| `_last_filter_us` | `uint64_t` | Microsecond timestamp of previous filter execution |
| `_cached_temp` | `float` | Rate-limited die temperature cache |
| `_last_temp_read_us` | `uint64_t` | Microsecond timestamp of previous temperature read |
| `_last_motion` | `Motion` | Complete telemetry packet from most recent read |
| `_mutex` | `SemaphoreHandle_t` | FreeRTOS recursive mutex for thread safety |

---

## 5. Practical Code Recipes

### Recipe 1: Quick Polling with Madgwick AHRS

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    // Initialize on SDA=21, SCL=22 at 400 kHz Fast-Mode
    if (!imu.begin(21, 22, 400000)) {
        Serial.println("Failed to initialize BMI160!");
        while (1) delay(100);
    }

    Serial.println("Keep sensor static... Calibrating...");
    imu.calibrate(200);
    Serial.println("Ready!");
}

void loop() {
    Motion m = imu.read();

    Serial.printf("Pitch: %6.1f° | Roll: %6.1f° | Yaw: %6.1f° | Temp: %.1f°C\n",
                  m.pitch, m.roll, m.yaw, m.temperature);
    delay(20);
}
```

---

### Recipe 2: Calibration, NVS Persistence, and Instant Taring

```cpp
#include <Arduino.h>
#include <Preferences.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;
Preferences prefs;

void setup() {
    Serial.begin(115200);
    imu.begin(21, 22);

    prefs.begin("imu_calib", false);

    if (prefs.isKey("calib_valid")) {
        // Restore pre-calibrated values instantly
        float bx = prefs.getFloat("bx", 0.0f);
        float by = prefs.getFloat("by", 0.0f);
        float bz = prefs.getFloat("bz", 0.0f);
        float tp = prefs.getFloat("tp", 0.0f);
        float tr = prefs.getFloat("tr", 0.0f);
        float ty = prefs.getFloat("ty", 0.0f);

        imu.setGyroOffsets(bx, by, bz);
        imu.setTare(tp, tr, ty);
        Serial.println("Restored calibration from NVS.");
    } else {
        // Run full calibration and persist
        imu.calibrate(200);
        imu.tare();

        float bx, by, bz, tp, tr, ty;
        imu.getGyroOffsets(bx, by, bz);
        imu.getTare(tp, tr, ty);

        prefs.putFloat("bx", bx);
        prefs.putFloat("by", by);
        prefs.putFloat("bz", bz);
        prefs.putFloat("tp", tp);
        prefs.putFloat("tr", tr);
        prefs.putFloat("ty", ty);
        prefs.putBool("calib_valid", true);
        Serial.println("Saved new calibration to NVS.");
    }
}

void loop() {
    Motion m = imu.read();
    delay(20);
}
```
