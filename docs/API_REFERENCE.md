# BMI160_ESPx API Reference

An Arduino and PlatformIO C++ library for the Bosch BMI160 6-axis IMU on ESP32 microcontrollers.

---

## Table of Contents

1. [Coordinate System](#1-coordinate-system)
2. [Data Structures & Enumerations](#2-data-structures--enumerations)
3. [Method Reference](#3-method-reference)
   - [Constructor & Destructor](#constructor--destructor)
   - [Initialization & Bus Recovery](#initialization--bus-recovery)
   - [Motion Reading](#motion-reading)
   - [Direct Accessors](#direct-accessors)
   - [Sensor Configuration](#sensor-configuration)
   - [Orientation Filters](#orientation-filters)
   - [Coordinate Frame Remapping & Axis Inversion](#coordinate-frame-remapping--axis-inversion)
   - [Tare Zeroing](#tare-zeroing)
   - [Calibration & Bias Compensation](#calibration--bias-compensation)
   - [Digital Filtering](#digital-filtering)
   - [Diagnostics & Register Access](#diagnostics--register-access)
4. [Internal State Variables](#4-internal-state-variables)
5. [Examples](#5-examples)

---

## 1. Coordinate System

### Coordinate Conventions
* **Sensor Frame:** Right-handed Cartesian coordinate system:
  * **+X:** Towards connector / forward
  * **+Y:** Left
  * **+Z:** Upwards out of the sensor package (+1g when laying flat on a table)
* **Attitude Angles:**
  * **Pitch:** Rotation around lateral Y-axis (nose up/down). Range: -90.0° to +90.0° (or -π/2 to +π/2 rad).
  * **Roll:** Rotation around longitudinal X-axis (banking left/right). Range: -180.0° to +180.0° (or -π to +π rad).
  * **Yaw:** Rotation around vertical Z-axis (heading). Range: -180.0° to +180.0° (or -π to +π rad).
* **Quaternions:** Hamilton unit quaternion convention `[qw, qx, qy, qz]` where `qw² + qx² + qy² + qz² = 1.0`.

### Data Flow
When `read()` is called:
```text
1. Burst read (0x0C - 0x17)
   │
2. Scale conversion (LSB to g and deg/s)
   │
3. Axis remapping (setAxisRemap)
   │
4. Gyro bias subtraction (calibrate / setGyroOffsets)
   │
5. Deadband (ZUPT) and dynamic acceleration gating
   │
6. Orientation filter (Complementary or Madgwick)
   │
7. Angle calculation (Roll, Pitch, Yaw, Quaternions)
   │
8. Axis sign inversion (invertPitch / invertRoll / invertYaw)
   │
9. Tare offset subtraction (tare / setTare)
```

---

## 2. Data Structures & Enumerations

All types reside in namespace `BMI160_ESPx_Types` and are aliased into the global scope in `BMI160_ESPx.h`.

### `Motion`
Telemetry and attitude structure.
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

    // Orientation Euler angles in degrees (or rad)
    float pitch = 0.0f;
    float roll  = 0.0f;
    float yaw   = 0.0f;

    // Unit quaternion (Hamilton convention: qw + qx*i + qy*j + qz*k)
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    // Die temperature in °C
    float temperature = 0.0f;

    // Timestamp in microseconds (esp_timer_get_time())
    uint64_t timestamp_us = 0;
};
```

### `RawMotion`
Structure for uncalibrated 16-bit register values.
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
    COMPLEMENTARY = 0, // Tilt filter
    MADGWICK      = 1  // Quaternion AHRS filter
};
```

### `FilterBandwidth`
```cpp
enum class FilterBandwidth : uint8_t {
    OSR4   = 0x00, // 4× oversampling
    OSR2   = 0x10, // 2× oversampling
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

## 3. Method Reference

### Constructor & Destructor

#### `explicit BMI160_ESPx(TwoWire &wire = Wire)`
* **Description:** Initializes the driver instance and allocates the FreeRTOS recursive mutex.
* **Parameters:** `wire` - Reference to TwoWire peripheral (defaults to `Wire`).

#### `virtual ~BMI160_ESPx()`
* **Description:** Cleans up the instance and deallocates the mutex.

---

### Initialization & Bus Recovery

#### `bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0)`
* **Description:** Initializes communication with the sensor.
  1. Checks SDA and SCL lines. If SDA is pulled low by a stuck device, generates 9 clock pulses on SCL to clear it.
  2. Probes addresses `0x68` and `0x69` for Chip ID `0xD1`.
  3. Soft-resets the chip (`0xB6`).
  4. Powers on Accelerometer and Gyroscope in normal mode.
  5. Sets defaults: ±2g, ±250 °/s, 100 Hz ODR.
  6. Sets the I2C clock frequency.

---

### Motion Reading

#### `bool read(Motion &m)`
* **Description:** Reads gyro and accel registers (`0x0C` - `0x17`), converts to engineering units, updates orientation filters, and populates `m`.
* **Returns:** `true` on success; `false` on bus error or timeout.

#### `Motion read()`
* **Description:** Value-returning overload of `read(Motion &m)`.

#### `const Motion& getLastMotion() const`
* **Description:** Returns a reference to the `Motion` struct from the most recent read.

#### `float getTemperature()`
* **Description:** Reads die temperature in °C. Rate-limited to 500 ms internally.

---

### Direct Accessors

Convenience methods to read the latest cached values:
* `float getPitch() const`
* `float getRoll() const`
* `float getYaw() const`
* `float getAccelX() const`, `float getAccelY() const`, `float getAccelZ() const`
* `float getGyroX() const`, `float getGyroY() const`, `float getGyroZ() const`

---

### Sensor Configuration

#### `bool setAccelRange(uint8_t g = 2)` / `bool setAccelRange(RangeG range)`
* **Description:** Sets accelerometer range: 2, 4, 8, or 16 g.

#### `bool setGyroRange(uint16_t dps = 250)` / `bool setGyroRange(RangeDPS range)`
* **Description:** Sets gyroscope range: 125, 250, 500, 1000, or 2000 °/s.

#### `bool setDataRate(uint16_t hz = 100)`
* **Description:** Sets output data rate: 25, 50, 100, 200, 400, 800, or 1600 Hz.

#### `void setDegrees(bool enable_degrees = true)`
* **Description:** `true` for degrees (° and deg/s); `false` for radians.

---

### Orientation Filters

#### `void setFusionAlgorithm(FusionMode mode = FusionMode::MADGWICK)`
* **Description:** Selects active algorithm (`FusionMode::MADGWICK` or `FusionMode::COMPLEMENTARY`).

#### `void setMadgwickBeta(float beta = 0.1f)`
* **Description:** Filter gain beta for Madgwick algorithm.

#### `void setFilterWeight(float alpha = 0.98f)`
* **Description:** Weight factor alpha for Complementary filter.

#### `void setDynamicAccelRejection(bool enable)`
* **Description:** Ignores accelerometer correction during periods of linear acceleration.

#### `void setZUPT(bool enable, float threshold_dps = 0.35f)`
* **Description:** Zero-velocity update: clamps gyro rates below `threshold_dps` to zero when stationary to reduce yaw drift.

#### `void resetOrientation()`
* **Description:** Resets orientation angles to 0 and re-aligns quaternion to gravity vector.

---

### Coordinate Frame Remapping & Axis Inversion

#### `void setAxisRemap(int8_t x_axis = 1, int8_t y_axis = 2, int8_t z_axis = 3)`
* **Description:** Remaps sensor axes (+/-1, +/-2, +/-3) to match board orientation.

#### `void invertPitch(bool invert = true)` / `invertRoll(bool invert = true)` / `invertYaw(bool invert = true)`
* **Description:** Inverts angle sign.

---

### Tare Zeroing

#### `void tare()`
* **Description:** Sets current angles as zero reference. Subsequent reads will be offset relative to this reference.

#### `void setTare(float pitch_offset, float roll_offset, float yaw_offset)`
* **Description:** Manually sets tare offsets (useful when restoring calibration from storage).

#### `void getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const`
* **Description:** Reads active tare offsets.

#### `void resetTare()`
* **Description:** Clears tare offsets to zero.

---

### Calibration & Bias Compensation

#### `void calibrate(uint16_t samples = 200)`
* **Description:** Takes samples while the sensor is at rest, averages gyro zero-rate biases, and aligns attitude to gravity. Sensor must remain still.

#### `void setGyroOffsets(float bx, float by, float bz)`
* **Description:** Manually sets gyro bias offsets (in deg/s).

#### `void getGyroOffsets(float &bx, float &by, float &bz) const`
* **Description:** Reads current gyro bias offsets.

---

### Digital Filtering

#### `bool setDLPF(FilterBandwidth bw = FilterBandwidth::OSR4)`
* **Description:** Sets hardware oversampling mode (`OSR4`, `OSR2`, or `NORMAL`).

---

### Diagnostics & Register Access

#### `ErrorCode getLastError() const`
* **Description:** Returns the last error recorded by the driver.

#### `bool isConnected()`
* **Description:** Checks if sensor responds with chip ID `0xD1`.

#### `bool softReset()`
* **Description:** Issues software reset (`0xB6`) and waits 100 ms.

#### `bool readRaw(RawMotion &raw)`
* **Description:** Reads raw uncalibrated 16-bit integers without running orientation filters.

#### `bool writeRegister(uint8_t reg, uint8_t value)`
* **Description:** Writes a single byte to a register.

#### `bool readRegister(uint8_t reg, uint8_t &value)`
* **Description:** Reads a single byte from a register.

#### `bool readBurst(uint8_t start_reg, uint8_t *buf, size_t length)`
* **Description:** Multi-byte burst read from consecutive registers.

#### `uint8_t getAddress() const`
* **Description:** Returns detected I2C address (`0x68` or `0x69`).

---

## 4. Internal State Variables

| Variable | Type | Description |
| :--- | :--- | :--- |
| `_wire` | `TwoWire&` | Reference to I2C peripheral |
| `_addr` | `uint8_t` | I2C address (`0x68` or `0x69`) |
| `_sda`, `_scl` | `int` | Configured GPIO pins |
| `_freq` | `uint32_t` | Configured I2C bus clock in Hz |
| `_use_degrees` | `bool` | Output units (`true` = degrees; `false` = radians) |
| `_fusion_mode` | `FusionMode` | Orientation algorithm (`MADGWICK` or `COMPLEMENTARY`) |
| `_active_bw` | `FilterBandwidth` | DLPF mode |
| `_active_odr_hz` | `uint16_t` | Output data rate in Hz |
| `_last_error` | `ErrorCode` | Last recorded error |
| `_dynamic_accel_rejection` | `bool` | Dynamic acceleration gating enable |
| `_zero_velocity_update` | `bool` | Zero-velocity update (ZUPT) enable |
| `_zupt_threshold_dps` | `float` | Angular velocity threshold for ZUPT clamping |
| `_accel_scale` | `float` | LSB/g conversion factor |
| `_gyro_scale` | `float` | LSB/(deg/s) conversion factor |
| `_bias_gx`, `_bias_gy`, `_bias_gz` | `float` | Gyro zero-rate bias offsets |
| `_filter_alpha` | `float` | Complementary filter alpha |
| `_madgwick_beta` | `float` | Madgwick filter beta |
| `_pitch`, `_roll`, `_yaw` | `float` | Current attitude angles |
| `_remap_x`, `_remap_y`, `_remap_z` | `int8_t` | Axis remapping settings |
| `_invert_pitch`, `_invert_roll`, `_invert_yaw` | `bool` | Angle sign inversion flags |
| `_tare_pitch`, `_tare_roll`, `_tare_yaw` | `float` | Tare offset angles |
| `_q0`, `_q1`, `_q2`, `_q3` | `float` | Madgwick quaternion state |
| `_last_filter_us` | `uint64_t` | Timestamp of previous filter update |
| `_cached_temp` | `float` | Cached die temperature |
| `_last_temp_read_us` | `uint64_t` | Timestamp of previous temperature read |
| `_last_motion` | `Motion` | Telemetry from most recent read |
| `_mutex` | `SemaphoreHandle_t` | FreeRTOS recursive mutex |

---

## 5. Examples

### Example 1: Basic Reading with Madgwick Filter

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    if (!imu.begin(21, 22, 400000)) {
        Serial.println("BMI160 initialization failed.");
        while (1) delay(100);
    }

    Serial.println("Calibrating (keep still)...");
    imu.calibrate(200);
    Serial.println("Ready.");
}

void loop() {
    Motion m = imu.read();
    Serial.printf("Pitch: %6.1f° | Roll: %6.1f° | Yaw: %6.1f° | Temp: %.1f°C\n",
                  m.pitch, m.roll, m.yaw, m.temperature);
    delay(20);
}
```

---

### Example 2: Saving Calibration to Preferences (NVS)

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
        // Restore saved offsets
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
        // Calibrate and save
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
        Serial.println("Saved calibration to NVS.");
    }
}

void loop() {
    Motion m = imu.read();
    delay(20);
}
```
