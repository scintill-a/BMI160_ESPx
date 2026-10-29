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
   - [Digital Filtering & Power Management](#digital-filtering--power-management)
   - [Hardware FIFO Buffer](#hardware-fifo-buffer)
   - [Hardware Pedometer (Step Counter)](#hardware-pedometer-step-counter)
   - [Hardware Interrupts](#hardware-interrupts)
   - [Diagnostics & Low-Level Register Access](#diagnostics--low-level-register-access)
   - [FreeRTOS Asynchronous Background Task](#freertos-asynchronous-background-task)
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

    // Gyroscope angular velocity (deg/s or rad/s)
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    // 6-DOF Orientation Euler Angles (deg or rad)
    float pitch = 0.0f;
    float roll  = 0.0f;
    float yaw   = 0.0f;

    // Unit Orientation Quaternion (Hamilton convention)
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    // Die temperature in degrees Celsius (°C)
    float temperature = 0.0f;

    // Microsecond timestamp from esp_timer_get_time()
    uint64_t timestamp_us = 0;
};
```

### `RawMotion`
Unscaled, uncalibrated 16-bit signed integers directly from hardware registers `0x0C` through `0x17`.
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
Algorithm selection for 6-DOF attitude estimation:
```cpp
enum class FusionMode : uint8_t {
    COMPLEMENTARY = 0, // 2-axis tilt filter combining accelerometer gravity and integrated gyro rate
    MADGWICK      = 1  // 6-DOF gradient-descent quaternion AHRS filter
};
```

### `RangeG`
Strongly-typed accelerometer full-scale measurement ranges:
```cpp
enum class RangeG : uint8_t {
    G_2  = 2,  // ±2g  (16384 LSB/g)
    G_4  = 4,  // ±4g  (8192 LSB/g)
    G_8  = 8,  // ±8g  (4096 LSB/g)
    G_16 = 16  // ±16g (2048 LSB/g)
};
```

### `RangeDPS`
Strongly-typed gyroscope full-scale measurement ranges:
```cpp
enum class RangeDPS : uint16_t {
    DPS_125  = 125,  // ±125 deg/s  (262.4 LSB/(deg/s))
    DPS_250  = 250,  // ±250 deg/s  (131.2 LSB/(deg/s))
    DPS_500  = 500,  // ±500 deg/s  (65.6 LSB/(deg/s))
    DPS_1000 = 1000, // ±1000 deg/s (32.8 LSB/(deg/s))
    DPS_2000 = 2000  // ±2000 deg/s (16.4 LSB/(deg/s))
};
```

### `FilterBandwidth`
Hardware Digital Low-Pass Filter (DLPF) oversampling ratios:
```cpp
enum class FilterBandwidth : uint8_t {
    OSR4   = 0x00, // 4× oversampling (maximum vibration attenuation)
    OSR2   = 0x10, // 2× oversampling (balanced filter latency)
    NORMAL = 0x20  // Normal mode (cutoff frequency = 0.5 × ODR; lowest group delay)
};
```

### `PowerMode`
Power states for internal sensor cores:
```cpp
enum class PowerMode : uint8_t {
    NORMAL,    // Full performance: both accelerometer and gyroscope active
    LOW_POWER, // Low power: accelerometer duty-cycled, gyroscope suspended
    SUSPEND    // Deep sleep standby (< 5 μA): all sensor cores suspended
};
```

### `ErrorCode`
Diagnostic error flags recorded during transactions:
```cpp
enum class ErrorCode : uint8_t {
    NONE = 0,           // Operation succeeded
    ERR_NO_PULLUP,      // Bus line held LOW or missing pull-up resistors
    ERR_CHIP_NOT_FOUND, // Sensor chip ID != 0xD1
    ERR_RESET_TIMEOUT,  // Soft reset did not finish in time
    ERR_I2C_NACK,       // I2C transmission NACKed by slave
    ERR_MUTEX_TIMEOUT,  // FreeRTOS mutex acquisition timed out
    ERR_FIFO_OVERFLOW   // Hardware FIFO buffer overflowed
};
```

---

## 3. Complete Method Reference

### Constructor & Destructor

#### `BMI160_ESPx(TwoWire &wire = Wire)`
* **Description:** Instantiates the driver and initializes the FreeRTOS synchronization mutex.
* **Parameters:**
  * `wire`: Reference to the `TwoWire` I2C peripheral instance (defaults to Arduino `Wire`).

#### `virtual ~BMI160_ESPx()`
* **Description:** Safely terminates any active background FreeRTOS streaming worker and deletes the synchronization mutex.

---

### Initialization & Bus Recovery

#### `bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0)`
* **Description:** Connection and initialization routine:
  1. Executes physical line pre-check. If SDA is clamped LOW by a hung slave, pulses SCL 9 times followed by a STOP condition.
  2. Probes chip ID (`0xD1`) at address `0x68` or `0x69` (auto-detects if `address = 0`).
  3. Executes software reset and delays for bootloader restart.
  4. Powers on accelerometer and gyroscope in Normal mode.
  5. Configures ±2g, ±250 deg/s, 100 Hz ODR defaults.
  6. Escalates I2C clock to `frequency` (e.g. 400 kHz Fast-Mode).
  7. Resets attitude filter states.
* **Returns:** `true` on success; `false` if line held LOW, chip ID mismatch, or I2C NACK.

---

### Motion Reading & Telemetry

#### `bool read(Motion &m)`
* **Description:** Thread-safe 12-byte burst read of registers `0x0C` through `0x17`. Converts raw units, subtracts gyro biases, applies coordinate remapping, runs the active attitude fusion filter, applies tare offsets, and refreshes the internal temperature cache.
* **Parameters:** `[out] m` - Reference to destination `Motion` struct.
* **Returns:** `true` on successful read; `false` on bus error.

#### `Motion read()`
* **Description:** Value-returning overload for inline assignment (`Motion m = imu.read();`).

#### `const Motion& getLastMotion() const`
* **Description:** Returns a read-only reference to the cached `Motion` packet populated by the most recent `read()` call without issuing any new I2C transactions.

---

### Direct Accessor Getters

All getters return values cached from the most recent `read()` execution:

| Method | Return Type | Unit | Description |
| :--- | :--- | :--- | :--- |
| `getPitch()` | `float` | deg or rad | Pitch angle (rotation around Y-axis) |
| `getRoll()` | `float` | deg or rad | Roll angle (rotation around X-axis) |
| `getYaw()` | `float` | deg or rad | Yaw angle (rotation around Z-axis) |
| `getAccelX()` | `float` | g | Calibrated linear acceleration X |
| `getAccelY()` | `float` | g | Calibrated linear acceleration Y |
| `getAccelZ()` | `float` | g | Calibrated linear acceleration Z |
| `getGyroX()` | `float` | deg/s or rad/s | Calibrated angular velocity X |
| `getGyroY()` | `float` | deg/s or rad/s | Calibrated angular velocity Y |
| `getGyroZ()` | `float` | deg/s or rad/s | Calibrated angular velocity Z |
| `getTemperature()` | `float` | °C | Internal die temperature (cached every 500 ms) |

---

### Sensor Configuration & Ranges

#### `bool setAccelRange(uint8_t g = 2)`
* **Description:** Sets accelerometer full-scale range using natural numbers.
* **Supported values:** `2` (±2g), `4` (±4g), `8` (±8g), `16` (±16g).
* **Returns:** `true` if valid and written to hardware; `false` otherwise.

#### `bool setAccelRange(RangeG range)`
* **Description:** Strongly-typed overload (`RangeG::G_2`, `RangeG::G_4`, `RangeG::G_8`, `RangeG::G_16`).

#### `bool setGyroRange(uint16_t dps = 250)`
* **Description:** Sets gyroscope full-scale angular rate range using natural numbers.
* **Supported values:** `125`, `250`, `500`, `1000`, `2000` deg/s.
* **Returns:** `true` if valid and written to hardware; `false` otherwise.

#### `bool setGyroRange(RangeDPS range)`
* **Description:** Strongly-typed overload (`RangeDPS::DPS_125`, `DPS_250`, `DPS_500`, `DPS_1000`, `DPS_2000`).

#### `bool setDataRate(uint16_t hz = 100)`
* **Description:** Configures sensor Output Data Rate (ODR) for both accelerometer and gyroscope while preserving the active DLPF oversampling mode.
* **Supported targets:** `25`, `50`, `100`, `200`, `400`, `800`, `1600` Hz.

#### `void setDegrees(bool enable_degrees = true)`
* **Description:** Configures angle units for gyro rates (`gx, gy, gz`) and attitude angles (`pitch, roll, yaw`). Accelerometer data remains in units of g. Pass `false` for radians.

---

### Sensor Fusion & AHRS Algorithms

#### `void setFusionAlgorithm(FusionMode mode = FusionMode::MADGWICK)`
* **Description:** Switches between `FusionMode::COMPLEMENTARY` and `FusionMode::MADGWICK`. Automatically calls `resetOrientation()` upon switching to avoid discontinuity.

#### `void setFilterWeight(float alpha = 0.98f)`
* **Description:** Sets the weighting coefficient for the Complementary filter (`0.0f` to `1.0f`). Higher values trust the gyroscope more; lower values trust the gravity vector more.

#### `void setMadgwickBeta(float beta = 0.1f)`
* **Description:** Sets the filter gain `beta` (rad/s) for Madgwick AHRS. Higher values speed up gravity alignment; lower values provide smoother, vibration-immune attitude tracking.

#### `void setDynamicAccelRejection(bool enable)`
* **Description:** Enables or disables gravity gating. When `|accel|` falls outside `0.80g` - `1.20g` (e.g., during shock or rapid acceleration), the attitude filter attenuates accelerometer corrections and relies purely on gyroscope integration.

#### `void setZUPT(bool enable, float threshold_dps = 0.35f)`
* **Description:** Enables or disables Zero-Velocity Update (ZUPT). Gyro rates below `threshold_dps` are clamped to zero when stationary to eliminate heading drift.

#### `void resetOrientation()`
* **Description:** Resets orientation angles (`pitch, roll, yaw`) to `0.0`, reinitializes the unit quaternion to identity `[1, 0, 0, 0]`, and resets filter timestamp tracking.

---

### Coordinate Frame Remapping & Axis Inversion

#### `void setAxisRemap(int8_t x_axis = 1, int8_t y_axis = 2, int8_t z_axis = 3)`
* **Description:** Remaps physical sensor axes into the chassis body frame before fusion is computed.
* **Axis codes:**
  * `+1` = +X, `-1` = -X
  * `+2` = +Y, `-2` = -Y
  * `+3` = +Z, `-3` = -Z
* **Common mounting configurations:**
  * Flat, default: `setAxisRemap(1, 2, 3)`
  * Rotated 90° clockwise: `setAxisRemap(2, -1, 3)`
  * Rotated 180°: `setAxisRemap(-1, -2, 3)`
  * Rotated 270° clockwise: `setAxisRemap(-2, 1, 3)`
  * Upside-down: `setAxisRemap(1, -2, -3)`

#### `void invertPitch(bool invert = true)`
* **Description:** Inverts pitch sign (`pitch = -pitch`).

#### `void invertRoll(bool invert = true)`
* **Description:** Inverts roll sign (`roll = -roll`).

#### `void invertYaw(bool invert = true)`
* **Description:** Inverts yaw sign (`yaw = -yaw`).

#### `bool isPitchInverted() const` / `bool isRollInverted() const` / `bool isYawInverted() const`
* **Description:** Returns the active inversion status for pitch, roll, or yaw.

---

### Tare Zeroing & Orientation Offsets

#### `void tare()`
* **Description:** Captures the current physical orientation (in the active coordinate frame, after axis inversion) and locks it in as the zero reference. All subsequent readings report `(0, 0, 0)` at this physical position.

#### `void setTare(float pitch_offset, float roll_offset, float yaw_offset)`
* **Description:** Manually overrides the tare offset angles (in degrees or radians). Useful for restoring stored offsets from NVS/EEPROM.

#### `void getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const`
* **Description:** Retrieves the active tare offsets in degrees or radians.

#### `void getRawOrientation(float &raw_pitch, float &raw_roll, float &raw_yaw) const`
* **Description:** Retrieves the un-tared, un-inverted internal attitude angles.

#### `void resetTare()`
* **Description:** Clears all tare offsets back to `0.0`.

---

### Calibration & Bias Compensation

#### `void calibrate(uint16_t samples = 200, bool run_hardware_foc = false)`
* **Description:** Joint calibration procedure:
  1. Optionally runs on-chip Fast Offset Compensation (FOC).
  2. Samples 200 consecutive 12-byte raw bursts (accelerometer + gyroscope) while stationary.
  3. Computes gyroscope stationary zero-rate biases.
  4. Averages the stationary gravity vector and analytically calculates initial pitch and roll:
     ```text
     roll  = atan2(ay, az)
     pitch = atan2(-ax, √(ay² + az²))
     ```
  5. Initializes the Madgwick quaternion and Complementary state directly to this resting equilibrium.
* **Note:** Eliminates the need for external post-calibration settling loops.

#### `void setGyroOffsets(float bx, float by, float bz)`
* **Description:** Manually sets stationary gyro zero-rate bias offsets (in deg/s). Useful for restoring factory calibration from NVS.

#### `void getGyroOffsets(float &bx, float &by, float &bz) const`
* **Description:** Retrieves active stationary gyro zero-rate bias offsets (in deg/s).

#### `bool runHardwareFOC(bool z_up = true)`
* **Description:** Triggers the internal Bosch BMI160 Fast Offset Compensation engine. Offsets are trimmed directly into on-chip registers `0x71` - `0x76`.
* **Parameters:** `z_up` - `true` if laying flat with Z pointing up; `false` if upside-down.

---

### Digital Filtering & Power Management

#### `bool setDLPF(FilterBandwidth bw = FilterBandwidth::OSR4)`
* **Description:** Configures the hardware Digital Low-Pass Filter anti-aliasing mode:
  * `FilterBandwidth::OSR4`: 4× oversampling (best vibration damping for motors/propellers).
  * `FilterBandwidth::OSR2`: 2× oversampling (balanced response).
  * `FilterBandwidth::NORMAL`: Normal mode (cutoff frequency = 0.5 × ODR; minimum phase lag).

#### `bool setPowerMode(PowerMode mode = PowerMode::NORMAL)`
* **Description:** Configures sensor power state (`PowerMode::NORMAL`, `PowerMode::LOW_POWER`, or `PowerMode::SUSPEND`).

---

### Hardware FIFO Buffer

#### `bool enableFIFO(bool accel = true, bool gyro = true)`
* **Description:** Enables the 1024-byte on-chip hardware FIFO queue. Automatically flushes old data.

#### `uint16_t getFIFOCount()`
* **Description:** Returns the total number of unread data bytes currently waiting in the FIFO buffer.

#### `uint16_t readFIFO(Motion *buffer, uint16_t max_samples)`
* **Description:** Unpacks 12-byte raw frames from FIFO into the provided array of `Motion` structs, converts units, and subtracts gyro biases.
* **Returns:** Number of complete frames unpacked.

#### `void flushFIFO()`
* **Description:** Clears all queued frames and resets the FIFO read pointer.

---

### Hardware Pedometer (Step Counter)

#### `void enablePedometer(bool enable = true)`
* **Description:** Activates the internal Bosch step detector engine. Step counting runs autonomously on-chip with 0% ESP32 CPU load.

#### `uint16_t getSteps()`
* **Description:** Reads the total accumulated step count from registers `0x78` and `0x79`.

#### `void resetSteps()`
* **Description:** Resets the on-chip step counter back to zero.

---

### Hardware Interrupts

#### `bool enableMotionInterrupt(uint8_t threshold_lsb = 20, int int_pin = 1)`
* **Description:** Configures the hardware Any-Motion (slope) interrupt on pin `INT1` or `INT2`. At ±2g, `1 LSB ≈ 3.91 mg`.

#### `bool enableTapInterrupt(bool double_tap = true, uint8_t threshold_lsb = 10, int int_pin = 1)`
* **Description:** Configures the hardware Single-Tap or Double-Tap gesture interrupt on pin `INT1` or `INT2`.

---

### Diagnostics & Low-Level Register Access

#### `uint8_t scanBus()`
* **Description:** Thread-safe diagnostic scan across all 127 I2C addresses. Prints discovered devices to `Serial`.
* **Returns:** Total count of responsive I2C devices found.

#### `ErrorCode getLastError() const`
* **Description:** Returns the last fault recorded by the driver (`ErrorCode::NONE`, `ERR_NO_PULLUP`, `ERR_I2C_NACK`, etc.).

#### `bool isConnected()`
* **Description:** Verifies if the sensor is responding with chip ID `0xD1`.

#### `bool softReset()`
* **Description:** Issues `CMD_SOFT_RESET` (`0xB6`) and waits 100 ms for reboot.

#### `bool readRaw(RawMotion &raw)`
* **Description:** Reads raw uncalibrated 16-bit integers in a single 12-byte burst without running fusion filters.

#### `bool writeRegister(uint8_t reg, uint8_t value)`
* **Description:** Writes an 8-bit byte to a BMI160 hardware register.

#### `bool readRegister(uint8_t reg, uint8_t &value)`
* **Description:** Reads an 8-bit byte from a BMI160 hardware register.

#### `bool readBurst(uint8_t start_reg, uint8_t *buf, size_t length)`
* **Description:** Multi-byte burst read leveraging auto-incrementing hardware registers.

#### `uint8_t getAddress() const`
* **Description:** Returns the active 7-bit I2C address (`0x68` or `0x69`).

---

### FreeRTOS Asynchronous Background Task

#### `bool startAsyncStream(uint16_t rate_hz, QueueHandle_t out_queue, BaseType_t core = 1)`
* **Description:** Spawns a dedicated FreeRTOS worker task pinned to `core` (default: Core 1). The task executes `read()` periodically using `vTaskDelayUntil()` and enqueues `Motion` packets directly into `out_queue`.
* **Parameters:**
  * `rate_hz`: Sampling frequency (e.g., 50, 100, 200 Hz).
  * `out_queue`: Handle created via `xQueueCreate(depth, sizeof(Motion))`.
  * `core`: CPU core index (`0` or `1`).
* **Returns:** `true` if task was started; `false` on invalid parameters or memory error.

#### `void stopAsyncStream()`
* **Description:** Signals the background worker task to exit, waits for task deletion, and cleans up handles.

#### `bool isAsyncStreamActive() const`
* **Description:** Returns `true` if the background streaming task is currently running.

---

## 4. Internal Attributes & State Variables

The following private member variables govern the internal state of `BMI160_ESPx`:

| Variable | Type | Description |
| :--- | :--- | :--- |
| `_wire` | `TwoWire&` | Reference to active I2C peripheral instance |
| `_addr` | `uint8_t` | Active 7-bit I2C slave address (`0x68` or `0x69`) |
| `_sda`, `_scl` | `int` | Configured ESP32 GPIO pins for I2C data and clock |
| `_freq` | `uint32_t` | Configured I2C bus clock speed in Hz (default: 400,000 Hz) |
| `_use_degrees` | `bool` | Output units flag (`true` = degrees and deg/s; `false` = radians) |
| `_fusion_mode` | `FusionMode` | Active attitude fusion algorithm (`MADGWICK` or `COMPLEMENTARY`) |
| `_active_bw` | `FilterBandwidth` | Active DLPF oversampling ratio |
| `_active_odr_hz` | `uint16_t` | Cached sensor output data rate (preserves rate during DLPF changes) |
| `_last_error` | `ErrorCode` | Most recent diagnostic error code |
| `_dynamic_accel_rejection` | `bool` | Dynamic acceleration gating enable flag |
| `_zero_velocity_update` | `bool` | Zero-Velocity Update (ZUPT) deadband enable flag |
| `_zupt_threshold_dps` | `float` | Angular velocity threshold for ZUPT clamping (default: 0.35 deg/s) |
| `_accel_scale` | `float` | LSB/g conversion divisor (16384 for ±2g, 8192 for ±4g, etc.) |
| `_gyro_scale` | `float` | LSB/(deg/s) conversion divisor (131.2 for ±250 deg/s, etc.) |
| `_bias_gx`, `_bias_gy`, `_bias_gz` | `float` | Stationary gyroscope zero-rate bias offsets in deg/s |
| `_filter_alpha` | `float` | Complementary filter weighting coefficient (default: 0.98) |
| `_madgwick_beta` | `float` | Madgwick filter divergence gain in rad/s (default: 0.1) |
| `_pitch`, `_roll`, `_yaw` | `float` | Cached attitude angles after axis inversion and tare subtraction |
| `_remap_x`, `_remap_y`, `_remap_z` | `int8_t` | Active axis remapping matrix codes (+1, -1, +2, -2, +3, -3) |
| `_invert_pitch`, `_invert_roll`, `_invert_yaw` | `bool` | Axis sign inversion flags |
| `_tare_pitch`, `_tare_roll`, `_tare_yaw` | `float` | Locked zero-reference offset angles |
| `_q0`, `_q1`, `_q2`, `_q3` | `float` | Madgwick unit orientation quaternion state vector [qw, qx, qy, qz] |
| `_last_filter_us` | `uint64_t` | Microsecond timestamp of previous filter execution for dt calculation |
| `_cached_temp` | `float` | Rate-limited die temperature cache |
| `_last_temp_read_us` | `uint64_t` | Microsecond timestamp of previous temperature register read |
| `_last_motion` | `Motion` | Complete telemetry packet from most recent read |
| `_mutex` | `SemaphoreHandle_t` | FreeRTOS mutex guaranteeing thread-safe I2C transactions |
| `_task_handle` | `TaskHandle_t` | FreeRTOS task handle for asynchronous background streamer |
| `_target_queue` | `QueueHandle_t` | FreeRTOS queue destination for background streamer |
| `_stream_hz` | `uint16_t` | Operating rate for asynchronous background streamer |
| `_stream_running` | `volatile bool` | Running state flag for background task lifecycle |

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

    // Joint stationary calibration: averages gyro biases and aligns attitude to gravity
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

---

### Recipe 3: Non-Blocking FreeRTOS Worker Task on Core 1

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;
QueueHandle_t imu_queue;

void setup() {
    Serial.begin(115200);
    imu.begin(21, 22);
    imu.calibrate(200);

    // Create a 10-packet queue
    imu_queue = xQueueCreate(10, sizeof(Motion));

    // Spawn 100 Hz background sampler pinned to Core 1
    imu.startAsyncStream(100, imu_queue, 1);
}

void loop() {
    Motion packet;
    // Consume packets without blocking the sensor sampling jitter
    if (xQueueReceive(imu_queue, &packet, portMAX_DELAY) == pdTRUE) {
        // Process telemetry packet
    }
}
```
