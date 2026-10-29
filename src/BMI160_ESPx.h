#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "BMI160_Registers.h"
#include "BMI160_Types.h"

using Motion          = BMI160_ESPx_Types::Motion;
using RawMotion       = BMI160_ESPx_Types::RawMotion;
using FusionMode      = BMI160_ESPx_Types::FusionMode;
using FilterBandwidth = BMI160_ESPx_Types::FilterBandwidth;
using PowerMode       = BMI160_ESPx_Types::PowerMode;
using ErrorCode       = BMI160_ESPx_Types::ErrorCode;

/**
 * @class BMI160_ESPx
 * @brief Industrial-grade, high-performance driver for the Bosch BMI160 6-DOF IMU on Espressif ESP32.
 * 
 * Features:
 * - Direct natural number configuration (`setAccelRange(2)`, `setGyroRange(250)`, `setDataRate(100)`).
 * - Real-time 6-DOF attitude estimation (Pitch, Roll, Yaw, Quaternions) via Complementary or Madgwick AHRS filters.
 * - Active 9-clock I2C bus recovery and line pre-check to prevent ESP32 Core 3.x bus lockups.
 * - Hardware Fast Offset Compensation (FOC) on-chip trim.
 * - FreeRTOS thread-safe operation with optional background worker task pinned to Core 1.
 * - Autonomous on-chip pedometer (step counter) and hardware interrupts.
 */
class BMI160_ESPx {
public:
    /**
     * @brief Constructs a new BMI160_ESPx driver instance.
     * 
     * Initializes driver state and creates a FreeRTOS mutex to guarantee thread-safe
     * I2C bus transactions across multiple FreeRTOS tasks and ESP32 cores.
     * 
     * @param wire Reference to the TwoWire I2C peripheral instance to communicate over.
     *             Defaults to the primary Arduino I2C bus (`Wire`).
     */
    explicit BMI160_ESPx(TwoWire &wire = Wire);

    /**
     * @brief Destructs the driver instance and safely releases FreeRTOS resources.
     * 
     * Stops the asynchronous FreeRTOS streaming task (if active) and deallocates
     * the internal I2C transaction synchronization mutex.
     */
    virtual ~BMI160_ESPx();

    // Quick-Start API (Simple 1-liners)
    
    /**
     * @brief Connects to the sensor, clears bus contention, and applies defaults (±2g, ±250 deg/s, 100 Hz).
     * 
     * Executes an 8-stage robust initialization sequence:
     * 1. Performs physical line pre-check with pull-ups. If SDA is clamped LOW by a hung slave,
     *    generates 9 clock pulses on SCL followed by a STOP condition to release the bus.
     * 2. Initializes `TwoWire` at 100 kHz for reliable device probing.
     * 3. Probes the device chip ID (expected `0xD1` at address `0x68` or `0x69`).
     *    If not found, executes a full I2C bus scan to assist the developer in troubleshooting wiring.
     * 4. Issues a software reset (`CMD_SOFT_RESET = 0xB6`) and delays for internal bootloader completion.
     * 5. Awakens Accelerometer and Gyroscope into Normal power mode.
     * 6. Configures standard high-performance defaults (±2g range, ±250 deg/s range, 100 Hz ODR).
     * 7. Accelerates the I2C peripheral clock to the requested operating frequency (e.g., 400 kHz).
     * 8. Resets orientation angles, quaternions, and filter timing state.
     * 
     * @param sda GPIO pin number connected to sensor SDA data line (default: 21).
     * @param scl GPIO pin number connected to sensor SCL clock line (default: 22).
     * @param frequency Desired operational I2C clock speed in Hz (default: 400,000 Hz / 400 kHz Fast-Mode).
     * @param address Specific 7-bit I2C slave address (`0x68` when SDO/SA0=GND, `0x69` when SDO/SA0=3.3V).
     *                Passing 0 triggers automatic probing of both `0x68` and `0x69` (default: 0).
     * @return `true` if sensor was detected, reset, awakened, and successfully initialized.
     * @return `false` if communication failed, pull-up resistors were missing, chip ID was invalid,
     *                 or mutex acquisition timed out.
     */
    bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0);

    /**
     * @brief Atomically reads high-speed motion telemetry and updates 6-DOF orientation.
     * 
     * Executes a thread-safe 12-byte burst read across gyro and accelerometer registers
     * (registers 0x0C through 0x17) in a single I2C transaction to eliminate sampling skew between axes.
     * Converts raw 16-bit values into physical units (g and deg/s or rad/s), subtracts calibrated gyro
     * zero-rate biases, calculates microsecond time delta (dt), and runs the configured sensor fusion filter
     * (Complementary or Madgwick AHRS) to update pitch, roll, yaw, and quaternion attitudes.
     * Also refreshes internal die temperature on a rate-limited 500 ms interval to conserve bus bandwidth.
     * 
     * @param[out] m Reference to a `Motion` struct where calibrated linear acceleration, angular rate,
     *               orientation angles, unit quaternions, temperature, and timestamp will be written.
     * @return `true` if 12-byte burst read succeeded and orientation was updated.
     * @return `false` if I2C bus transaction failed (NACK) or mutex acquisition timed out.
     */
    bool read(Motion &m);

    /**
     * @brief Direct motion return for clean one-line access.
     * 
     * Ergonomic value-returning overload of `read(Motion &m)`. Ideal for concise assignment
     * in polling loops (`Motion m = imu.read();`).
     * 
     * @return Motion Populated `Motion` struct containing the latest sensor measurements and attitude.
     *                Returns a zeroed/default struct if the read transaction fails.
     */
    Motion read();

    // Quick Direct Getters

    /**
     * @brief Gets the latest estimated pitch angle from the most recent `read()` call.
     * 
     * Pitch corresponds to rotation around the lateral/Y axis (nose up/down).
     * 
     * @return float Pitch angle in degrees (or radians if `setDegrees(false)` was configured).
     */
    float getPitch();

    /**
     * @brief Gets the latest estimated roll angle from the most recent `read()` call.
     * 
     * Roll corresponds to rotation around the longitudinal/X axis (banking left/right).
     * 
     * @return float Roll angle in degrees (or radians if `setDegrees(false)` was configured).
     */
    float getRoll();

    /**
     * @brief Gets the latest estimated yaw angle from the most recent `read()` call.
     * 
     * Yaw corresponds to heading / rotation around the vertical/Z axis, calculated by
     * integrating calibrated angular velocity over time.
     * @note Without a magnetometer reference, yaw angle will exhibit slow long-term drift.
     * 
     * @return float Yaw angle in degrees (or radians if `setDegrees(false)` was configured).
     */
    float getYaw();

    /**
     * @brief Gets the latest linear acceleration along the X-axis from the most recent `read()` call.
     * 
     * @return float Acceleration in units of g (1g ≈ 9.80665 m/s²).
     */
    float getAccelX();

    /**
     * @brief Gets the latest linear acceleration along the Y-axis from the most recent `read()` call.
     * 
     * @return float Acceleration in units of g (1g ≈ 9.80665 m/s²).
     */
    float getAccelY();

    /**
     * @brief Gets the latest linear acceleration along the Z-axis from the most recent `read()` call.
     * 
     * @return float Acceleration in units of g (1g ≈ 9.80665 m/s²).
     */
    float getAccelZ();

    /**
     * @brief Gets the latest calibrated angular velocity around the X-axis from the most recent `read()` call.
     * 
     * Stationary zero-rate bias offset is automatically subtracted.
     * 
     * @return float Angular velocity in deg/s (or rad/s if `setDegrees(false)` was configured).
     */
    float getGyroX();

    /**
     * @brief Gets the latest calibrated angular velocity around the Y-axis from the most recent `read()` call.
     * 
     * Stationary zero-rate bias offset is automatically subtracted.
     * 
     * @return float Angular velocity in deg/s (or rad/s if `setDegrees(false)` was configured).
     */
    float getGyroY();

    /**
     * @brief Gets the latest calibrated angular velocity around the Z-axis from the most recent `read()` call.
     * 
     * Stationary zero-rate bias offset is automatically subtracted.
     * 
     * @return float Angular velocity in deg/s (or rad/s if `setDegrees(false)` was configured).
     */
    float getGyroZ();

    /**
     * @brief Reads and returns the internal silicon die temperature.
     * 
     * Reads registers `REG_TEMP_L` (0x20) and `REG_TEMP_H` (0x21). The BMI160 internal sensor
     * provides 1/512 °C (0.001953125 °C/LSB) resolution centered at 23 °C.
     * 
     * @return float Die temperature in degrees Celsius (°C). Returns cached value if I2C bus is busy.
     */
    float getTemperature();
    const Motion& getLastMotion() const { return _last_motion; }

    // Ergonomic Customization (Plain Natural Numbers)

    /**
     * @brief Sets the full-scale measurement range of the accelerometer using natural numbers.
     * 
     * Writes to `REG_ACC_RANGE` (0x41) and updates internal scale factors:
     * - 2g:   16384 LSB/g (register code: 0x03) [Default]
     * - 4g:   8192 LSB/g  (register code: 0x05)
     * - 8g:   4096 LSB/g  (register code: 0x08)
     * - 16g:  2048 LSB/g  (register code: 0x0C)
     * 
     * @param g Full-scale range in units of g. Must be exactly 2, 4, 8, or 16. Default: 2 (±2g).
     * @return `true` if a valid range was supplied and successfully written to hardware.
     * @return `false` if an invalid range was provided or I2C write failed.
     */
    bool setAccelRange(uint8_t g = 2);

    /**
     * @brief Sets the full-scale measurement range of the gyroscope using natural numbers.
     * 
     * Writes to `REG_GYR_RANGE` (0x43) and updates internal scale factors:
     * - 125 deg/s:   262.4 LSB/(deg/s) (register code: 0x04)
     * - 250 deg/s:   131.2 LSB/(deg/s) (register code: 0x03) [Default]
     * - 500 deg/s:   65.6 LSB/(deg/s)  (register code: 0x02)
     * - 1000 deg/s:  32.8 LSB/(deg/s)  (register code: 0x01)
     * - 2000 deg/s:  16.4 LSB/(deg/s)  (register code: 0x00)
     * 
     * @param dps Full-scale angular rate range in deg/s. Must be exactly 125, 250, 500, 1000, or 2000. Default: 250 (±250 deg/s).
     * @return `true` if a valid range was supplied and successfully written to hardware.
     * @return `false` if an invalid range was provided or I2C write failed.
     */
    bool setGyroRange(uint16_t dps = 250);

    /**
     * @brief Sets the sensor Output Data Rate (ODR) for both accelerometer and gyroscope.
     * 
     * Writes to `REG_ACC_CONF` (0x40) and `REG_GYR_CONF` (0x42), combining the selected ODR
     * with the active digital low-pass filter (DLPF) bandwidth setting.
     * Supported ODR target values:
     * - <= 25 Hz:   25 Hz  (code: 0x06)
     * - <= 50 Hz:   50 Hz  (code: 0x07)
     * - <= 100 Hz:  100 Hz (code: 0x08) [Default]
     * - <= 200 Hz:  200 Hz (code: 0x09)
     * - <= 400 Hz:  400 Hz (code: 0x0A)
     * - <= 800 Hz:  800 Hz (code: 0x0B)
     * - > 800 Hz:   1600 Hz (code: 0x0C)
     * 
     * @param hz Desired output data rate in Hertz (e.g., 25, 50, 100, 200, 400, 800, 1600). Default: 100 (100 Hz).
     * @return `true` if both accelerometer and gyroscope configuration registers were updated.
     * @return `false` if I2C transmission failed or mutex timed out.
     */
    bool setDataRate(uint16_t hz = 100);

    /**
     * @brief Configures whether angular quantities are reported in degrees or radians.
     * 
     * Controls units for gyroscope telemetry (gx, gy, gz) and orientation angles (pitch, roll, yaw).
     * Accelerometer readings remain in g regardless of this setting.
     * 
     * @param enable_degrees `true` to output angles in degrees (° and deg/s),
     *                       `false` to output angles in radians (rad and rad/s).
     *                       Default is `true`.
     */
    void setDegrees(bool enable_degrees = true);

    // Sensor Fusion Configuration

    /**
     * @brief Selects the real-time sensor fusion algorithm used to calculate orientation.
     * 
     * Automatically calls `resetOrientation()` upon switching algorithms to prevent state discontinuity.
     * 
     * Available modes:
     * - `FusionMode::COMPLEMENTARY`: Ultra-fast, deterministic 2-axis tilt filter combining
     *                                accelerometer gravity vector with integrated gyro rates.
     *                                Negligible computational overhead.
     * - `FusionMode::MADGWICK`: Gradient-descent 6-DOF quaternion AHRS orientation filter.
     *                           Provides complete 3D spatial attitude representation with zero gimbal lock.
     * 
     * @param mode Target fusion algorithm (`FusionMode::COMPLEMENTARY` or `FusionMode::MADGWICK`). Default: `FusionMode::MADGWICK`.
     */
    void setFusionAlgorithm(FusionMode mode = FusionMode::MADGWICK);

    /**
     * @brief Tunes the weighting factor (alpha) for the Complementary filter.
     * 
     * In the complementary filter formulation:
     * `angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_tilt`
     * 
     * - Higher alpha (e.g., 0.98 - 0.99): Trusts gyro integration more; smoother output,
     *   immune to linear acceleration and vibration spikes, but slower recovery from long-term drift.
     * - Lower alpha (e.g., 0.90 - 0.95): Trusts accelerometer gravity tilt more; faster drift
     *   correction, but more sensitive to external mechanical vibration.
     * 
     * @param alpha Filter weighting factor in the range [0.0f, 1.0f]. Default: 0.98f.
     *              Values outside this range are ignored.
     */
    void setFilterWeight(float alpha = 0.98f);

    /**
     * @brief Tunes the filter gain (beta) for the Madgwick AHRS quaternion algorithm.
     * 
     * Beta represents the gyroscope measurement error expressed as the divergence rate
     * of the quaternion in rad/s.
     * 
     * - Higher beta: Faster convergence towards the accelerometer gravity vector, quicker
     *   correction of gyro bias, but higher susceptibility to accelerometer vibration noise.
     * - Lower beta: Smoother quaternion output, higher immunity to dynamic acceleration, but
     *   slower attitude convergence.
     * 
     * @param beta Filter gain in rad/s. Must be >= 0.0f. Default: 0.1f.
     */
    void setMadgwickBeta(float beta = 0.1f);

    /**
     * @brief Enables or disables dynamic acceleration rejection (gravity gating).
     * 
     * When enabled, the sensor fusion engine automatically attenuates or ignores corrupted
     * accelerometer vectors during rapid hand motion, flicking, or vibration (|accel| outside 0.80g - 1.20g),
     * relying purely on the gyroscope until steady motion resumes.
     * 
     * @param enable Set to `true` to enable dynamic rejection (default: `true`).
     */
    void setDynamicAccelRejection(bool enable) { _dynamic_accel_rejection = enable; }

    /**
     * @brief Enables or disables Zero-Velocity Update (ZUPT) / stationary gyro deadband.
     * 
     * When stationary on a surface, clamps small gyro noise to zero to prevent Yaw creep.
     * 
     * @param enable Set to `true` to enable ZUPT deadband (default: `true`).
     * @param threshold_dps Angular velocity threshold in deg/s (default: 0.35 deg/s).
     */
    void setZUPT(bool enable, float threshold_dps = 0.35f) {
        _zero_velocity_update = enable;
        _zupt_threshold_dps = threshold_dps;
    }

    /**
     * @brief Resets current orientation angles and quaternion state to baseline.
     * 
     * Clears pitch, roll, and yaw angles to 0.0, resets internal quaternion to identity
     * ([qw=1, qx=0, qy=0, qz=0]), and synchronizes the filter timestamp to current time.
     */
    void resetOrientation();

    // Coordinate Frame Transformation & Orientation Adjustment

    /**
     * @brief Remaps sensor physical axes to match custom mounting orientations.
     * 
     * When the IMU is installed rotated (e.g. 90°, 180°, 270°) or upside-down relative
     * to your device or chassis, this function transforms accelerometer and gyroscope
     * vectors into your vehicle body frame before orientation filtering is computed.
     * 
     * Axis definition codes:
     * - `+1` = +X, `-1` = -X
     * - `+2` = +Y, `-2` = -Y
     * - `+3` = +Z, `-3` = -Z
     * 
     * Common mounting configurations:
     * - Default flat (component facing up, nose forward):
     *   `setAxisRemap(1, 2, 3)`
     * - Rotated 90° clockwise (yaw):
     *   `setAxisRemap(2, -1, 3)`
     * - Rotated 180° (yaw):
     *   `setAxisRemap(-1, -2, 3)`
     * - Rotated 270° clockwise / 90° CCW (yaw):
     *   `setAxisRemap(-2, 1, 3)`
     * - Upside-down (Z pointing down):
     *   `setAxisRemap(1, -2, -3)`
     * 
     * @note To maintain a standard right-handed coordinate frame, ensure X × Y = Z.
     * 
     * @param x_axis Source axis mapped to body X (+1, -1, +2, -2, +3, -3).
     * @param y_axis Source axis mapped to body Y (+1, -1, +2, -2, +3, -3).
     * @param z_axis Source axis mapped to body Z (+1, -1, +2, -2, +3, -3).
     */
    void setAxisRemap(int8_t x_axis = 1, int8_t y_axis = 2, int8_t z_axis = 3);

    /**
     * @brief Toggles sign inversion for pitch angle output.
     * @param invert `true` to invert pitch sign (pitch = -pitch); `false` for default.
     */
    void invertPitch(bool invert = true);

    /**
     * @brief Toggles sign inversion for roll angle output.
     * @param invert `true` to invert roll sign (roll = -roll); `false` for default.
     */
    void invertRoll(bool invert = true);

    /**
     * @brief Toggles sign inversion for yaw angle output.
     * @param invert `true` to invert yaw sign (yaw = -yaw); `false` for default.
     */
    void invertYaw(bool invert = true);

    /**
     * @brief Checks if pitch angle inversion is active.
     */
    bool isPitchInverted() const { return _invert_pitch; }

    /**
     * @brief Checks if roll angle inversion is active.
     */
    bool isRollInverted() const { return _invert_roll; }

    /**
     * @brief Checks if yaw angle inversion is active.
     */
    bool isYawInverted() const { return _invert_yaw; }

    /**
     * @brief Zeroes/tares pitch, roll, and yaw angles at the current physical orientation.
     * 
     * Captures current attitude and applies it as an offset reference so that subsequent
     * angle outputs read (0, 0, 0) at the current position. Useful for leveling out
     * mechanical mounting tilts.
     */
    void tare();

    /**
     * @brief Manually sets tare offset angles.
     * @param pitch_offset Pitch angle offset to subtract (in degrees or radians).
     * @param roll_offset Roll angle offset to subtract (in degrees or radians).
     * @param yaw_offset Yaw angle offset to subtract (in degrees or radians).
     */
    void setTare(float pitch_offset, float roll_offset, float yaw_offset);

    /**
     * @brief Retrieves currently active tare angle offsets.
     * @param[out] pitch_offset Active pitch tare offset in degrees or radians.
     * @param[out] roll_offset  Active roll tare offset in degrees or radians.
     * @param[out] yaw_offset   Active yaw tare offset in degrees or radians.
     */
    void getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const;

    /**
     * @brief Retrieves raw orientation before tare offsets and sign inversion are applied.
     * @param[out] raw_pitch Un-tared internal pitch angle.
     * @param[out] raw_roll  Un-tared internal roll angle.
     * @param[out] raw_yaw   Un-tared internal yaw angle.
     */
    void getRawOrientation(float &raw_pitch, float &raw_roll, float &raw_yaw) const;

    /**
     * @brief Clears any active tare zero offsets back to 0.0.
     */
    void resetTare();

    // Stationary Gyro Calibration

    /**
     * @brief Runs an automatic stationary gyroscope zero-rate bias calibration.
     * 
     * Repeatedly samples the gyroscope while the sensor is completely motionless,
     * computes the mean zero-rate bias per axis, and stores them in internal bias registers.
     * These offsets are automatically subtracted during every subsequent `read()` call.
     * 
     * @warning The sensor MUST remain completely stationary on a stable, vibration-free
     *          surface while this function executes! Motion during calibration will corrupt offsets.
     * 
     * @param samples Number of stationary samples to average (default: 200).
     *                Delays ~4 ms between samples (~800 ms total execution time for 200 samples).
     * @param run_hardware_foc If `true`, runs Bosch on-chip Fast Offset Compensation before software bias refinement (default: `true`).
     */
    void calibrate(uint16_t samples = 200, bool run_hardware_foc = true);

    // Advanced Silicon Features (Robotics & Flight Control)

    /**
     * @brief Configures Digital Low-Pass Filter (DLPF) bandwidth / oversampling ratio.
     * 
     * Sets the hardware anti-aliasing filter mode for both accelerometer and gyroscope:
     * - `FilterBandwidth::OSR4`: 4× oversampling (maximum vibration attenuation; ideal for motors/props).
     * - `FilterBandwidth::OSR2`: 2× oversampling (balanced filter response and latency).
     * - `FilterBandwidth::NORMAL`: Normal mode (cutoff frequency = 0.5 × ODR; fastest tracking response).
     * 
     * @param bw Selected filter bandwidth mode (`FilterBandwidth::OSR4`, `OSR2`, or `NORMAL`). Default: `FilterBandwidth::OSR4`.
     * @return `true` if filter configuration was written to hardware successfully.
     * @return `false` if I2C communication failed.
     */
    bool setDLPF(FilterBandwidth bw = FilterBandwidth::OSR4);

    /**
     * @brief Runs on-chip Fast Offset Compensation (FOC) hardware calibration.
     * 
     * Triggers the internal Bosch BMI160 hardware calibration engine to trim accelerometer
     * offset to 0g on horizontal axes, ±1g on the vertical axis, and 0 deg/s on all gyro axes.
     * The calibrated values are written directly to on-chip hardware offset registers (0x71 - 0x76)
     * and enabled permanently via `REG_OFFSET_EN` (0x77).
     * 
     * @warning The device MUST remain completely static and level on a flat surface throughout the procedure!
     * 
     * @param z_up Physical orientation during calibration:
     *             - `true`: Sensor is laying flat with Z pointing upwards (+1g on Z, 0g on X/Y).
     *             - `false`: Sensor is upside-down with Z pointing downwards (-1g on Z, 0g on X/Y).
     *             Default is `true`.
     * @return `true` if hardware FOC completed within the 400 ms timeout window and offsets were enabled.
     * @return `false` if FOC timed out, communication failed, or mutex could not be acquired.
     */
    bool runHardwareFOC(bool z_up = true);

    /**
     * @brief Sets the operating power mode for the accelerometer and gyroscope cores.
     * 
     * Supported power modes:
     * - `PowerMode::NORMAL`: Full performance: both accelerometer and gyroscope active.
     * - `PowerMode::LOW_POWER`: Duty-cycled low-power mode for accelerometer; gyroscope suspended.
     * - `PowerMode::SUSPEND`: Deep low-power standby mode (< 5 μA); all sensor cores suspended.
     * 
     * @param mode Target power mode (`PowerMode::NORMAL`, `PowerMode::LOW_POWER`, or `PowerMode::SUSPEND`). Default: `PowerMode::NORMAL`.
     * @return `true` if power mode commands were acknowledged by the sensor.
     * @return `false` if I2C write failed or mutex acquisition timed out.
     */
    bool setPowerMode(PowerMode mode = PowerMode::NORMAL);

    // Hardware FIFO Engine (1024 bytes)

    /**
     * @brief Configures and enables the 1024-byte on-chip hardware FIFO buffer.
     * 
     * Configures the BMI160 hardware FIFO engine to autonomously queue sensor data frames
     * without host MCU intervention. Automatically flushes any preexisting FIFO contents upon activation.
     * 
     * @param accel Set to `true` to store accelerometer data in FIFO frames (default: `true`).
     * @param gyro Set to `true` to store gyroscope data in FIFO frames (default: `true`).
     * @return `true` if FIFO configuration was written successfully.
     * @return `false` if I2C write failed or mutex could not be acquired.
     */
    bool enableFIFO(bool accel = true, bool gyro = true);

    /**
     * @brief Reads the current byte count stored in the hardware FIFO buffer.
     * 
     * Reads registers `REG_FIFO_LENGTH_0` (0x22) and `REG_FIFO_LENGTH_1` (0x23).
     * 
     * @return uint16_t Number of data bytes available in the FIFO buffer (0 to 1024 bytes).
     *                  Returns 0 if read fails or mutex is unavailable.
     */
    uint16_t getFIFOCount();

    /**
     * @brief Reads and unpacks sensor frames from the hardware FIFO buffer into an array.
     * 
     * Unpacks 12-byte raw frames (6 bytes gyro + 6 bytes accel), converts them into physical
     * engineering units (g and deg/s), subtracts gyro biases, attaches temperature, and writes
     * them into the provided `buffer`.
     * 
     * @param[out] buffer Pointer to pre-allocated array of `Motion` structs to receive unpacked frames.
     * @param max_samples Maximum capacity of `buffer` in number of `Motion` frames.
     * @return uint16_t Number of complete sensor frames successfully read and unpacked.
     */
    uint16_t readFIFO(Motion *buffer, uint16_t max_samples);

    /**
     * @brief Flushes and empties the on-chip hardware FIFO buffer.
     * 
     * Issues command `CMD_FIFO_FLUSH` (0xB0) to `REG_CMD` (0x7E), clearing all queued frames
     * and resetting the FIFO read pointer.
     */
    void flushFIFO();

    // Hardware Pedometer

    /**
     * @brief Enables or disables the on-chip hardware step detector and pedometer engine.
     * 
     * Enables Bosch BMI160 internal pedometer DSP engine. Step counting runs entirely on the
     * IMU silicon with zero CPU overhead on the ESP32.
     * 
     * @param enable `true` to activate step counting; `false` to disable. Default is `true`.
     */
    void enablePedometer(bool enable = true);

    /**
     * @brief Reads the total accumulated step count from the hardware pedometer.
     * 
     * Reads 16-bit step counter registers `REG_STEP_CNT_0` (0x78) and `REG_STEP_CNT_1` (0x79).
     * 
     * @return uint16_t Number of detected steps since sensor power-on or last `resetSteps()`.
     *                  Returns 0 on bus read failure.
     */
    uint16_t getSteps();

    /**
     * @brief Resets the on-chip hardware step counter back to zero.
     * 
     * Issues `CMD_STEP_CNT_CLR` (0xB2) command to `REG_CMD` (0x7E).
     */
    void resetSteps();

    // Hardware Interrupts

    /**
     * @brief Configures and enables the hardware Any-Motion (slope) interrupt.
     * 
     * Detects physical movement across any axis when linear acceleration slope exceeds
     * `threshold_lsb`. Can be mapped to physical INT1 or INT2 pins to wake an external host
     * or trigger an ESP32 GPIO interrupt.
     * 
     * @param threshold_lsb Motion slope threshold in LSB. Scale depends on selected g-range
     *                      (e.g., at ±2g, 1 LSB ≈ 3.91 mg). Default is 20 (~78.2 mg).
     * @param int_pin Physical hardware interrupt output pin on the BMI160 module:
     *                - 1: Output on INT1 pin.
     *                - 2: Output on INT2 pin.
     *                Default is 1.
     * @return `true` if interrupt registers and pin mappings were configured successfully.
     * @return `false` if I2C write failed or mutex acquisition timed out.
     */
    bool enableMotionInterrupt(uint8_t threshold_lsb = 20, int int_pin = 1);

    /**
     * @brief Configures and enables the hardware Tap / Double-Tap gesture interrupt.
     * 
     * Triggers when a mechanical shock signature matching a single or double tap is detected.
     * 
     * @param double_tap Set to `true` to trigger on double-tap; `false` for single-tap. Default is `true`.
     * @param threshold_lsb Shock threshold in LSB (bits [4:0] of `REG_INT_TAP_1`). Default is 10.
     * @param int_pin Physical hardware interrupt output pin on the BMI160 module (1 = INT1, 2 = INT2). Default: 1.
     * @return `true` if tap interrupt was configured and mapped to the requested pin.
     * @return `false` if I2C write failed or mutex acquisition timed out.
     */
    bool enableTapInterrupt(bool double_tap = true, uint8_t threshold_lsb = 10, int int_pin = 1);

    // Diagnostics

    /**
     * @brief Scans all 127 I2C bus addresses and prints responding devices to Serial.
     * 
     * Useful diagnostic utility for validating hardware connections, solder joints,
     * pull-up resistors, and detecting alternate I2C addresses.
     * 
     * @return uint8_t Total count of responsive I2C devices discovered on the bus.
     */
    uint8_t scanBus();

    /**
     * @brief Returns the most recent error code recorded by the driver.
     * 
     * @return ErrorCode Enum representing the last fault (e.g. `ErrorCode::NONE`,
     *                   `ERR_NO_PULLUP`, `ERR_CHIP_NOT_FOUND`, `ERR_I2C_NACK`,
     *                   `ERR_MUTEX_TIMEOUT`, etc.).
     */
    ErrorCode getLastError() const { return _last_error; }

    // Low-Level Manipulation & Multi-Core FreeRTOS

    /**
     * @brief Reads uncalibrated, unscaled raw 16-bit register values in a single burst.
     * 
     * Performs an atomic 12-byte burst read of registers 0x0C through 0x17 without applying
     * scale factor divisions, bias subtraction, or orientation filter processing. Ideal for
     * minimal-latency raw logging or custom DSP algorithms.
     * 
     * @param[out] raw Output reference to `RawMotion` struct to receive raw 16-bit integer values.
     * @return `true` if burst read was successful.
     * @return `false` if I2C transaction failed or mutex lock timed out.
     */
    bool readRaw(RawMotion &raw);

    /**
     * @brief Writes an 8-bit value to a specific BMI160 hardware register.
     * 
     * @param reg 8-bit register address (see [BMI160_Registers.h](file:///home/scintill-a/Documents/PlatformIO/Projects/imu-bmi160/lib/BMI160_ESPx/src/BMI160_Registers.h)).
     * @param value 8-bit byte to write.
     * @return `true` if write transaction was acknowledged by sensor (ACK).
     * @return `false` if bus transmission failed or was NACKed.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Reads a single 8-bit value from a specific BMI160 hardware register.
     * 
     * @param reg 8-bit register address to read.
     * @param[out] value Reference to uint8_t where the read byte will be stored.
     * @return `true` if read was successful and byte was received.
     * @return `false` if bus transaction failed or register could not be read.
     */
    bool readRegister(uint8_t reg, uint8_t &value);

    /**
     * @brief Performs a multi-byte burst read starting from a specific register address.
     * 
     * Leverages BMI160 internal auto-incrementing address pointer to read multiple consecutive
     * registers in a single atomic I2C transaction.
     * 
     * @param start_reg Starting register address.
     * @param[out] buf Pointer to destination buffer of at least `length` bytes.
     * @param length Number of contiguous bytes to read.
     * @return `true` if all requested bytes were read successfully.
     * @return `false` if pointer is null, length is 0, or bus transaction failed.
     */
    bool readBurst(uint8_t start_reg, uint8_t *buf, size_t length);

    /**
     * @brief Manually sets stationary gyroscope zero-rate bias offsets.
     * 
     * Useful for restoring pre-calibrated factory offsets stored in Non-Volatile Storage
     * (NVS / EEPROM) on boot without having to re-run stationary calibration every time.
     * 
     * @param bx X-axis gyro zero-rate bias offset in deg/s.
     * @param by Y-axis gyro zero-rate bias offset in deg/s.
     * @param bz Z-axis gyro zero-rate bias offset in deg/s.
     */
    void setGyroOffsets(float bx, float by, float bz);

    /**
     * @brief Retrieves the active stationary gyroscope zero-rate bias offsets.
     * 
     * Reads the current bias offsets (calculated via `calibrate()` or set via `setGyroOffsets()`),
     * allowing the application to persist them to NVS/EEPROM.
     * 
     * @param[out] bx Output reference for X-axis bias in deg/s.
     * @param[out] by Output reference for Y-axis bias in deg/s.
     * @param[out] bz Output reference for Z-axis bias in deg/s.
     */
    void getGyroOffsets(float &bx, float &by, float &bz) const;

    /**
     * @brief Gets the active 7-bit I2C slave address currently used by the driver.
     * 
     * @return uint8_t Current 7-bit I2C address (typically 0x68 or 0x69).
     */
    uint8_t getAddress() const { return _addr; }

    /**
     * @brief Verifies whether the BMI160 sensor is alive and responding on the I2C bus.
     * 
     * Reads `REG_CHIP_ID` (0x00) and compares the returned value with `CHIP_ID_VALUE` (0xD1).
     * 
     * @return `true` if communication succeeds and chip ID is 0xD1; `false` otherwise.
     */
    bool isConnected();

    /**
     * @brief Performs a software reset of the BMI160 sensor.
     * 
     * Issues command `CMD_SOFT_RESET` (0xB6) to `REG_CMD` (0x7E), restoring all internal
     * registers to their default power-on reset state, and blocks for 100 ms while the
     * internal sensor bootloader reloads.
     * 
     * @return `true` if soft-reset command was successfully sent to the sensor.
     * @return `false` if mutex acquisition timed out or I2C write failed.
     */
    bool softReset();

    /**
     * @brief Spawns a high-priority background FreeRTOS sampling task pinned to a specific ESP32 CPU core.
     * 
     * The background task (`BMI160_Async`) runs periodically at `rate_hz` using `vTaskDelayUntil()`,
     * executes atomic `read()` calls, and pushes complete `Motion` telemetry packets directly
     * into `out_queue`. This guarantees deterministic sampling jitter and offloads IMU
     * communication from the main loop or UI tasks.
     * 
     * @param rate_hz Desired sampling rate in Hertz (e.g., 50, 100, 200). Must be > 0.
     * @param out_queue FreeRTOS QueueHandle_t configured with item size `sizeof(Motion)`
     *                  (created via `xQueueCreate(queue_depth, sizeof(Motion))`).
     * @param core ESP32 CPU core to pin the worker task to (0 or 1, default: 1).
     * @return `true` if FreeRTOS task was successfully created and started.
     * @return `false` if task is already running, queue handle is null, rate is 0,
     *                 or memory allocation failed.
     */
    bool startAsyncStream(uint16_t rate_hz, QueueHandle_t out_queue, BaseType_t core = 1);

    /**
     * @brief Stops and terminates the background FreeRTOS streaming worker task.
     * 
     * Signals the streaming loop to exit, unbinds the target queue, and waits for task deletion.
     */
    void stopAsyncStream();

    /**
     * @brief Checks whether the background FreeRTOS streaming task is currently running.
     * 
     * @return `true` if the background streamer task is active; `false` otherwise.
     */
    bool isAsyncStreamActive() const { return _task_handle != nullptr; }

private:
    TwoWire &_wire;
    uint8_t _addr = 0x68;
    int _sda = 21;
    int _scl = 22;
    uint32_t _freq = 400000;

    bool _use_degrees = true;
    FusionMode _fusion_mode = FusionMode::MADGWICK;
    FilterBandwidth _active_bw = FilterBandwidth::OSR4;
    ErrorCode _last_error = ErrorCode::NONE;

    // Stability & Dynamic Rejection features
    bool _dynamic_accel_rejection = true;
    bool _zero_velocity_update = true;
    float _zupt_threshold_dps = 0.35f;

    // Scale factors
    float _accel_scale = 16384.0f; // Default ±2g
    float _gyro_scale  = 131.2f;   // Default ±250 dps

    // Gyro stationary biases
    float _bias_gx = 0.0f;
    float _bias_gy = 0.0f;
    float _bias_gz = 0.0f;

    // Orientation state
    float _filter_alpha = 0.98f;
    float _madgwick_beta = 0.1f;
    float _pitch = 0.0f;
    float _roll  = 0.0f;
    float _yaw   = 0.0f;

    // Coordinate transformation & tare offsets
    int8_t _remap_x = 1;
    int8_t _remap_y = 2;
    int8_t _remap_z = 3;
    bool _invert_pitch = false;
    bool _invert_roll  = false;
    bool _invert_yaw   = false;
    float _tare_pitch  = 0.0f;
    float _tare_roll   = 0.0f;
    float _tare_yaw    = 0.0f;

    static float _remapAxis(int8_t map, float x, float y, float z);

    // Quaternion state (Madgwick AHRS)
    float _q0 = 1.0f;
    float _q1 = 0.0f;
    float _q2 = 0.0f;
    float _q3 = 0.0f;

    uint64_t _last_filter_us = 0;

    // Temperature cache & rate limiter
    float _cached_temp = 23.0f;
    uint64_t _last_temp_read_us = 0;

    Motion _last_motion;

    // Thread safety & FreeRTOS
    SemaphoreHandle_t _mutex = nullptr;
    TaskHandle_t _task_handle = nullptr;
    QueueHandle_t _target_queue = nullptr;
    uint16_t _stream_hz = 100;
    volatile bool _stream_running = false;

    /**
     * @brief Performs bus health check and active 9-clock recovery to clear hung I2C devices.
     * 
     * Tests SDA and SCL with internal pull-ups. If SDA is clamped LOW by a slave stuck
     * mid-byte, pulses SCL 9 times followed by a manual STOP condition to release the bus.
     * 
     * @param sda GPIO pin for SDA.
     * @param scl GPIO pin for SCL.
     * @return `true` if bus lines are healthy (HIGH); `false` if pull-ups are missing or line is shorted.
     */
    bool _preCheckBus(int sda, int scl);

    /**
     * @brief Takes internal FreeRTOS mutex to guarantee thread-safe I2C transactions.
     * @param ticks Timeout in FreeRTOS ticks (default: 50 ms).
     * @return `true` if mutex was acquired; `false` on timeout.
     */
    bool _acquireMutex(TickType_t ticks = pdMS_TO_TICKS(50));

    /**
     * @brief Releases the internal FreeRTOS transaction mutex.
     */
    void _releaseMutex();

    /**
     * @brief Computes delta-time and dispatches motion readings to active fusion algorithm.
     * @param ax Calibrated linear acceleration X in g.
     * @param ay Calibrated linear acceleration Y in g.
     * @param az Calibrated linear acceleration Z in g.
     * @param gx Calibrated angular rate X in deg/s.
     * @param gy Calibrated angular rate Y in deg/s.
     * @param gz Calibrated angular rate Z in deg/s.
     * @param now_us Current system timestamp in microseconds.
     */
    void _updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, uint64_t now_us);

    /**
     * @brief Executes one iteration of the Madgwick AHRS 6-DOF quaternion filter.
     * @param ax Calibrated linear acceleration X in g.
     * @param ay Calibrated linear acceleration Y in g.
     * @param az Calibrated linear acceleration Z in g.
     * @param gx_dps Calibrated angular rate X in deg/s.
     * @param gy_dps Calibrated angular rate Y in deg/s.
     * @param gz_dps Calibrated angular rate Z in deg/s.
     * @param dt Sampling time interval in seconds.
     */
    void _updateMadgwick(float ax, float ay, float az, float gx_dps, float gy_dps, float gz_dps, float dt);

    /**
     * @brief Executes one iteration of the lightweight Complementary tilt filter.
     * @param ax Calibrated linear acceleration X in g.
     * @param ay Calibrated linear acceleration Y in g.
     * @param az Calibrated linear acceleration Z in g.
     * @param gx Calibrated angular rate X in deg/s.
     * @param gy Calibrated angular rate Y in deg/s.
     * @param gz Calibrated angular rate Z in deg/s.
     * @param dt Sampling time interval in seconds.
     */
    void _updateComplementary(float ax, float ay, float az, float gx, float gy, float gz, float dt);

    /**
     * @brief Static FreeRTOS task entry trampoline function.
     * @param param Pointer to the calling `BMI160_ESPx` instance (`this`).
     */
    static void _taskRunner(void *param);

    /**
     * @brief FreeRTOS background task loop that samples IMU and enqueues telemetry packets.
     */
    void _taskLoop();
};
