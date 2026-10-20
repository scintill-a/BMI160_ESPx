#pragma once

#include <stdint.h>

/**
 * @file BMI160_Types.h
 * @brief Industrial-grade data structures and types for BMI160_ESPx.
 */

namespace BMI160_ESPx_Types {

    /**
     * @struct Motion
     * @brief Primary motion, attitude, and telemetry packet.
     * 
     * Contains calibrated 3-axis linear acceleration, 3-axis angular velocity,
     * calculated 6-DOF Euler orientation angles, 4-element unit orientation quaternion,
     * internal die temperature, and microsecond precision timestamp.
     */
    struct Motion {
        // Accelerometer linear acceleration (in units of g, where 1g ≈ 9.80665 m/s²)
        float ax = 0.0f; ///< Linear acceleration along X-axis (forward/backward) in g
        float ay = 0.0f; ///< Linear acceleration along Y-axis (lateral left/right) in g
        float az = 0.0f; ///< Linear acceleration along Z-axis (vertical up/down) in g

        // Gyroscope angular velocity (in deg/s or rad/s depending on setDegrees())
        float gx = 0.0f; ///< Calibrated angular velocity around X-axis (roll rate)
        float gy = 0.0f; ///< Calibrated angular velocity around Y-axis (pitch rate)
        float gz = 0.0f; ///< Calibrated angular velocity around Z-axis (yaw rate)

        // 6-DOF Orientation Euler Angles (in degrees or radians depending on setDegrees())
        float pitch = 0.0f; ///< Pitch angle: rotation around lateral/Y axis (nose up/down)
        float roll  = 0.0f; ///< Roll angle: rotation around longitudinal/X axis (banking left/right)
        float yaw   = 0.0f; ///< Yaw angle: heading / rotation around vertical/Z axis (integrated from gyro)

        // Unit Orientation Quaternion [qw, qx, qy, qz] (Hamilton convention)
        float qw = 1.0f; ///< Quaternion scalar component (W)
        float qx = 0.0f; ///< Quaternion vector X component
        float qy = 0.0f; ///< Quaternion vector Y component
        float qz = 0.0f; ///< Quaternion vector Z component

        // Die temperature
        float temperature = 0.0f; ///< Internal sensor die temperature in degrees Celsius (°C)

        // High-precision ESP32 microsecond timestamp
        uint64_t timestamp_us = 0; ///< esp_timer_get_time() timestamp in microseconds (μs) when sample was read
    };

    /**
     * @struct RawMotion
     * @brief Uncalibrated 16-bit raw integer readings from BMI160 hardware registers.
     * 
     * Contains raw signed 16-bit integers directly from registers 0x0C through 0x17.
     * Useful for direct integer DSP pipelines, external custom calibration routines, or minimal-overhead logging.
     */
    struct RawMotion {
        int16_t ax = 0; ///< Raw 16-bit signed accelerometer X register value
        int16_t ay = 0; ///< Raw 16-bit signed accelerometer Y register value
        int16_t az = 0; ///< Raw 16-bit signed accelerometer Z register value
        int16_t gx = 0; ///< Raw 16-bit signed gyroscope X register value
        int16_t gy = 0; ///< Raw 16-bit signed gyroscope Y register value
        int16_t gz = 0; ///< Raw 16-bit signed gyroscope Z register value
    };

    /**
     * @enum FusionMode
     * @brief Sensor fusion algorithm selection for real-time 6-DOF attitude estimation.
     */
    enum class FusionMode : uint8_t {
        /**
         * @brief Lightweight 2-axis tilt filter combining gravity vector and integrated gyro rates.
         * 
         * Extremely fast, deterministic execution with negligible CPU overhead.
         */
        COMPLEMENTARY = 0,

        /**
         * @brief SOTA 6-DOF gradient-descent quaternion AHRS filter.
         * 
         * Provides true 3D spatial attitude representation with zero gimbal lock.
         */
        MADGWICK      = 1
    };

    /**
     * @enum FilterBandwidth
     * @brief Digital Low-Pass Filter (DLPF) / anti-aliasing oversampling modes.
     */
    enum class FilterBandwidth : uint8_t {
        OSR4   = 0x00, ///< 4× oversampling (maximum vibration damping; highest noise attenuation)
        OSR2   = 0x10, ///< 2× oversampling (balanced filter response and latency)
        NORMAL = 0x20  ///< Normal mode (filter cutoff frequency = 0.5 × ODR; fastest tracking response)
    };

    /**
     * @enum PowerMode
     * @brief Operating power modes for accelerometer and gyroscope cores.
     */
    enum class PowerMode : uint8_t {
        NORMAL,    ///< Full performance: both accelerometer and gyroscope actively sampling
        LOW_POWER, ///< Low power mode: accelerometer duty-cycled, gyroscope suspended
        SUSPEND    ///< Deep sleep standby (< 5 μA): all sensor cores suspended
    };

    /**
     * @enum ErrorCode
     * @brief Structured diagnostic error codes recorded during bus and sensor operations.
     */
    enum class ErrorCode : uint8_t {
        NONE = 0,           ///< No error detected; operation succeeded
        ERR_NO_PULLUP,      ///< Bus pre-check detected missing pull-up resistors or line held LOW
        ERR_CHIP_NOT_FOUND, ///< Sensor chip ID did not return expected value (0xD1) at probed addresses
        ERR_RESET_TIMEOUT,  ///< Software reset did not complete within expected time window
        ERR_I2C_NACK,       ///< I2C transmission was not acknowledged (NACK) by slave device
        ERR_MUTEX_TIMEOUT,  ///< FreeRTOS transaction synchronization mutex acquisition timed out
        ERR_FIFO_OVERFLOW   ///< Hardware FIFO buffer overflow condition detected
    };

    /**
     * @enum RangeG
     * @brief Strongly-typed accelerometer full-scale measurement ranges.
     */
    enum class RangeG : uint8_t {
        G_2  = 2,  ///< ±2g full-scale range (sensitivity: 16384 LSB/g)
        G_4  = 4,  ///< ±4g full-scale range (sensitivity: 8192 LSB/g)
        G_8  = 8,  ///< ±8g full-scale range (sensitivity: 4096 LSB/g)
        G_16 = 16  ///< ±16g full-scale range (sensitivity: 2048 LSB/g)
    };

    /**
     * @enum RangeDPS
     * @brief Strongly-typed gyroscope full-scale measurement ranges.
     */
    enum class RangeDPS : uint16_t {
        DPS_125  = 125,  ///< ±125 deg/s full-scale range (sensitivity: 262.4 LSB/(deg/s))
        DPS_250  = 250,  ///< ±250 deg/s full-scale range (sensitivity: 131.2 LSB/(deg/s))
        DPS_500  = 500,  ///< ±500 deg/s full-scale range (sensitivity: 65.6 LSB/(deg/s))
        DPS_1000 = 1000, ///< ±1000 deg/s full-scale range (sensitivity: 32.8 LSB/(deg/s))
        DPS_2000 = 2000  ///< ±2000 deg/s full-scale range (sensitivity: 16.4 LSB/(deg/s))
    };

} // namespace BMI160_ESPx_Types
