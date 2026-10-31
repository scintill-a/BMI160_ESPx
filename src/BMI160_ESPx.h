#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "BMI160_Registers.h"
#include "BMI160_Types.h"

using Motion          = BMI160_ESPx_Types::Motion;
using RawMotion       = BMI160_ESPx_Types::RawMotion;
using FusionMode      = BMI160_ESPx_Types::FusionMode;
using FilterBandwidth = BMI160_ESPx_Types::FilterBandwidth;
using ErrorCode       = BMI160_ESPx_Types::ErrorCode;
using RangeG          = BMI160_ESPx_Types::RangeG;
using RangeDPS        = BMI160_ESPx_Types::RangeDPS;

/**
 * @class BMI160_ESPx
 * @brief Streamlined, high-performance driver for the Bosch BMI160 6-DOF IMU on Espressif ESP32.
 * 
 * Core Features:
 * - Direct natural number configuration (setAccelRange(2), setGyroRange(250), setDataRate(100)).
 * - Real-time 6-DOF attitude estimation (Pitch, Roll, Yaw, Quaternions) via Madgwick AHRS or Complementary filter.
 * - Active 9-clock I2C bus recovery and line pre-check to prevent bus lockups.
 * - Stationary gyro calibration, tare zeroing, and coordinate remapping / axis inversion.
 * - FreeRTOS thread-safe I2C transactions.
 */
class BMI160_ESPx {
public:
    /**
     * @brief Constructs a new BMI160_ESPx driver instance.
     * @param wire Reference to TwoWire peripheral (defaults to Wire).
     */
    explicit BMI160_ESPx(TwoWire &wire = Wire);

    /**
     * @brief Destructs the driver instance and safely deallocates the FreeRTOS mutex.
     */
    virtual ~BMI160_ESPx();

    // Quick-Start API

    /**
     * @brief Connects to the sensor, clears bus contention, and applies defaults (±2g, ±250 deg/s, 100 Hz).
     * @param sda GPIO pin for SDA (default: 21).
     * @param scl GPIO pin for SCL (default: 22).
     * @param frequency Operational I2C clock speed in Hz (default: 400000 Hz).
     * @param address Specific 7-bit address (0 for auto probing 0x68 and 0x69).
     * @return true if initialized successfully; false otherwise.
     */
    bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0);

    /**
     * @brief Atomically reads high-speed motion telemetry and updates 6-DOF orientation.
     * @param[out] m Reference to Motion struct to populate.
     * @return true if read succeeded; false on bus error or timeout.
     */
    bool read(Motion &m);

    /**
     * @brief Direct motion return for clean one-line access in polling loops.
     * @return Motion struct with latest telemetry and attitude.
     */
    Motion read();

    // Direct Getters (Inlined)
    float getPitch() const              { return _last_motion.pitch; }
    float getRoll() const               { return _last_motion.roll; }
    float getYaw() const                { return _last_motion.yaw; }
    float getAccelX() const             { return _last_motion.ax; }
    float getAccelY() const             { return _last_motion.ay; }
    float getAccelZ() const             { return _last_motion.az; }
    float getGyroX() const              { return _last_motion.gx; }
    float getGyroY() const              { return _last_motion.gy; }
    float getGyroZ() const              { return _last_motion.gz; }
    float getTemperature();
    const Motion& getLastMotion() const { return _last_motion; }

    // Configuration Methods
    bool setAccelRange(uint8_t g = 2);
    bool setAccelRange(RangeG range);
    bool setGyroRange(uint16_t dps = 250);
    bool setGyroRange(RangeDPS range);
    bool setDataRate(uint16_t hz = 100);
    void setDegrees(bool enable_degrees = true);
    bool setDLPF(FilterBandwidth bw = FilterBandwidth::OSR4);

    // Sensor Fusion Configuration
    void setFusionAlgorithm(FusionMode mode = FusionMode::MADGWICK);
    void setFilterWeight(float alpha = 0.98f);
    void setMadgwickBeta(float beta = 0.1f);
    void setDynamicAccelRejection(bool enable) { _dynamic_accel_rejection = enable; }
    void setZUPT(bool enable, float threshold_dps = 0.35f) {
        _zero_velocity_update = enable;
        _zupt_threshold_dps = threshold_dps;
    }
    void resetOrientation();

    // Coordinate Frame Transformation & Tare
    void setAxisRemap(int8_t x_axis = 1, int8_t y_axis = 2, int8_t z_axis = 3);
    void invertPitch(bool invert = true);
    void invertRoll(bool invert = true);
    void invertYaw(bool invert = true);
    bool isPitchInverted() const { return _invert_pitch; }
    bool isRollInverted() const  { return _invert_roll; }
    bool isYawInverted() const   { return _invert_yaw; }

    void tare();
    void setTare(float pitch_offset, float roll_offset, float yaw_offset);
    void getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const;
    void getRawOrientation(float &raw_pitch, float &raw_roll, float &raw_yaw) const;
    void resetTare();

    // Stationary Gyro Calibration
    void calibrate(uint16_t samples = 200);
    void setGyroOffsets(float bx, float by, float bz);
    void getGyroOffsets(float &bx, float &by, float &bz) const;

    // Diagnostics & Low-Level Manipulation
    ErrorCode getLastError() const { return _last_error; }
    bool readRaw(RawMotion &raw);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readBurst(uint8_t start_reg, uint8_t *buf, size_t length);
    uint8_t getAddress() const { return _addr; }
    bool isConnected();
    bool softReset();

private:
    TwoWire &_wire;
    uint8_t _addr = 0x68;
    int _sda = 21;
    int _scl = 22;
    uint32_t _freq = 400000;

    bool _use_degrees = true;
    FusionMode _fusion_mode = FusionMode::MADGWICK;
    FilterBandwidth _active_bw = FilterBandwidth::OSR4;
    uint16_t _active_odr_hz = 100;
    ErrorCode _last_error = ErrorCode::NONE;

    // Dynamic Rejection & Stationary ZUPT
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

    // Thread safety
    SemaphoreHandle_t _mutex = nullptr;

    bool _preCheckBus(int sda, int scl);
    bool _acquireMutex(TickType_t ticks = pdMS_TO_TICKS(50));
    void _releaseMutex();

    void _updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, uint64_t now_us);
    void _updateMadgwick(float ax, float ay, float az, float gx_dps, float gy_dps, float gz_dps, float dt);
    void _updateComplementary(float ax, float ay, float az, float gx, float gy, float gz, float dt);
};
