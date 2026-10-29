#include "BMI160_ESPx.h"
#include <esp_timer.h>
#include <cmath>

using namespace BMI160_ESPx_Defs;

/**
 * @brief Constructs the driver instance and allocates the internal FreeRTOS mutex.
 * @param wire Reference to the TwoWire I2C peripheral (defaults to Wire).
 */
BMI160_ESPx::BMI160_ESPx(TwoWire &wire) : _wire(wire) {
    _mutex = xSemaphoreCreateRecursiveMutex();
}

/**
 * @brief Destructs the driver instance, stops active worker tasks, and deletes mutex.
 */
BMI160_ESPx::~BMI160_ESPx() {
    stopAsyncStream();
    if (_mutex != nullptr) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

/**
 * @brief Acquires the internal FreeRTOS mutex for thread-safe I2C access.
 * @param ticks Maximum tick timeout to wait for mutex acquisition (default: 50 ms).
 * @return `true` if mutex was successfully taken; `false` on timeout.
 */
bool BMI160_ESPx::_acquireMutex(TickType_t ticks) {
    if (_mutex == nullptr) return true;
    return (xSemaphoreTakeRecursive(_mutex, ticks) == pdTRUE);
}

/**
 * @brief Releases the internal FreeRTOS mutex after an I2C transaction finishes.
 */
void BMI160_ESPx::_releaseMutex() {
    if (_mutex != nullptr) {
        xSemaphoreGiveRecursive(_mutex);
    }
}

/**
 * @brief Tests I2C bus lines with internal pull-ups and clears stuck bus conditions.
 * 
 * If a slave device holds SDA LOW mid-transaction, clocks SCL 9 times followed by a
 * STOP condition to release the bus before Wire peripheral attaches.
 * 
 * @param sda GPIO pin assigned to SDA.
 * @param scl GPIO pin assigned to SCL.
 * @return `true` if bus lines are healthy (logic HIGH); `false` if missing pull-ups or line shorted.
 */
bool BMI160_ESPx::_preCheckBus(int sda, int scl) {
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delay(10);

    // If lines are not high with pull-ups, report warning
    if (digitalRead(sda) == LOW || digitalRead(scl) == LOW) {
        // If SDA is held low by a hung sensor, clock SCL 9 times to free it
        if (digitalRead(sda) == LOW) {
            pinMode(scl, OUTPUT);
            for (int i = 0; i < 9; ++i) {
                digitalWrite(scl, LOW);
                delayMicroseconds(5);
                digitalWrite(scl, HIGH);
                delayMicroseconds(5);
            }

            // Generate STOP condition
            pinMode(sda, OUTPUT);
            digitalWrite(sda, LOW);
            delayMicroseconds(5);
            digitalWrite(scl, HIGH);
            delayMicroseconds(5);
            digitalWrite(sda, HIGH);
            delayMicroseconds(5);

            pinMode(sda, INPUT_PULLUP);
            pinMode(scl, INPUT_PULLUP);
            delay(10);
        }

        if (digitalRead(sda) == LOW || digitalRead(scl) == LOW) {
            _last_error = ErrorCode::ERR_NO_PULLUP;
            return false;
        }
    }
    return true;
}

/**
 * @brief Initializes the BMI160 IMU, probes chip ID, and applies standard defaults.
 * 
 * @param sda GPIO pin for SDA (default 21).
 * @param scl GPIO pin for SCL (default 22).
 * @param frequency Target I2C bus frequency in Hz (default 400,000 Hz).
 * @param address 7-bit slave address (0x68 or 0x69; 0 for auto-detection).
 * @return `true` if initialization succeeded; `false` on communication error or missing device.
 */
bool BMI160_ESPx::begin(int sda, int scl, uint32_t frequency, uint8_t address) {
    _sda  = sda;
    _scl  = scl;
    _freq = frequency;
    _last_error = ErrorCode::NONE;

    // 1. Clear physical line contention before Wire attaches
    _preCheckBus(_sda, _scl);

    // 2. Initialize Wire peripheral at conservative 100 kHz for initial handshake
    _wire.begin(_sda, _scl);
    _wire.setClock(100000);
    delay(30);

    if (!_acquireMutex()) {
        _last_error = ErrorCode::ERR_MUTEX_TIMEOUT;
        return false;
    }

    // 3. Address resolution & Chip ID probing
    uint8_t chip_id = 0;
    bool found = false;

    if (address != 0) {
        _addr = address;
        if (readRegister(REG_CHIP_ID, chip_id) && chip_id == CHIP_ID_VALUE) {
            found = true;
        }
    } else {
        // Probe 0x68 (SA0=GND) then 0x69 (SA0=VCC)
        _addr = I2C_ADDR_LOWER;
        if (readRegister(REG_CHIP_ID, chip_id) && chip_id == CHIP_ID_VALUE) {
            found = true;
        } else {
            _addr = I2C_ADDR_UPPER;
            if (readRegister(REG_CHIP_ID, chip_id) && chip_id == CHIP_ID_VALUE) {
                found = true;
            }
        }
    }

    if (!found) {
        _last_error = ErrorCode::ERR_CHIP_NOT_FOUND;
        _releaseMutex();
        Serial.printf("[BMI160] Chip not detected at 0x68 or 0x69 (Read ID: 0x%02X). Running bus scan:\n", chip_id);
        scanBus();
        return false;
    }

    // 4. Soft-reset device and wait for internal bootloader
    writeRegister(REG_CMD, CMD_SOFT_RESET);
    delay(100);

    // 5. Power on Accelerometer and Gyroscope into Normal Mode
    writeRegister(REG_CMD, CMD_ACC_PM_NORMAL);
    delay(50);
    writeRegister(REG_CMD, CMD_GYR_PM_NORMAL);
    delay(85);

    _releaseMutex();

    // 6. Set standard defaults (±2g, ±250 dps, 100 Hz)
    setAccelRange(2);
    setGyroRange(250);
    setDataRate(100);

    // 7. Accelerate I2C clock to user preference (e.g. 400 kHz)
    _wire.setClock(_freq);

    // Reset orientation state
    resetOrientation();

    return true;
}

/**
 * @brief Sends software reset command (0xB6) and waits 100 ms for reboot.
 * @return `true` if command was successfully transmitted; `false` on failure.
 */
bool BMI160_ESPx::softReset() {
    if (!_acquireMutex()) return false;
    bool ok = writeRegister(REG_CMD, CMD_SOFT_RESET);
    delay(100);
    _releaseMutex();
    return ok;
}

/**
 * @brief Checks if BMI160 is responding on I2C bus with valid chip ID (0xD1).
 * @return `true` if connected and responsive; `false` otherwise.
 */
bool BMI160_ESPx::isConnected() {
    uint8_t id = 0;
    if (!readRegister(REG_CHIP_ID, id)) return false;
    return (id == CHIP_ID_VALUE);
}

/**
 * @brief Sets the full-scale measurement range of the accelerometer.
 * 
 * Configures `REG_ACC_RANGE` (0x41) and updates internal scale factor `_accel_scale`:
 * - 2g:   16384 LSB/g (register value: 0x03) [Default]
 * - 4g:   8192 LSB/g  (register value: 0x05)
 * - 8g:   4096 LSB/g  (register value: 0x08)
 * - 16g:  2048 LSB/g  (register value: 0x0C)
 * 
 * @param g Acceleration range in g (acceptable values: 2, 4, 8, 16; default: 2).
 * @return `true` if valid range was specified and written successfully.
 * @return `false` if range value is invalid or I2C write failed.
 */
bool BMI160_ESPx::setAccelRange(uint8_t g) {
    if (!_acquireMutex()) return false;
    uint8_t reg_val;
    switch (g) {
        case 2:  reg_val = 0x03; _accel_scale = 16384.0f; break;
        case 4:  reg_val = 0x05; _accel_scale = 8192.0f;  break;
        case 8:  reg_val = 0x08; _accel_scale = 4096.0f;  break;
        case 16: reg_val = 0x0C; _accel_scale = 2048.0f;  break;
        default: 
            _releaseMutex();
            return false;
    }
    bool ok = writeRegister(REG_ACC_RANGE, reg_val);
    _releaseMutex();
    return ok;
}

bool BMI160_ESPx::setAccelRange(RangeG range) {
    return setAccelRange(static_cast<uint8_t>(range));
}

/**
 * @brief Sets the full-scale measurement range of the gyroscope.
 * 
 * Configures `REG_GYR_RANGE` (0x43) and updates internal scale factor `_gyro_scale`:
 * - 125 deg/s:   262.4 LSB/(deg/s) (register value: 0x04)
 * - 250 deg/s:   131.2 LSB/(deg/s) (register value: 0x03) [Default]
 * - 500 deg/s:   65.6 LSB/(deg/s)  (register value: 0x02)
 * - 1000 deg/s:  32.8 LSB/(deg/s)  (register value: 0x01)
 * - 2000 deg/s:  16.4 LSB/(deg/s)  (register value: 0x00)
 * 
 * @param dps Angular rate range in deg/s (acceptable values: 125, 250, 500, 1000, 2000; default: 250).
 * @return `true` if valid range was specified and written successfully.
 * @return `false` if range value is invalid or I2C write failed.
 */
bool BMI160_ESPx::setGyroRange(uint16_t dps) {
    if (!_acquireMutex()) return false;
    uint8_t reg_val;
    switch (dps) {
        case 125:  reg_val = 0x04; _gyro_scale = 262.4f; break;
        case 250:  reg_val = 0x03; _gyro_scale = 131.2f; break;
        case 500:  reg_val = 0x02; _gyro_scale = 65.6f;  break;
        case 1000: reg_val = 0x01; _gyro_scale = 32.8f;  break;
        case 2000: reg_val = 0x00; _gyro_scale = 16.4f;  break;
        default: 
            _releaseMutex();
            return false;
    }
    bool ok = writeRegister(REG_GYR_RANGE, reg_val);
    _releaseMutex();
    return ok;
}

bool BMI160_ESPx::setGyroRange(RangeDPS range) {
    return setGyroRange(static_cast<uint16_t>(range));
}

/**
 * @brief Sets Output Data Rate (ODR) for both accelerometer and gyroscope.
 * 
 * Configures `REG_ACC_CONF` (0x40) and `REG_GYR_CONF` (0x42) with active DLPF bandwidth mode.
 * 
 * @param hz Target sampling rate in Hz (e.g. 25, 50, 100, 200, 400, 800, 1600; default: 100).
 * @return `true` if configuration was written successfully to both sensor cores; `false` on bus error.
 */
bool BMI160_ESPx::setDataRate(uint16_t hz) {
    if (!_acquireMutex()) return false;
    _active_odr_hz = hz;
    uint8_t odr_val = 0x08; // default 100 Hz

    if (hz <= 25)        odr_val = 0x06; // 25 Hz
    else if (hz <= 50)   odr_val = 0x07; // 50 Hz
    else if (hz <= 100)  odr_val = 0x08; // 100 Hz
    else if (hz <= 200)  odr_val = 0x09; // 200 Hz
    else if (hz <= 400)  odr_val = 0x0A; // 400 Hz
    else if (hz <= 800)  odr_val = 0x0B; // 800 Hz
    else                 odr_val = 0x0C; // 1600 Hz

    uint8_t bw = static_cast<uint8_t>(_active_bw);
    bool ok_acc = writeRegister(REG_ACC_CONF, odr_val | bw);
    bool ok_gyr = writeRegister(REG_GYR_CONF, odr_val | bw);
    _releaseMutex();
    return (ok_acc && ok_gyr);
}

/**
 * @brief Configures Digital Low-Pass Filter (DLPF) oversampling mode.
 * @param bw Filter bandwidth setting (`FilterBandwidth::OSR4`, `OSR2`, or `NORMAL`).
 * @return `true` if updated successfully; `false` on bus error.
 */
bool BMI160_ESPx::setDLPF(FilterBandwidth bw) {
    _active_bw = bw;
    return setDataRate(_active_odr_hz);
}

/**
 * @brief Selects angular units for gyro readings and orientation output.
 * @param enable_degrees `true` for degrees (° and deg/s), `false` for radians (rad and rad/s).
 */
void BMI160_ESPx::setDegrees(bool enable_degrees) {
    _use_degrees = enable_degrees;
}

/**
 * @brief Selects orientation fusion engine (`FusionMode::COMPLEMENTARY` or `MADGWICK`).
 * @param mode Target sensor fusion algorithm.
 */
void BMI160_ESPx::setFusionAlgorithm(FusionMode mode) {
    _fusion_mode = mode;
    resetOrientation();
}

/**
 * @brief Sets weighting factor alpha for the Complementary filter.
 * @param alpha Gyro weighting coefficient between 0.0f and 1.0f (default: 0.98f).
 */
void BMI160_ESPx::setFilterWeight(float alpha) {
    if (alpha >= 0.0f && alpha <= 1.0f) {
        _filter_alpha = alpha;
    }
}

/**
 * @brief Sets filter gain beta for the Madgwick AHRS quaternion algorithm.
 * @param beta Divergence rate in rad/s (default: 0.1f).
 */
void BMI160_ESPx::setMadgwickBeta(float beta) {
    if (beta >= 0.0f) {
        _madgwick_beta = beta;
    }
}

/**
 * @brief Resets pitch, roll, yaw angles to 0 and initializes quaternion to identity ([1, 0, 0, 0]).
 */
void BMI160_ESPx::resetOrientation() {
    _pitch = 0.0f;
    _roll  = 0.0f;
    _yaw   = 0.0f;
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;
    _last_filter_us = esp_timer_get_time();

    // Fast initial gravity alignment if accelerometer data is available
    uint8_t buf[6];
    if (_acquireMutex(pdMS_TO_TICKS(20))) {
        if (readBurst(REG_DATA_ACCEL, buf, 6)) {
            int16_t raw_ax = static_cast<int16_t>((buf[1] << 8) | buf[0]);
            int16_t raw_ay = static_cast<int16_t>((buf[3] << 8) | buf[2]);
            int16_t raw_az = static_cast<int16_t>((buf[5] << 8) | buf[4]);

            float s_ax = static_cast<float>(raw_ax) / _accel_scale;
            float s_ay = static_cast<float>(raw_ay) / _accel_scale;
            float s_az = static_cast<float>(raw_az) / _accel_scale;

            float ax = _remapAxis(_remap_x, s_ax, s_ay, s_az);
            float ay = _remapAxis(_remap_y, s_ax, s_ay, s_az);
            float az = _remapAxis(_remap_z, s_ax, s_ay, s_az);

            float a_norm = sqrt(ax * ax + ay * ay + az * az);
            if (a_norm > 0.5f) {
                float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));
                float roll_rad  = atan2(ay, az);
                _pitch = pitch_rad * (180.0f / static_cast<float>(M_PI));
                _roll  = roll_rad  * (180.0f / static_cast<float>(M_PI));

                float cr = cos(roll_rad * 0.5f);
                float sr = sin(roll_rad * 0.5f);
                float cp = cos(pitch_rad * 0.5f);
                float sp = sin(pitch_rad * 0.5f);

                _q0 = cr * cp;
                _q1 = sr * cp;
                _q2 = cr * sp;
                _q3 = -sr * sp;

                // Sync cached last motion so getters return true state immediately
                float user_p = _invert_pitch ? -_pitch : _pitch;
                float user_r = _invert_roll  ? -_roll  : _roll;
                float user_y = _invert_yaw   ? -_yaw   : _yaw;
                _last_motion.pitch = user_p - _tare_pitch;
                _last_motion.roll  = user_r - _tare_roll;
                _last_motion.yaw   = user_y - _tare_yaw;
                if (!_use_degrees) {
                    static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
                    _last_motion.pitch *= DEG_TO_RAD_F;
                    _last_motion.roll  *= DEG_TO_RAD_F;
                    _last_motion.yaw   *= DEG_TO_RAD_F;
                }
                _last_motion.qw    = _q0;
                _last_motion.qx    = _q1;
                _last_motion.qy    = _q2;
                _last_motion.qz    = _q3;
                _last_motion.ax    = ax;
                _last_motion.ay    = ay;
                _last_motion.az    = az;
            }
        }
        _releaseMutex();
    }
}

float BMI160_ESPx::_remapAxis(int8_t map, float x, float y, float z) {
    switch (map) {
        case  1: return  x;
        case -1: return -x;
        case  2: return  y;
        case -2: return -y;
        case  3: return  z;
        case -3: return -z;
        default: return  x;
    }
}

void BMI160_ESPx::setAxisRemap(int8_t x_axis, int8_t y_axis, int8_t z_axis) {
    _remap_x = x_axis;
    _remap_y = y_axis;
    _remap_z = z_axis;
    resetOrientation();
}

void BMI160_ESPx::invertPitch(bool invert) {
    if (_invert_pitch != invert) {
        _invert_pitch = invert;
        _tare_pitch = -_tare_pitch;
        float user_p = _invert_pitch ? -_pitch : _pitch;
        _last_motion.pitch = user_p - _tare_pitch;
        if (!_use_degrees) _last_motion.pitch *= 0.017453292519943295f;
    }
}

void BMI160_ESPx::invertRoll(bool invert) {
    if (_invert_roll != invert) {
        _invert_roll = invert;
        _tare_roll = -_tare_roll;
        float user_r = _invert_roll ? -_roll : _roll;
        _last_motion.roll = user_r - _tare_roll;
        if (!_use_degrees) _last_motion.roll *= 0.017453292519943295f;
    }
}

void BMI160_ESPx::invertYaw(bool invert) {
    if (_invert_yaw != invert) {
        _invert_yaw = invert;
        _tare_yaw = -_tare_yaw;
        float user_y = _invert_yaw ? -_yaw : _yaw;
        _last_motion.yaw = user_y - _tare_yaw;
        if (!_use_degrees) _last_motion.yaw *= 0.017453292519943295f;
    }
}

void BMI160_ESPx::tare() {
    _tare_pitch = _invert_pitch ? -_pitch : _pitch;
    _tare_roll  = _invert_roll  ? -_roll  : _roll;
    _tare_yaw   = _invert_yaw   ? -_yaw   : _yaw;
    _last_motion.pitch = 0.0f;
    _last_motion.roll  = 0.0f;
    _last_motion.yaw   = 0.0f;
}

void BMI160_ESPx::setTare(float pitch_offset, float roll_offset, float yaw_offset) {
    if (!_use_degrees) {
        static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
        pitch_offset *= RAD_TO_DEG_F;
        roll_offset  *= RAD_TO_DEG_F;
        yaw_offset   *= RAD_TO_DEG_F;
    }
    _tare_pitch = pitch_offset;
    _tare_roll  = roll_offset;
    _tare_yaw   = yaw_offset;
    float user_p = _invert_pitch ? -_pitch : _pitch;
    float user_r = _invert_roll  ? -_roll  : _roll;
    float user_y = _invert_yaw   ? -_yaw   : _yaw;
    _last_motion.pitch = user_p - _tare_pitch;
    _last_motion.roll  = user_r - _tare_roll;
    _last_motion.yaw   = user_y - _tare_yaw;
    if (!_use_degrees) {
        static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
        _last_motion.pitch *= DEG_TO_RAD_F;
        _last_motion.roll  *= DEG_TO_RAD_F;
        _last_motion.yaw   *= DEG_TO_RAD_F;
    }
}

void BMI160_ESPx::getTare(float &pitch_offset, float &roll_offset, float &yaw_offset) const {
    if (!_use_degrees) {
        static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
        pitch_offset = _tare_pitch * DEG_TO_RAD_F;
        roll_offset  = _tare_roll  * DEG_TO_RAD_F;
        yaw_offset   = _tare_yaw   * DEG_TO_RAD_F;
    } else {
        pitch_offset = _tare_pitch;
        roll_offset  = _tare_roll;
        yaw_offset   = _tare_yaw;
    }
}

void BMI160_ESPx::getRawOrientation(float &raw_pitch, float &raw_roll, float &raw_yaw) const {
    if (!_use_degrees) {
        static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
        raw_pitch = _pitch * DEG_TO_RAD_F;
        raw_roll  = _roll  * DEG_TO_RAD_F;
        raw_yaw   = _yaw   * DEG_TO_RAD_F;
    } else {
        raw_pitch = _pitch;
        raw_roll  = _roll;
        raw_yaw   = _yaw;
    }
}

void BMI160_ESPx::resetTare() {
    _tare_pitch = 0.0f;
    _tare_roll  = 0.0f;
    _tare_yaw   = 0.0f;
    float user_p = _invert_pitch ? -_pitch : _pitch;
    float user_r = _invert_roll  ? -_roll  : _roll;
    float user_y = _invert_yaw   ? -_yaw   : _yaw;
    _last_motion.pitch = user_p;
    _last_motion.roll  = user_r;
    _last_motion.yaw   = user_y;
    if (!_use_degrees) {
        static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
        _last_motion.pitch *= DEG_TO_RAD_F;
        _last_motion.roll  *= DEG_TO_RAD_F;
        _last_motion.yaw   *= DEG_TO_RAD_F;
    }
}

// Sensor Fusion Engines (Complementary & Madgwick AHRS)

/**
 * @brief Computes 2-axis tilt attitude and quaternion using the Complementary filter.
 * 
 * Computes roll and pitch references from the accelerometer gravity vector:
 * - Pitch tilt: `pitch_acc = atan2(-ax, sqrt(ay² + az²)) * (180 / π)`
 * - Roll tilt:  `roll_acc  = atan2(ay, az) * (180 / π)`
 * 
 * Fuses gyro angular rates with accelerometer gravity reference using weighting factor alpha:
 * `angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle`
 * 
 * @param ax Calibrated linear acceleration X in g.
 * @param ay Calibrated linear acceleration Y in g.
 * @param az Calibrated linear acceleration Z in g.
 * @param gx Calibrated angular rate X in deg/s.
 * @param gy Calibrated angular rate Y in deg/s.
 * @param gz Calibrated angular rate Z in deg/s.
 * @param dt Integration time interval in seconds.
 */
void BMI160_ESPx::_updateComplementary(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    // Accelerometer tilt reference in degrees
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0f / static_cast<float>(M_PI));
    float roll_acc  = atan2(ay, az) * (180.0f / static_cast<float>(M_PI));

    // Handle wrap-around discontinuity at ±180 degrees
    float roll_diff = roll_acc - _roll;
    if (roll_diff > 180.0f)  roll_diff -= 360.0f;
    if (roll_diff < -180.0f) roll_diff += 360.0f;

    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    float alpha = _filter_alpha;

    // Dynamic Acceleration Rejection:
    // If acceleration deviates significantly from 1g (e.g. hand shaking or flicking),
    // reduce or eliminate accelerometer influence so dynamic forces don't corrupt tilt!
    if (_dynamic_accel_rejection) {
        float err = fabsf(a_norm - 1.0f);
        if (err > 0.25f) {
            alpha = 1.0f; // Pure gyro tracking during heavy motion
        } else if (err > 0.10f) {
            float factor = (err - 0.10f) / 0.15f;
            alpha = _filter_alpha + (1.0f - _filter_alpha) * factor;
        }
    }

    _pitch = alpha * (_pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
    _roll  = alpha * (_roll + gx * dt) + (1.0f - alpha) * (_roll + roll_diff);
    _yaw  += gz * dt;

    // Compute representative quaternion
    float cr = cos(_roll * 0.5f * (static_cast<float>(M_PI) / 180.0f));
    float sr = sin(_roll * 0.5f * (static_cast<float>(M_PI) / 180.0f));
    float cp = cos(_pitch * 0.5f * (static_cast<float>(M_PI) / 180.0f));
    float sp = sin(_pitch * 0.5f * (static_cast<float>(M_PI) / 180.0f));
    float cy = cos(_yaw * 0.5f * (static_cast<float>(M_PI) / 180.0f));
    float sy = sin(_yaw * 0.5f * (static_cast<float>(M_PI) / 180.0f));

    _q0 = cr * cp * cy + sr * sp * sy;
    _q1 = sr * cp * cy - cr * sp * sy;
    _q2 = cr * sp * cy + sr * cp * sy;
    _q3 = cr * cp * sy - sr * sp * cy;
}

/**
 * @brief Computes 6-DOF orientation quaternion and Euler angles using the Madgwick AHRS filter.
 * 
 * Formulates orientation as a unit quaternion, integrating gyroscope angular rates and applying
 * an analytically derived gradient-descent step based on normalized accelerometer gravity vector.
 * 
 * @param ax Calibrated linear acceleration X in g.
 * @param ay Calibrated linear acceleration Y in g.
 * @param az Calibrated linear acceleration Z in g.
 * @param gx_dps Calibrated angular rate X in deg/s.
 * @param gy_dps Calibrated angular rate Y in deg/s.
 * @param gz_dps Calibrated angular rate Z in deg/s.
 * @param dt Integration time interval in seconds.
 */
void BMI160_ESPx::_updateMadgwick(float ax, float ay, float az, float gx_dps, float gy_dps, float gz_dps, float dt) {
    static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
    float gx = gx_dps * DEG_TO_RAD_F;
    float gy = gy_dps * DEG_TO_RAD_F;
    float gz = gz_dps * DEG_TO_RAD_F;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    qDot2 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
    qDot3 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
    qDot4 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    bool use_accel = (a_norm > 0.05f);

    float beta = _madgwick_beta;
    if (_dynamic_accel_rejection) {
        float err = fabsf(a_norm - 1.0f);
        float gyro_mag_dps = sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

        if (err > 0.25f) {
            use_accel = false; // Heavy dynamic motion / flip: ignore linear accel
        } else if (err > 0.10f) {
            float factor = 1.0f - ((err - 0.10f) / 0.15f);
            beta *= factor;
        } else if (err < 0.05f && gyro_mag_dps < 3.0f) {
            // Stationary / Quasi-static on table: boost beta to lock rapidly to true gravity vector
            beta = 0.45f;
        }
    }

    if (use_accel) {
        recipNorm = 1.0f / a_norm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * _q0;
        _2q1 = 2.0f * _q1;
        _2q2 = 2.0f * _q2;
        _2q3 = 2.0f * _q3;
        _4q0 = 4.0f * _q0;
        _4q1 = 4.0f * _q1;
        _4q2 = 4.0f * _q2;
        _8q1 = 8.0f * _q1;
        _8q2 = 8.0f * _q2;
        q0q0 = _q0 * _q0;
        q1q1 = _q1 * _q1;
        q2q2 = _q2 * _q2;
        q3q3 = _q3 * _q3;

        // Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _8q1 * q1q1 + _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _8q2 * q2q2 + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _4q2 * az;
        s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
        float s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (s_norm_sq > 1e-10f) {
            recipNorm = 1.0f / sqrt(s_norm_sq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }
    }

    // Integrate rate of change of quaternion
    _q0 += qDot1 * dt;
    _q1 += qDot2 * dt;
    _q2 += qDot3 * dt;
    _q3 += qDot4 * dt;

    // Normalise quaternion
    float q_norm_sq = _q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3;
    if (q_norm_sq > 1e-10f) {
        recipNorm = 1.0f / sqrt(q_norm_sq);
        _q0 *= recipNorm;
        _q1 *= recipNorm;
        _q2 *= recipNorm;
        _q3 *= recipNorm;
    }

    // Convert quaternion to Euler angles in degrees
    float sin_p = 2.0f * (_q1 * _q3 - _q0 * _q2);
    if (sin_p > 1.0f)  sin_p = 1.0f;
    if (sin_p < -1.0f) sin_p = -1.0f;
    _pitch = -asin(sin_p) * (180.0f / static_cast<float>(M_PI));
    _roll  = atan2(2.0f * (_q0 * _q1 + _q2 * _q3), _q0 * _q0 - _q1 * _q1 - _q2 * _q2 + _q3 * _q3) * (180.0f / static_cast<float>(M_PI));
    _yaw   = atan2(2.0f * (_q1 * _q2 + _q0 * _q3), _q0 * _q0 + _q1 * _q1 - _q2 * _q2 - _q3 * _q3) * (180.0f / static_cast<float>(M_PI));
}

/**
 * @brief Computes delta-time dt and dispatches sensor readings to the active fusion engine.
 * @param ax Calibrated linear acceleration X in g.
 * @param ay Calibrated linear acceleration Y in g.
 * @param az Calibrated linear acceleration Z in g.
 * @param gx Calibrated angular rate X in deg/s.
 * @param gy Calibrated angular rate Y in deg/s.
 * @param gz Calibrated angular rate Z in deg/s.
 * @param now_us Current system timestamp in microseconds (from esp_timer_get_time()).
 */
void BMI160_ESPx::_updateOrientation(float ax, float ay, float az, float gx, float gy, float gz, uint64_t now_us) {
    if (_last_filter_us == 0) {
        _last_filter_us = now_us;
        return;
    }

    float dt = static_cast<float>(now_us - _last_filter_us) / 1000000.0f;
    _last_filter_us = now_us;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

    // Zero-Velocity Update (ZUPT / Stationary deadband):
    // When the board is resting on a surface (accelerometer is ~1.0g and angular rates are tiny noise),
    // clamp angular velocity to 0 to prevent stationary Yaw integration drift!
    if (_zero_velocity_update) {
        float a_mag = sqrt(ax * ax + ay * ay + az * az);
        if (a_mag >= 0.90f && a_mag <= 1.10f &&
            fabsf(gx) < _zupt_threshold_dps &&
            fabsf(gy) < _zupt_threshold_dps &&
            fabsf(gz) < _zupt_threshold_dps) {
            gx = 0.0f;
            gy = 0.0f;
            gz = 0.0f;
        }
    }

    if (_fusion_mode == FusionMode::MADGWICK) {
        _updateMadgwick(ax, ay, az, gx, gy, gz, dt);
    } else {
        _updateComplementary(ax, ay, az, gx, gy, gz, dt);
    }
}

/**
 * @brief Atomically reads high-speed motion telemetry and updates 6-DOF orientation.
 * 
 * Performs a 12-byte burst read of registers 0x0C through 0x17, scales accelerometer and
 * gyroscope readings, subtracts stationary bias, runs orientation filtering, updates
 * rate-limited temperature (every 500 ms), and writes the result into `m`.
 * 
 * @param[out] m Reference to destination `Motion` struct.
 * @return `true` if read transaction and orientation update succeeded; `false` on bus error or timeout.
 */
bool BMI160_ESPx::read(Motion &m) {
    uint8_t buffer[12]; // Strict 12-byte burst read (6 Gyro + 6 Accel)

    if (!_acquireMutex(pdMS_TO_TICKS(20))) {
        _last_error = ErrorCode::ERR_MUTEX_TIMEOUT;
        return false;
    }
    bool ok = readBurst(REG_DATA_GYRO, buffer, 12);
    _releaseMutex();

    if (!ok) {
        _last_error = ErrorCode::ERR_I2C_NACK;
        return false;
    }

    // Parse 16-bit signed Little-Endian values
    int16_t raw_gx = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    int16_t raw_gy = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    int16_t raw_gz = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    int16_t raw_ax = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
    int16_t raw_ay = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
    int16_t raw_az = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);

    // Convert to standard physical units (g and deg/s)
    float raw_scaled_ax = static_cast<float>(raw_ax) / _accel_scale;
    float raw_scaled_ay = static_cast<float>(raw_ay) / _accel_scale;
    float raw_scaled_az = static_cast<float>(raw_az) / _accel_scale;

    float raw_cal_gx = (static_cast<float>(raw_gx) / _gyro_scale) - _bias_gx;
    float raw_cal_gy = (static_cast<float>(raw_gy) / _gyro_scale) - _bias_gy;
    float raw_cal_gz = (static_cast<float>(raw_gz) / _gyro_scale) - _bias_gz;

    // Apply coordinate frame axis remapping (Sensor frame -> Body frame)
    float ax = _remapAxis(_remap_x, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);
    float ay = _remapAxis(_remap_y, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);
    float az = _remapAxis(_remap_z, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);

    float gx_dps = _remapAxis(_remap_x, raw_cal_gx, raw_cal_gy, raw_cal_gz);
    float gy_dps = _remapAxis(_remap_y, raw_cal_gx, raw_cal_gy, raw_cal_gz);
    float gz_dps = _remapAxis(_remap_z, raw_cal_gx, raw_cal_gy, raw_cal_gz);

    uint64_t now_us = esp_timer_get_time();

    // Update orientation filter in degrees
    _updateOrientation(ax, ay, az, gx_dps, gy_dps, gz_dps, now_us);

    // Rate-limited temperature cache update (every 500ms) to conserve I2C bus bandwidth
    if (now_us - _last_temp_read_us >= 500000 || _last_temp_read_us == 0) {
        _last_temp_read_us = now_us;
        getTemperature();
    }

    float gx_out = gx_dps;
    float gy_out = gy_dps;
    float gz_out = gz_dps;

    // Coordinate Transform Pipeline:
    // 1. Transform raw fusion attitude to user body frame (axis inversion)
    float user_pitch = _invert_pitch ? -_pitch : _pitch;
    float user_roll  = _invert_roll  ? -_roll  : _roll;
    float user_yaw   = _invert_yaw   ? -_yaw   : _yaw;

    // 2. Subtract user-space tare offsets
    float pitch_out = user_pitch - _tare_pitch;
    float roll_out  = user_roll  - _tare_roll;
    float yaw_out   = user_yaw   - _tare_yaw;

    // If radians requested, convert from degrees
    if (!_use_degrees) {
        static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
        gx_out    *= DEG_TO_RAD_F;
        gy_out    *= DEG_TO_RAD_F;
        gz_out    *= DEG_TO_RAD_F;
        pitch_out *= DEG_TO_RAD_F;
        roll_out  *= DEG_TO_RAD_F;
        yaw_out   *= DEG_TO_RAD_F;
    }

    m.ax = ax;
    m.ay = ay;
    m.az = az;
    m.gx = gx_out;
    m.gy = gy_out;
    m.gz = gz_out;
    m.pitch = pitch_out;
    m.roll  = roll_out;
    m.yaw   = yaw_out;
    m.qw    = _q0;
    m.qx    = _q1;
    m.qy    = _q2;
    m.qz    = _q3;
    m.temperature = _cached_temp;
    m.timestamp_us = now_us;

    _last_motion = m;
    return true;
}

/**
 * @brief Reads and returns motion telemetry packet by value.
 * @return Motion Latest populated motion data struct.
 */
Motion BMI160_ESPx::read() {
    Motion m;
    read(m);
    return m;
}

/**
 * @brief Reads unscaled raw 16-bit signed integer values directly from sensor registers.
 * @param[out] raw Output reference to `RawMotion` struct.
 * @return `true` if 12-byte burst read succeeded; `false` on bus error or timeout.
 */
bool BMI160_ESPx::readRaw(RawMotion &raw) {
    uint8_t buffer[12];
    if (!_acquireMutex(pdMS_TO_TICKS(20))) return false;
    bool ok = readBurst(REG_DATA_GYRO, buffer, 12);
    _releaseMutex();

    if (!ok) return false;

    raw.gx = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
    raw.gy = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
    raw.gz = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

    raw.ax = static_cast<int16_t>((buffer[7] << 8) | buffer[6]);
    raw.ay = static_cast<int16_t>((buffer[9] << 8) | buffer[8]);
    raw.az = static_cast<int16_t>((buffer[11] << 8) | buffer[10]);

    return true;
}

/**
 * @brief Reads internal die temperature in degrees Celsius (°C).
 * 
 * Resolution: 1/512 °C (0.001953125 °C/LSB) centered at 23 °C.
 * 
 * @return float Temperature in °C.
 */
float BMI160_ESPx::getTemperature() {
    uint8_t buffer[2];
    if (!_acquireMutex(pdMS_TO_TICKS(20))) return _cached_temp;
    bool ok = readBurst(REG_TEMP_L, buffer, 2);
    _releaseMutex();

    if (ok) {
        int16_t raw_temp = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
        if (raw_temp != static_cast<int16_t>(0x8000)) {
            _cached_temp = 23.0f + (static_cast<float>(raw_temp) * 0.001953125f);
        }
    }
    return _cached_temp;
}

// Quick Getters
/** @brief Returns latest pitch angle in degrees or radians. */
float BMI160_ESPx::getPitch()       { return _last_motion.pitch; }
/** @brief Returns latest roll angle in degrees or radians. */
float BMI160_ESPx::getRoll()        { return _last_motion.roll; }
/** @brief Returns latest yaw angle in degrees or radians. */
float BMI160_ESPx::getYaw()         { return _last_motion.yaw; }
/** @brief Returns latest X-axis acceleration in g. */
float BMI160_ESPx::getAccelX()      { return _last_motion.ax; }
/** @brief Returns latest Y-axis acceleration in g. */
float BMI160_ESPx::getAccelY()      { return _last_motion.ay; }
/** @brief Returns latest Z-axis acceleration in g. */
float BMI160_ESPx::getAccelZ()      { return _last_motion.az; }
/** @brief Returns latest X-axis calibrated angular rate. */
float BMI160_ESPx::getGyroX()       { return _last_motion.gx; }
/** @brief Returns latest Y-axis calibrated angular rate. */
float BMI160_ESPx::getGyroY()       { return _last_motion.gy; }
/** @brief Returns latest Z-axis calibrated angular rate. */
float BMI160_ESPx::getGyroZ()       { return _last_motion.gz; }

/**
 * @brief Runs stationary zero-rate calibration by averaging `samples` readings.
 * @warning Device must remain completely stationary during execution.
 * @param samples Number of samples to average (default: 200).
 */
void BMI160_ESPx::calibrate(uint16_t samples, bool run_hardware_foc) {
    if (run_hardware_foc) {
        // Execute on-chip Bosch Fast Offset Compensation
        runHardwareFOC(true);
        delay(50);
    }

    if (samples == 0) {
        resetOrientation();
        return;
    }

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;

    _bias_gx = 0.0f;
    _bias_gy = 0.0f;
    _bias_gz = 0.0f;

    uint16_t valid = 0;
    uint8_t buf[12]; // Read Gyro (0x0C..0x11) + Accel (0x12..0x17) simultaneously

    for (uint16_t i = 0; i < samples; ++i) {
        if (_acquireMutex(pdMS_TO_TICKS(20))) {
            if (readBurst(REG_DATA_GYRO, buf, 12)) {
                int16_t raw_gx = static_cast<int16_t>((buf[1] << 8) | buf[0]);
                int16_t raw_gy = static_cast<int16_t>((buf[3] << 8) | buf[2]);
                int16_t raw_gz = static_cast<int16_t>((buf[5] << 8) | buf[4]);

                int16_t raw_ax = static_cast<int16_t>((buf[7] << 8) | buf[6]);
                int16_t raw_ay = static_cast<int16_t>((buf[9] << 8) | buf[8]);
                int16_t raw_az = static_cast<int16_t>((buf[11] << 8) | buf[10]);

                sum_gx += static_cast<float>(raw_gx) / _gyro_scale;
                sum_gy += static_cast<float>(raw_gy) / _gyro_scale;
                sum_gz += static_cast<float>(raw_gz) / _gyro_scale;

                sum_ax += static_cast<float>(raw_ax) / _accel_scale;
                sum_ay += static_cast<float>(raw_ay) / _accel_scale;
                sum_az += static_cast<float>(raw_az) / _accel_scale;

                valid++;
            }
            _releaseMutex();
        }
        delay(4);
    }

    if (valid > 0) {
        float f_valid = static_cast<float>(valid);
        _bias_gx = sum_gx / f_valid;
        _bias_gy = sum_gy / f_valid;
        _bias_gz = sum_gz / f_valid;

        // Apply body-frame axis remapping to the averaged gravity vector
        float mean_s_ax = sum_ax / f_valid;
        float mean_s_ay = sum_ay / f_valid;
        float mean_s_az = sum_az / f_valid;

        float ax = _remapAxis(_remap_x, mean_s_ax, mean_s_ay, mean_s_az);
        float ay = _remapAxis(_remap_y, mean_s_ax, mean_s_ay, mean_s_az);
        float az = _remapAxis(_remap_z, mean_s_ax, mean_s_ay, mean_s_az);

        float a_norm = sqrt(ax * ax + ay * ay + az * az);
        if (a_norm > 0.5f) {
            float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));
            float roll_rad  = atan2(ay, az);
            _pitch = pitch_rad * (180.0f / static_cast<float>(M_PI));
            _roll  = roll_rad  * (180.0f / static_cast<float>(M_PI));
            _yaw   = 0.0f;

            float cr = cos(roll_rad * 0.5f);
            float sr = sin(roll_rad * 0.5f);
            float cp = cos(pitch_rad * 0.5f);
            float sp = sin(pitch_rad * 0.5f);

            _q0 = cr * cp;
            _q1 = sr * cp;
            _q2 = cr * sp;
            _q3 = -sr * sp;

            // Sync cached last motion so getters return exact state immediately
            float user_p = _invert_pitch ? -_pitch : _pitch;
            float user_r = _invert_roll  ? -_roll  : _roll;
            float user_y = _invert_yaw   ? -_yaw   : _yaw;
            _last_motion.pitch = user_p - _tare_pitch;
            _last_motion.roll  = user_r - _tare_roll;
            _last_motion.yaw   = user_y - _tare_yaw;
            if (!_use_degrees) {
                static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
                _last_motion.pitch *= DEG_TO_RAD_F;
                _last_motion.roll  *= DEG_TO_RAD_F;
                _last_motion.yaw   *= DEG_TO_RAD_F;
            }
            _last_motion.qw = _q0;
            _last_motion.qx = _q1;
            _last_motion.qy = _q2;
            _last_motion.qz = _q3;
            _last_motion.ax = ax;
            _last_motion.ay = ay;
            _last_motion.az = az;
            _last_motion.gx = 0.0f;
            _last_motion.gy = 0.0f;
            _last_motion.gz = 0.0f;
            _last_motion.timestamp_us = esp_timer_get_time();
            _last_filter_us = _last_motion.timestamp_us;
            return;
        }
    }

    // Fallback if no valid samples were obtained
    resetOrientation();
}

/**
 * @brief Sets stationary gyroscope bias offsets directly.
 * @param bx X-axis bias in deg/s.
 * @param by Y-axis bias in deg/s.
 * @param bz Z-axis bias in deg/s.
 */
void BMI160_ESPx::setGyroOffsets(float bx, float by, float bz) {
    _bias_gx = bx;
    _bias_gy = by;
    _bias_gz = bz;
}

/**
 * @brief Reads current stationary gyroscope bias offsets.
 * @param[out] bx Output reference for X-axis bias in deg/s.
 * @param[out] by Output reference for Y-axis bias in deg/s.
 * @param[out] bz Output reference for Z-axis bias in deg/s.
 */
void BMI160_ESPx::getGyroOffsets(float &bx, float &by, float &bz) const {
    bx = _bias_gx;
    by = _bias_gy;
    bz = _bias_gz;
}

// Advanced Silicon Features (Hardware FOC, FIFO, Power Management)

/**
 * @brief Runs on-chip Fast Offset Compensation (FOC) hardware calibration.
 * 
 * Instructs the BMI160 hardware engine to calibrate accelerometer and gyroscope trims.
 * Calibrated offsets are written to on-chip registers (0x71-0x76) and permanently activated.
 * 
 * @warning The device MUST remain motionless on a flat surface throughout the procedure.
 * 
 * @param z_up `true` if Z axis points upwards (+1g on Z, 0g on X/Y); `false` if inverted (-1g on Z).
 * @return `true` if FOC finished successfully and offsets were enabled; `false` on timeout or bus failure.
 */
bool BMI160_ESPx::runHardwareFOC(bool z_up) {
    if (!_acquireMutex()) return false;

    // FOC_CONF: foc_gyr_en = 1 (bit 6), acc_z = 0b01 (+1g) or 0b10 (-1g), acc_y = 0b11 (0g), acc_x = 0b11 (0g)
    uint8_t foc_acc_z = z_up ? 0x01 : 0x02;
    uint8_t foc_conf = (1 << 6) | (foc_acc_z << 4) | (0x03 << 2) | 0x03;

    if (!writeRegister(REG_FOC_CONF, foc_conf)) {
        _releaseMutex();
        return false;
    }

    writeRegister(REG_CMD, CMD_START_FOC);

    uint8_t status = 0;
    uint32_t start_ms = millis();
    bool completed = false;

    while (millis() - start_ms < 400) {
        delay(20);
        if (readRegister(REG_STATUS, status)) {
            if (status & (1 << 3)) { // Bit 3: foc_rdy
                completed = true;
                break;
            }
        }
    }

    if (completed) {
        uint8_t off_en = 0;
        readRegister(REG_OFFSET_EN, off_en);
        writeRegister(REG_OFFSET_EN, off_en | 0xC0); // Preserve MSB bits [5:0] and enable Accel & Gyro hardware offsets
    }

    _releaseMutex();
    return completed;
}

/**
 * @brief Sets operating power mode for accelerometer and gyroscope cores.
 * 
 * Transitions sensors between `PowerMode::NORMAL`, `LOW_POWER`, and `SUSPEND`.
 * 
 * @param mode Target power mode.
 * @return `true` if power transition commands succeeded; `false` on I2C write error.
 */
bool BMI160_ESPx::setPowerMode(PowerMode mode) {
    if (!_acquireMutex()) return false;
    bool ok = true;
    switch (mode) {
        case PowerMode::NORMAL:
            ok &= writeRegister(REG_CMD, CMD_ACC_PM_NORMAL);
            delay(10);
            ok &= writeRegister(REG_CMD, CMD_GYR_PM_NORMAL);
            delay(85);
            break;
        case PowerMode::LOW_POWER:
            ok &= writeRegister(REG_CMD, CMD_ACC_PM_LOWPOWER);
            delay(10);
            ok &= writeRegister(REG_CMD, CMD_GYR_PM_SUSPEND);
            break;
        case PowerMode::SUSPEND:
            ok &= writeRegister(REG_CMD, CMD_ACC_PM_SUSPEND);
            delay(5);
            ok &= writeRegister(REG_CMD, CMD_GYR_PM_SUSPEND);
            break;
    }
    _releaseMutex();
    return ok;
}

/**
 * @brief Configures and enables the 1024-byte hardware FIFO buffer.
 * @param accel If `true`, enables accelerometer frames in FIFO.
 * @param gyro If `true`, enables gyroscope frames in FIFO.
 * @return `true` if configured and flushed successfully; `false` on bus error.
 */
bool BMI160_ESPx::enableFIFO(bool accel, bool gyro) {
    if (!_acquireMutex()) return false;
    uint8_t conf1 = 0;
    if (gyro)  conf1 |= (1 << 7);
    if (accel) conf1 |= (1 << 6);
    writeRegister(REG_FIFO_CONFIG_0, 0x00);
    bool ok = writeRegister(REG_FIFO_CONFIG_1, conf1);
    flushFIFO();
    _releaseMutex();
    return ok;
}

/**
 * @brief Reads number of bytes currently stored in hardware FIFO buffer.
 * @return uint16_t Queued byte count (0 to 1024 bytes); returns 0 on error.
 */
uint16_t BMI160_ESPx::getFIFOCount() {
    uint8_t buf[2];
    if (!_acquireMutex(pdMS_TO_TICKS(20))) return 0;
    bool ok = readBurst(REG_FIFO_LENGTH_0, buf, 2);
    _releaseMutex();
    if (!ok) return 0;
    return static_cast<uint16_t>(buf[0] | ((buf[1] & 0x07) << 8));
}

/**
 * @brief Flushes and empties the on-chip hardware FIFO buffer.
 */
void BMI160_ESPx::flushFIFO() {
    if (!_acquireMutex()) return;
    writeRegister(REG_CMD, CMD_FIFO_FLUSH);
    _releaseMutex();
}

/**
 * @brief Reads and unpacks sensor frames from FIFO buffer into `buffer` array.
 * 
 * Unpacks 12-byte frames (6 bytes gyro + 6 bytes accel), converts into physical units,
 * subtracts gyro zero-rate bias, attaches temperature, and populates `buffer`.
 * 
 * @param[out] buffer Pointer to pre-allocated array of `Motion` structures.
 * @param max_samples Maximum capacity of `buffer` in number of frames.
 * @return uint16_t Number of frames successfully read and unpacked.
 */
uint16_t BMI160_ESPx::readFIFO(Motion *buffer, uint16_t max_samples) {
    if (buffer == nullptr || max_samples == 0) return 0;
    if (!_acquireMutex(pdMS_TO_TICKS(50))) return 0;

    uint16_t byte_count = getFIFOCount();
    uint16_t available_frames = byte_count / 12;
    uint16_t frames_to_read = (available_frames > max_samples) ? max_samples : available_frames;

    if (frames_to_read == 0) {
        _releaseMutex();
        return 0;
    }

    for (uint16_t i = 0; i < frames_to_read; ++i) {
        uint8_t raw[12];
        if (!readBurst(REG_FIFO_DATA, raw, 12)) break;

        int16_t raw_gx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        int16_t raw_gy = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        int16_t raw_gz = static_cast<int16_t>((raw[5] << 8) | raw[4]);

        int16_t raw_ax = static_cast<int16_t>((raw[7] << 8) | raw[6]);
        int16_t raw_ay = static_cast<int16_t>((raw[9] << 8) | raw[8]);
        int16_t raw_az = static_cast<int16_t>((raw[11] << 8) | raw[10]);

        float raw_scaled_ax = static_cast<float>(raw_ax) / _accel_scale;
        float raw_scaled_ay = static_cast<float>(raw_ay) / _accel_scale;
        float raw_scaled_az = static_cast<float>(raw_az) / _accel_scale;

        float raw_cal_gx = (static_cast<float>(raw_gx) / _gyro_scale) - _bias_gx;
        float raw_cal_gy = (static_cast<float>(raw_gy) / _gyro_scale) - _bias_gy;
        float raw_cal_gz = (static_cast<float>(raw_gz) / _gyro_scale) - _bias_gz;

        buffer[i].ax = _remapAxis(_remap_x, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);
        buffer[i].ay = _remapAxis(_remap_y, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);
        buffer[i].az = _remapAxis(_remap_z, raw_scaled_ax, raw_scaled_ay, raw_scaled_az);

        buffer[i].gx = _remapAxis(_remap_x, raw_cal_gx, raw_cal_gy, raw_cal_gz);
        buffer[i].gy = _remapAxis(_remap_y, raw_cal_gx, raw_cal_gy, raw_cal_gz);
        buffer[i].gz = _remapAxis(_remap_z, raw_cal_gx, raw_cal_gy, raw_cal_gz);
        buffer[i].temperature = _cached_temp;
    }

    _releaseMutex();
    return frames_to_read;
}

// Pedometer & Hardware Interrupts (Thread-Safe)

/**
 * @brief Enables or disables on-chip hardware step detector and counter.
 * @param enable `true` to enable step counting; `false` to disable.
 */
void BMI160_ESPx::enablePedometer(bool enable) {
    if (!_acquireMutex()) return;
    writeRegister(REG_STEP_CONF_0, 0x15);
    writeRegister(REG_STEP_CONF_1, enable ? 0x0B : 0x03);
    _releaseMutex();
}

/**
 * @brief Reads total accumulated step count from hardware step counter.
 * @return uint16_t Step count since boot or last `resetSteps()`.
 */
uint16_t BMI160_ESPx::getSteps() {
    uint8_t buf[2];
    if (!_acquireMutex(pdMS_TO_TICKS(20))) return 0;
    bool ok = readBurst(REG_STEP_CNT_0, buf, 2);
    _releaseMutex();

    if (!ok) return 0;
    return static_cast<uint16_t>(buf[0] | (buf[1] << 8));
}

/**
 * @brief Clears hardware step counter back to 0.
 */
void BMI160_ESPx::resetSteps() {
    if (!_acquireMutex()) return;
    writeRegister(REG_CMD, CMD_STEP_CNT_CLR);
    _releaseMutex();
}

/**
 * @brief Configures Any-Motion slope interrupt across all axes.
 * @param threshold_lsb Acceleration threshold in LSB (default: 20, ~78.2 mg at ±2g).
 * @param int_pin Physical output pin (1 for INT1, 2 for INT2; default: 1).
 * @return `true` if interrupt was configured and mapped successfully; `false` on error.
 */
bool BMI160_ESPx::enableMotionInterrupt(uint8_t threshold_lsb, int int_pin) {
    if (!_acquireMutex()) return false;
    writeRegister(REG_INT_MOTION_0, threshold_lsb);
    writeRegister(REG_INT_MOTION_1, 0x02);

    uint8_t en0 = 0;
    readRegister(REG_INT_EN_0, en0);
    writeRegister(REG_INT_EN_0, en0 | 0x07);

    if (int_pin == 1) {
        writeRegister(REG_INT_MAP_0, 0x04);
    } else {
        writeRegister(REG_INT_MAP_1, 0x04);
    }

    uint8_t current_ctrl = 0;
    readRegister(REG_INT_OUT_CTRL, current_ctrl);
    uint8_t pin_mask = (int_pin == 1) ? 0x0A : 0xA0;
    writeRegister(REG_INT_OUT_CTRL, current_ctrl | pin_mask);
    _releaseMutex();
    return true;
}

/**
 * @brief Configures single or double tap detection interrupt.
 * @param double_tap `true` for double-tap; `false` for single-tap (default: `true`).
 * @param threshold_lsb Tap shock threshold in LSB (default: 10).
 * @param int_pin Physical output pin (1 for INT1, 2 for INT2; default: 1).
 * @return `true` if interrupt was configured and mapped successfully; `false` on error.
 */
bool BMI160_ESPx::enableTapInterrupt(bool double_tap, uint8_t threshold_lsb, int int_pin) {
    if (!_acquireMutex()) return false;
    writeRegister(REG_INT_TAP_0, 0x04);
    writeRegister(REG_INT_TAP_1, threshold_lsb & 0x1F);

    uint8_t en0 = 0;
    readRegister(REG_INT_EN_0, en0);
    en0 |= double_tap ? (1 << 4) : (1 << 5);
    writeRegister(REG_INT_EN_0, en0);

    if (int_pin == 1) {
        writeRegister(REG_INT_MAP_0, double_tap ? (1 << 4) : (1 << 5));
    } else {
        writeRegister(REG_INT_MAP_1, double_tap ? (1 << 4) : (1 << 5));
    }

    uint8_t current_ctrl = 0;
    readRegister(REG_INT_OUT_CTRL, current_ctrl);
    uint8_t pin_mask = (int_pin == 1) ? 0x0A : 0xA0;
    writeRegister(REG_INT_OUT_CTRL, current_ctrl | pin_mask);
    _releaseMutex();
    return true;
}

/**
 * @brief Scans all 127 I2C addresses and prints detected devices to Serial.
 * @return uint8_t Number of responsive I2C devices discovered.
 */
uint8_t BMI160_ESPx::scanBus() {
    if (!_acquireMutex(pdMS_TO_TICKS(500))) return 0;
    uint8_t found = 0;
    for (uint8_t a = 1; a < 127; ++a) {
        _wire.beginTransmission(a);
        if (_wire.endTransmission() == 0) {
            Serial.printf("  -> I2C device responding at address: 0x%02X\n", a);
            found++;
        }
    }
    if (found == 0) {
        Serial.println("  -> No I2C devices detected! Verify 3.3V, GND, SDA (GPIO 21), and SCL (GPIO 22).");
    }
    _releaseMutex();
    return found;
}

// FreeRTOS Streaming Worker

/**
 * @brief Static FreeRTOS task entry trampoline function.
 * @param param Pointer to the `BMI160_ESPx` driver instance.
 */
void BMI160_ESPx::_taskRunner(void *param) {
    auto *self = static_cast<BMI160_ESPx*>(param);
    self->_taskLoop();
}

/**
 * @brief High-priority background task loop that samples IMU and enqueues telemetry.
 */
void BMI160_ESPx::_taskLoop() {
    TickType_t period = pdMS_TO_TICKS(1000 / _stream_hz);
    if (period == 0) period = 1;

    TickType_t wake = xTaskGetTickCount();
    Motion m;

    while (_stream_running) {
        if (read(m)) {
            if (_target_queue != nullptr) {
                xQueueSend(_target_queue, &m, 0);
            }
        }
        vTaskDelayUntil(&wake, period);
    }

    _task_handle = nullptr;
    vTaskDelete(nullptr);
}

/**
 * @brief Spawns a dedicated FreeRTOS background task pinned to a core for deterministic streaming.
 * 
 * @param rate_hz Streaming rate in Hz (e.g. 50, 100, 200). Must be > 0.
 * @param out_queue Target FreeRTOS queue handle created for `Motion` items.
 * @param core ESP32 core index (0 or 1, default: 1).
 * @return `true` if task was successfully spawned; `false` on parameter error or allocation failure.
 */
bool BMI160_ESPx::startAsyncStream(uint16_t rate_hz, QueueHandle_t out_queue, BaseType_t core) {
    if (_task_handle != nullptr || out_queue == nullptr || rate_hz == 0) return false;

    _stream_hz      = rate_hz;
    _target_queue   = out_queue;
    _stream_running = true;

    BaseType_t res = xTaskCreatePinnedToCore(
        _taskRunner,
        "BMI160_Async",
        4096,
        this,
        configMAX_PRIORITIES - 2,
        &_task_handle,
        core
    );

    if (res != pdPASS) {
        _stream_running = false;
        _task_handle = nullptr;
        return false;
    }
    return true;
}

/**
 * @brief Gracefully terminates the background FreeRTOS streaming task and unbinds queue.
 */
void BMI160_ESPx::stopAsyncStream() {
    if (_task_handle != nullptr) {
        TaskHandle_t th = _task_handle;
        _stream_running = false;
        _task_handle = nullptr;
        _target_queue = nullptr;
        vTaskDelete(th);
    }
}

// Low-Level Bus Helpers

/**
 * @brief Writes an 8-bit byte to a specific BMI160 register.
 * @param reg Register address.
 * @param value Byte value to write.
 * @return `true` if acknowledged by slave (ACK); `false` on NACK or bus error.
 */
bool BMI160_ESPx::writeRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value);
    return (_wire.endTransmission() == 0);
}

/**
 * @brief Reads a single 8-bit byte from a specific BMI160 register.
 * @param reg Register address to read from.
 * @param[out] value Reference where the read byte will be stored.
 * @return `true` if byte was received successfully; `false` on bus error.
 */
bool BMI160_ESPx::readRegister(uint8_t reg, uint8_t &value) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;

    if (_wire.requestFrom(static_cast<uint8_t>(_addr), static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    value = _wire.read();
    return true;
}

/**
 * @brief Reads multiple consecutive bytes starting from a register address.
 * @param start_reg Starting register address.
 * @param[out] buf Pointer to destination buffer of at least `length` bytes.
 * @param length Number of bytes to read.
 * @return `true` if all bytes were read successfully; `false` on pointer error or bus error.
 */
bool BMI160_ESPx::readBurst(uint8_t start_reg, uint8_t *buf, size_t length) {
    if (buf == nullptr || length == 0) return false;

    _wire.beginTransmission(_addr);
    _wire.write(start_reg);
    if (_wire.endTransmission(false) != 0) return false;

    size_t got = _wire.requestFrom(static_cast<uint8_t>(_addr), static_cast<uint8_t>(length));
    if (got != length) return false;

    for (size_t i = 0; i < length; ++i) {
        buf[i] = _wire.read();
    }
    return true;
}
