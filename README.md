# BMI160_ESPx

An ESP32 Arduino and PlatformIO library for the Bosch BMI160 6-axis inertial measurement unit (accelerometer and gyroscope).

---

## Features

- **Raw and scaled data:** Reads acceleration in g (`ax`, `ay`, `az`) and angular velocity in degrees/s (`gx`, `gy`, `gz`).
- **Sensor fusion options:** Includes both a **Complementary filter** and a 6-DOF **Madgwick AHRS filter** for real-time pitch, roll, yaw, and quaternion calculations.
- **Configurable ranges and rates:**
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±125, ±250, ±500, ±1000, ±2000 °/s
  - Output data rate: 25 Hz to 1600 Hz
- **I2C bus recovery:** Includes clock-pulsing bus recovery before initializing `Wire` to clear hung I2C states.
- **Auto-address detection:** Checks both standard BMI160 I2C addresses (`0x68` and `0x69`).
- **Hardware pedometer:** Reads the chip's internal step detector.
- **FreeRTOS support:** Optional background sampling task pinned to a specific ESP32 core.

---

## Wiring (ESP32)

| BMI160 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Connect to 3.3V logic (do not use 5V) |
| **GND** | GND | Common ground |
| **SDA** | GPIO 21 | Default I2C SDA (configurable) |
| **SCL** | GPIO 22 | Default I2C SCL (configurable) |
| **SA0 / SDO** | GND or 3.3V | GND = `0x68`, 3.3V = `0x69` |

---

## Installation

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/scintill-a/BMI160_ESPx.git
```
Or clone/copy into your project's `lib/` directory.

### Arduino IDE
Download or clone this repository into your `Arduino/libraries/` folder.

---

## Basic Example

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    // Initialize on SDA=21, SCL=22
    if (!imu.begin(21, 22)) {
        Serial.println("Failed to find BMI160 sensor. Check wiring.");
        while (1) delay(100);
    }

    // Optional: calibrate stationary gyro bias
    imu.calibrate();
}

void loop() {
    Motion m = imu.read();

    Serial.printf("Pitch: %5.1f° | Roll: %5.1f° | Accel: (%+.2f, %+.2f, %+.2f) g\n",
                  m.pitch, m.roll, m.ax, m.ay, m.az);

    delay(20);
}
```

---

## Sensor Fusion

The library includes two built-in orientation estimation algorithms:

### 1. Madgwick AHRS (`FusionMode::MADGWICK`, Default)
A 6-DOF gradient-descent quaternion filter. It computes complete 3D spatial attitude without gimbal lock, providing normalized quaternions (`q0`, `q1`, `q2`, `q3`) alongside `pitch`, `roll`, and `yaw`.
```cpp
imu.setFusionAlgorithm(FusionMode::MADGWICK);
imu.setMadgwickBeta(0.1f); // Tuning parameter: divergence gain beta (default: 0.1f)
```

### 2. Complementary Filter (`FusionMode::COMPLEMENTARY`)
A lightweight 2-axis tilt filter combining integrated gyro rates with the accelerometer gravity vector:
```text
angle = alpha × (angle + gyro × dt) + (1 - alpha) × accel_angle
```
```cpp
imu.setFusionAlgorithm(FusionMode::COMPLEMENTARY);
imu.setFilterWeight(0.98f); // Weight factor alpha in [0.0, 1.0] (default: 0.98f)
```

---

## API Reference

### Initialization & Calibration
```cpp
bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000);
bool calibrate(int samples = 200);
```

### Sensor & Fusion Configuration
```cpp
void setAccelRange(int range_g = 2);     // 2, 4, 8, 16
void setGyroRange(int range_dps = 250);  // 125, 250, 500, 1000, 2000
void setDataRate(int rate_hz = 100);     // 25, 50, 100, 200, 400, etc.
void setDegrees(bool use_deg = true);    // true for degrees, false for radians

// Fusion algorithm selection & tuning
void setFusionAlgorithm(FusionMode mode);// FusionMode::MADGWICK or FusionMode::COMPLEMENTARY
void setMadgwickBeta(float beta = 0.1f); // Beta gain for Madgwick AHRS
void setFilterWeight(float alpha = 0.98f);// Weight alpha for Complementary filter
```

### Data Reading
```cpp
Motion read();               // Returns ax, ay, az, gx, gy, gz, pitch, roll, yaw
float getPitch();
float getRoll();
float getTemperature();
```

### Hardware Pedometer
```cpp
void enablePedometer(bool enable = true);
uint16_t getSteps();
void resetSteps();
```

### FreeRTOS Background Task
```cpp
// Runs continuous sampling on Core 1 at the specified rate
bool startWorker(int core = 1, int priority = 2);
void stopWorker();
```

### Direct Register Access
```cpp
bool writeRegister(uint8_t reg, uint8_t value);
bool readRegister(uint8_t reg, uint8_t &value);
bool readBurst(uint8_t start_reg, uint8_t *buffer, size_t length);
```

---

## Examples

Included in the [examples](examples/) directory:
- `01_BasicMotion`: Reads raw accelerometer, gyroscope, and computed pitch/roll.
- `02_Pedometer`: Step counting with on-chip hardware pedometer.
- `03_FreeRTOS_Worker`: Background task sampling without blocking `loop()`.

---

## License

MIT License. See [LICENSE](LICENSE) for details.
