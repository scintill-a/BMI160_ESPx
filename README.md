# BMI160_ESPx

An ESP32 Arduino/PlatformIO library for the Bosch BMI160 6-axis accelerometer and gyroscope.

It handles I2C communication, sensor configuration, basic bus recovery, stationary calibration, and orientation estimation (Madgwick AHRS or Complementary filter).

---

## What It Includes

- **Sensor readings:** Linear acceleration (`ax, ay, az`), angular rate (`gx, gy, gz`), and die temperature.
- **Orientation estimation:** Madgwick AHRS (pitch, roll, yaw, and quaternions) and Complementary filter.
- **Drift mitigation:** Stationary deadband (ZUPT) to reduce yaw drift when motionless, and acceleration gating during sudden movement.
- **Bus recovery:** 9-clock recovery pulse on startup if the I2C bus is stuck low.
- **Address check:** Probes both `0x68` and `0x69`.
- **Coordinate adjustments:** Axis remapping (`setAxisRemap`), sign inversion (`invertPitch/Roll/Yaw`), and zero taring (`tare()`).
- **Thread safety:** FreeRTOS recursive mutex around I2C transactions.

---

## Installation

### PlatformIO

Add the repository URL to `lib_deps` in `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/scintill-a/BMI160_ESPx.git
```

---

## Wiring

| BMI160 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | Connect to 3.3V power (not 5V). |
| **GND** | **GND** | Ground. |
| **SDA** | **GPIO 21** | Default ESP32 I2C data pin. |
| **SCL** | **GPIO 22** | Default ESP32 I2C clock pin. |
| **SA0 / SDO** | *(Leave floating)* | Optional. Pulled to GND (`0x68`) by default. Connect to 3.3V for `0x69`. |

---

## Quick Start

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    // Initialize with SDA=21, SCL=22
    if (!imu.begin(21, 22)) {
        Serial.println("BMI160 not detected. Check wiring.");
        while (1) delay(100);
    }

    // Set full-scale ranges and rate:
    imu.setAccelRange(2);   // ±2g
    imu.setGyroRange(250);  // ±250 °/s
    imu.setDataRate(100);   // 100 Hz

    // Keep the sensor still while calibrating gyro bias
    imu.calibrate(200);
}

void loop() {
    Motion m = imu.read();
    Serial.printf("Pitch: %6.1f° | Roll: %6.1f° | Yaw: %6.1f°\n", m.pitch, m.roll, m.yaw);
    delay(20);
}
```

---

## Orientation Filters

### 1. Madgwick AHRS (Default)
Combines accelerometer and gyroscope data to produce quaternions and Euler angles:
```cpp
imu.setFusionAlgorithm(FusionMode::MADGWICK); // Default
imu.setMadgwickBeta(0.1f);                    // Filter gain (default: 0.1f)
```
Outputs quaternions (`m.qw, m.qx, m.qy, m.qz`) and Euler angles (`m.pitch, m.roll, m.yaw`).

### 2. Complementary Filter
A simpler tilt filter combining integrated gyro rates with gravity tilt:
```cpp
imu.setFusionAlgorithm(FusionMode::COMPLEMENTARY);
imu.setFilterWeight(0.98f);                   // Alpha weight in [0.0, 1.0] (default: 0.98f)
```

---

## Common Methods

### Setup & Calibration
```cpp
bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0);
void calibrate(uint16_t samples = 200);
uint8_t getAddress(); // Returns detected I2C address (0x68 or 0x69)
```

### Reading Telemetry
```cpp
Motion read();                 // Returns populated Motion struct
bool read(Motion &m);          // Pass-by-reference read, returns true on success
const Motion& getLastMotion(); // Reference to the last read sample
float getPitch();              // Current pitch angle
float getRoll();               // Current roll angle
float getYaw();                // Current yaw angle
float getTemperature();        // Die temperature in °C
```

### Configuration
```cpp
imu.setAccelRange(2);               // 2, 4, 8, 16 (in g)
imu.setGyroRange(250);              // 125, 250, 500, 1000, 2000 (in °/s)
imu.setDataRate(100);               // 25, 50, 100, 200, 400, 800, 1600 (in Hz)
imu.setDLPF(FilterBandwidth::OSR4); // OSR4, OSR2, NORMAL
imu.setDegrees(true);               // true for degrees (°), false for radians
```

### Offsets & Coordinate Remapping
```cpp
imu.setAxisRemap(1, 2, 3); // Map sensor axes (+/-1, +/-2, +/-3) to board orientation
imu.invertPitch(true);     // Invert pitch direction
imu.tare();                // Set current orientation as zero reference
imu.resetTare();           // Clear tare offsets back to zero
```

---

## Examples

Example sketches are in the [examples/](examples/) folder:
- **[01_BasicMotion](examples/01_BasicMotion/01_BasicMotion.ino)**: Initializing the sensor and reading orientation.
- **[02_SensorFusion](examples/02_SensorFusion/02_SensorFusion.ino)**: Tuning Madgwick and Complementary filters.

---

## License

MIT License. See [LICENSE](LICENSE) for details.
