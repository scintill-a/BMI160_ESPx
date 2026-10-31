# BMI160_ESPx

A streamlined, high-performance ESP32 driver for the Bosch BMI160 6-axis IMU (accelerometer and gyroscope) with built-in sensor fusion (Madgwick AHRS / Complementary tilt), active I2C bus recovery, stationary calibration, and tare zeroing.

---

## Features

- **6-DOF Motion:** Reads linear acceleration (`ax`, `ay`, `az`), angular velocity (`gx`, `gy`, `gz`), and temperature in physical engineering units.
- **Real-Time Sensor Fusion:** Built-in **Madgwick AHRS** (quaternion 3D attitude with zero gimbal lock) and **Complementary** tilt filters.
- **Dynamic Motion Rejection & ZUPT:** Gating to reject linear motion acceleration spikes during rapid movements, and Zero-Velocity Update (ZUPT) deadband to prevent stationary yaw drift.
- **I2C Bus Auto-Recovery:** Automatic 9-clock recovery to clear stuck slave buses if line lockups occur.
- **Auto-Address Probing:** Automatically detects both `0x68` (SDO to GND) and `0x69` (SDO to 3.3V).
- **Coordinate Transformation & Tare:** Full axis remapping (`setAxisRemap`), sign inversion (`invertPitch/Roll/Yaw`), and runtime tare offsets.
- **Thread-Safe:** FreeRTOS recursive mutex guarantees safe multi-task I2C bus access on dual-core ESP32 systems.

---

## Installation

### PlatformIO

Add the repository URL to `lib_deps` in your `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/scintill-a/BMI160_ESPx.git
```

---

## Wiring (ESP32)

| BMI160 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | Connect to 3.3V (do not use 5V). |
| **GND** | **GND** | Common ground. |
| **SDA** | **GPIO 21** | Default ESP32 I2C data pin. |
| **SCL** | **GPIO 22** | Default ESP32 I2C clock pin. |
| **SA0 / SDO** | *(Leave floating)* | Optional. Default pulls to GND (`0x68`). Connect to 3.3V for `0x69`. |

---

## Quick Start

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    if (!imu.begin(21, 22)) {
        Serial.println("BMI160 not found, check wiring.");
        while (1) delay(100);
    }

    // Configure ranges and rate:
    imu.setAccelRange(2);   // ±2g
    imu.setGyroRange(250);  // ±250 °/s
    imu.setDataRate(100);   // 100 Hz

    // Quick calibration (keep sensor still)
    imu.calibrate(200);
}

void loop() {
    Motion m = imu.read();
    Serial.printf("Pitch: %6.1f° | Roll: %6.1f° | Yaw: %6.1f°\n", m.pitch, m.roll, m.yaw);
    delay(20);
}
```

---

## Sensor Fusion & Tuning

### 1. Madgwick AHRS (Default)
A 6-DOF quaternion filter for complete 3D orientation without gimbal lock:
```cpp
imu.setFusionAlgorithm(FusionMode::MADGWICK); // Default
imu.setMadgwickBeta(0.1f);                    // Divergence gain beta (default: 0.1f)
```
Outputs quaternions (`m.qw`, `m.qx`, `m.qy`, `m.qz`) and Euler angles (`m.pitch`, `m.roll`, `m.yaw`).

### 2. Complementary Filter
A lightweight tilt filter combining gyro integration with accelerometer gravity tilt:
```cpp
imu.setFusionAlgorithm(FusionMode::COMPLEMENTARY);
imu.setFilterWeight(0.98f);                   // Weight alpha in [0.0, 1.0] (default: 0.98f)
```

---

## API Cheatsheet

### Initialization & Calibration
```cpp
bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0);
void calibrate(uint16_t samples = 200);
uint8_t getAddress(); // Returns detected address (0x68 or 0x69)
```

### Reading Motion Data
```cpp
Motion read();             // Returns Motion struct
bool read(Motion &m);      // Pass-by-reference read, returns true on success
const Motion& getLastMotion(); // Returns reference to last cached sample
float getPitch();          // Latest pitch angle
float getRoll();           // Latest roll angle
float getYaw();            // Latest yaw angle
float getTemperature();    // Die temperature in °C
```

### Configuration
```cpp
imu.setAccelRange(2);      // 2, 4, 8, 16 (in g)
imu.setGyroRange(250);     // 125, 250, 500, 1000, 2000 (in °/s)
imu.setDataRate(100);      // 25, 50, 100, 200, 400, 800, 1600 (in Hz)
imu.setDLPF(FilterBandwidth::OSR4); // OSR4, OSR2, NORMAL
imu.setDegrees(true);      // true for degrees (°), false for radians
```

### Coordinate Transforms & Offsets
```cpp
imu.setAxisRemap(1, 2, 3); // Map sensor axes (+/-1, +/-2, +/-3) to vehicle frame
imu.invertPitch(true);     // Invert pitch sign
imu.tare();                // Set current orientation as zero reference
imu.resetTare();           // Clear zero tare offsets
```

---

## Examples

Check the [examples/](examples/) directory for ready-to-flash sketches:
- **[01_BasicMotion](examples/01_BasicMotion/01_BasicMotion.ino)**: Basic motion and orientation reading.
- **[02_SensorFusion](examples/02_SensorFusion/02_SensorFusion.ino)**: Choosing and tuning between Madgwick AHRS and Complementary filters.

---

## License

MIT License. See [LICENSE](LICENSE) for details.
