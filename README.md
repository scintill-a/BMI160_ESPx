# BMI160_ESPx

An ESP32 library for the Bosch BMI160 6-axis IMU (accelerometer and gyroscope) with built-in sensor fusion (pitch, roll, yaw), step counting, and I2C bus recovery.

---

## Features

- **6-DOF Motion:** Reads linear acceleration (`ax`, `ay`, `az`), angular velocity (`gx`, `gy`, `gz`), and temperature.
- **Sensor Fusion:** Built-in **Madgwick AHRS** (quaternion 3D attitude) and **Complementary** tilt filters.
- **I2C Bus Recovery:** Clears hung I2C bus lines automatically if a communication lockup occurs.
- **Auto-Address Detection:** Automatically checks `0x68` and `0x69`.
- **Configurable Ranges:** Simple setters for acceleration (±2g, ±4g, ±8g, ±16g), gyro rate (±125 to ±2000 °/s), and sample rates (25 Hz to 1600 Hz).
- **Hardware Pedometer:** Built-in on-chip step counter that runs without using ESP32 CPU cycles.
- **FreeRTOS Streaming:** Optional background task to stream sensor readings to a FreeRTOS queue on Core 1.

---

## Installation

### Option 1: Arduino IDE

#### Method A: Add .ZIP Library (Easiest)
1. Download this repository as a `.zip` file:
   - Click the green **Code** button at the top of this GitHub page and select **Download ZIP** (or [click here to download](https://github.com/scintill-a/BMI160_ESPx/archive/refs/heads/main.zip)).
2. In Arduino IDE, go to **Sketch** &rarr; **Include Library** &rarr; **Add .ZIP Library...**.
3. Select the downloaded `.zip` file.
4. Open an example via **File** &rarr; **Examples** &rarr; **BMI160_ESPx** &rarr; **01_BasicMotion**.

#### Method B: Git Clone
Clone directly into your Arduino `libraries` folder:

- **Windows (PowerShell):**
  ```powershell
  cd "$env:USERPROFILE\Documents\Arduino\libraries"
  git clone https://github.com/scintill-a/BMI160_ESPx.git
  ```
- **macOS:**
  ```bash
  cd ~/Documents/Arduino/libraries
  git clone https://github.com/scintill-a/BMI160_ESPx.git
  ```
- **Linux:**
  ```bash
  cd ~/Arduino/libraries
  git clone https://github.com/scintill-a/BMI160_ESPx.git
  ```

> [!TIP]
> If extracting the ZIP manually, make sure the folder inside `libraries/` is named `BMI160_ESPx` (remove any `-main` suffix added by GitHub) and restart Arduino IDE.

---

### Option 2: PlatformIO

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
| **SA0 / SDO** | *(Leave floating)* | Optional. Can be left floating (defaults to `0x68`). Connect to 3.3V for `0x69`. |

> [!NOTE]
> - **SA0 / SDO Pin:** You can just leave this pin disconnected/floating. Most modules pull it to GND automatically, which sets the I2C address to `0x68`.
> - **Custom Pins:** To use different pins, pass them to `imu.begin(sda, scl)`, for example: `imu.begin(18, 19)`.

---

## Quick Start

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("BMI160 not found, check wiring.");
        while (1) delay(100);
    }

    imu.calibrate();
}

void loop() {
    Motion m = imu.read();
    Serial.printf("Pitch: %.1f | Roll: %.1f | Yaw: %.1f\n", m.pitch, m.roll, m.yaw);
    delay(50);
}
```

Upload the sketch and open the Serial Monitor at **115200** baud to view live pitch, roll, and yaw values.

---

## Sensor Fusion & Tuning

The library includes two built-in orientation algorithms:

### 1. Madgwick AHRS (Default)
A 6-DOF quaternion filter for complete 3D orientation without gimbal lock:
```cpp
imu.setFusionAlgorithm(FusionMode::MADGWICK); // Default
imu.setMadgwickBeta(0.1f);                    // Divergence gain beta (default: 0.1f)
```
Outputs quaternions (`m.qw`, `m.qx`, `m.qy`, `m.qz`) and Euler angles (`m.pitch`, `m.roll`, `m.yaw`).

### 2. Complementary Filter
A lightweight tilt filter combining gyro integration with accelerometer gravity tilt:
```text
angle = alpha × (angle + gyro × dt) + (1 - alpha) × accel_angle
```
```cpp
imu.setFusionAlgorithm(FusionMode::COMPLEMENTARY);
imu.setFilterWeight(0.98f);                   // Weight alpha in [0.0, 1.0] (default: 0.98f)
```

---

## API Cheatsheet

### Initialization & Calibration
```cpp
bool begin(int sda = 21, int scl = 22, uint32_t frequency = 400000, uint8_t address = 0);
void calibrate(uint16_t samples = 200, bool run_hardware_foc = true);
uint8_t getAddress(); // Returns detected address (0x68 or 0x69)
```

### Reading Motion Data
```cpp
Motion read();             // Returns Motion struct (pitch, roll, yaw, ax, ay, az, gx, gy, gz, qw, qx, qy, qz)
bool read(Motion &m);      // Pass-by-reference read, returns true on success
float getPitch();          // Pitch angle
float getRoll();           // Roll angle
float getYaw();            // Yaw angle
float getTemperature();    // Die temperature in °C
```

### Sensor Configuration
```cpp
imu.setAccelRange(2);      // 2, 4, 8, 16 (in g)
imu.setGyroRange(250);     // 125, 250, 500, 1000, 2000 (in °/s)
imu.setDataRate(100);      // 25, 50, 100, 200, 400, 800, 1600 (in Hz)
imu.setDegrees(true);      // true for degrees (°), false for radians
```

### Hardware Pedometer
```cpp
imu.enablePedometer(true); // Enable step detector
uint16_t steps = imu.getSteps(); // Read step count
imu.resetSteps();          // Reset step counter to 0
```

### FreeRTOS Dual-Core Streaming
```cpp
imu.startAsyncStream(100, motion_queue, 1); // 100 Hz on Core 1
imu.stopAsyncStream();
```

---

## Examples

Check the [examples/](examples/) directory for ready-to-flash sketches:
- **[01_BasicMotion](examples/01_BasicMotion/01_BasicMotion.ino)**: Basic motion and orientation reading.
- **[02_Pedometer](examples/02_Pedometer/02_Pedometer.ino)**: On-chip step counter.
- **[03_FreeRTOS_Worker](examples/03_FreeRTOS_Worker/03_FreeRTOS_Worker.ino)**: Background streaming task on Core 1 using a FreeRTOS queue.
- **[04_SensorFusion](examples/04_SensorFusion/04_SensorFusion.ino)**: Choosing and tuning between Madgwick AHRS and Complementary filters.

---

## License

MIT License. See [LICENSE](LICENSE) for details.
