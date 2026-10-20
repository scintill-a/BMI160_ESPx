# BMI160_ESPx

Industrial-grade, high-performance, and ultra-ergonomic C++17 driver for the Bosch BMI160 6-DOF IMU on Espressif ESP32.

## Why BMI160_ESPx?

* **No Convoluted Enums:** Configure ranges and rates using plain natural numbers: `setAccelRange(2)`, `setGyroRange(250)`, `setDataRate(100)`.
* **Real-Time Tilt in Degrees:** Built-in complementary 6-DOF orientation engine outputs **Pitch**, **Roll**, and **Yaw** in degrees directly (`m.pitch`, `m.roll`).
* **Rock-Solid I2C:** Safe GPIO bus clearing before `Wire.begin()` avoids ESP32 Core 3.x handle corruption (`ESP_ERR_INVALID_STATE`). Strict 12-byte burst reads prevent sensor NACKs.
* **Auto-Probing & Diagnostics:** Automatically detects `0x68` and `0x69`, with an automated I2C bus scanner fallback if wiring is disconnected.
* **Dual-Core FreeRTOS Streamer:** Asynchronous sampling worker pinned to Core 1 pushes directly to FreeRTOS queues.
* **Hardware Pedometer:** Built-in step counter runs on-chip with zero CPU load.

## Quick Start (3 Lines)

```cpp
#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);

    // 1. Connect (SDA=21, SCL=22) - auto-detects address, clears bus, applies defaults
    imu.begin(21, 22);

    // 2. Optional: calibrate stationary gyro bias
    imu.calibrate();
}

void loop() {
    // 3. Read motion + orientation in degrees
    Motion m = imu.read();

    Serial.printf("Pitch: %5.1f deg | Roll: %5.1f deg | Accel: (%+.2f, %+.2f, %+.2f) g\n",
                  m.pitch, m.roll, m.ax, m.ay, m.az);

    delay(50);
}
```

## Customizable & Manipulatable API

```cpp
// Plain natural number configuration (all have zero-arg defaults!)
imu.setAccelRange();      // Defaults to 2 (±2g). Options: 2, 4, 8, 16
imu.setGyroRange();       // Defaults to 250 (±250 deg/s). Options: 125, 250, 500, 1000, 2000
imu.setDataRate();        // Defaults to 100 (100 Hz). Options: 25, 50, 100, 200, 400, etc.
imu.setDegrees(true);     // Toggle degrees vs radians (default: true)
imu.setFilterWeight();    // Complementary filter weight (default: 0.98f)
imu.setMadgwickBeta();    // Madgwick filter divergence gain (default: 0.10f)

// Quick getters
float pitch = imu.getPitch();
float roll  = imu.getRoll();
float temp  = imu.getTemperature();

// Hardware pedometer
imu.enablePedometer(true);
uint16_t steps = imu.getSteps();
imu.resetSteps();

// Low-level register access
imu.writeRegister(reg, val);
imu.readRegister(reg, val);
imu.readBurst(reg, buffer, len);
```
