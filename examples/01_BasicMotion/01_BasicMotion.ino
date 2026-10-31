#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing BMI160...");

    // Connect with standard ESP32 pins (SDA=21, SCL=22)
    if (!imu.begin(21, 22)) {
        Serial.println("Error: BMI160 not responding!");
        while (1) delay(500);
    }

    Serial.printf("BMI160 ready on 0x%02X\n", imu.getAddress());

    // Configure with plain numbers:
    imu.setAccelRange(2);   // ±2g
    imu.setGyroRange(250);  // ±250 deg/s
    imu.setDataRate(100);   // 100 Hz

    // Quick calibration
    Serial.println("Calibrating (keep still)...");
    imu.calibrate(200);
    Serial.println("Ready.\n");
}

void loop() {
    static uint32_t last_sample = 0;
    static uint32_t last_print = 0;
    uint32_t now = millis();

    // 100 Hz IMU sampling and attitude update
    if (now - last_sample >= 10) {
        last_sample = now;
        imu.read();
    }

    if (now - last_print >= 50) { // 20 Hz serial output
        last_print = now;

        const Motion &m = imu.getLastMotion();
        Serial.printf("PITCH: %+5.1f° | ROLL: %+5.1f° | YAW: %+5.1f° | GYR: (%+5.1f, %+5.1f, %+5.1f) dps | ACC: (%+4.2f, %+4.2f, %+4.2f) g\n",
                      m.pitch, m.roll, m.yaw,
                      m.gx, m.gy, m.gz,
                      m.ax, m.ay, m.az);
    }
}
