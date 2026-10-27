#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing BMI160...");

    if (!imu.begin()) {
        Serial.println("BMI160 not found, check wiring.");
        while (1) delay(500);
    }

    // Select fusion algorithm:
    // 1. Madgwick AHRS (default): 6-DOF quaternion filter for full 3D attitude
    imu.setFusionAlgorithm(FusionMode::MADGWICK);
    imu.setMadgwickBeta(0.1f);

    // 2. Alternatively, use the Complementary filter for simple 2-axis tilt:
    // imu.setFusionAlgorithm(FusionMode::COMPLEMENTARY);
    // imu.setFilterWeight(0.98f);

    Serial.println("Calibrating (keep sensor still)...");
    imu.calibrate();
    Serial.println("Ready.\n");
}

void loop() {
    static uint32_t last_print = 0;
    uint32_t now = millis();

    Motion m = imu.read();

    if (now - last_print >= 50) { // 20 Hz
        last_print = now;
        Serial.printf("Pitch: %6.1f | Roll: %6.1f | Yaw: %6.1f | Quat: [%.2f, %.2f, %.2f, %.2f]\n",
                      m.pitch, m.roll, m.yaw, m.qw, m.qx, m.qy, m.qz);
    }
}
