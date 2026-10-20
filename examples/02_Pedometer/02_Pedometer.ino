#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing BMI160...");

    if (!imu.begin(21, 22)) {
        Serial.println("Error: BMI160 initialization failed!");
        while (1) delay(500);
    }

    imu.enablePedometer(true);
    imu.resetSteps();

    Serial.println("Pedometer active. Movement will count steps.\n");
}

void loop() {
    static uint32_t last_check = 0;
    uint32_t now = millis();

    if (now - last_check >= 500) {
        last_check = now;

        uint16_t steps = imu.getSteps();
        Serial.printf("Steps: %u\n", steps);
    }
}
