#include <Arduino.h>
#include <BMI160_ESPx.h>

BMI160_ESPx imu;
QueueHandle_t motion_queue = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Initializing BMI160 FreeRTOS worker...");

    if (!imu.begin(21, 22)) {
        Serial.println("Error: BMI160 failed to initialize!");
        while (1) delay(500);
    }

    imu.setAccelRange(2);
    imu.setGyroRange(250);

    motion_queue = xQueueCreate(20, sizeof(Motion));
    if (motion_queue == nullptr) {
        Serial.println("Error: Queue allocation failed!");
        while (1) delay(500);
    }

    // Start background sampling task on Core 1 at 100 Hz
    if (imu.startAsyncStream(100, motion_queue, 1)) {
        Serial.println("Async worker streaming on Core 1 @ 100 Hz.\n");
    } else {
        Serial.println("Error: Failed to launch async worker!");
        while (1) delay(500);
    }
}

void loop() {
    Motion m;

    if (xQueueReceive(motion_queue, &m, pdMS_TO_TICKS(50)) == pdTRUE) {
        static uint32_t last_print = 0;
        uint32_t now = millis();

        if (now - last_print >= 100) {
            last_print = now;
            Serial.printf("[Core %d] PITCH: %+5.1f° | ROLL: %+5.1f° | YAW: %+5.1f° | GYR: (%+5.1f, %+5.1f, %+5.1f) dps | ACC: (%+4.2f, %+4.2f, %+4.2f) g\n",
                          xPortGetCoreID(),
                          m.pitch, m.roll, m.yaw,
                          m.gx, m.gy, m.gz,
                          m.ax, m.ay, m.az);
        }
    }
}
