
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <array>

#include "src/ODriveEnums.h"
#include "src/balancer.hpp"
#include "src/bot_can.hpp"
#include "src/can_helpers.hpp"
#include "src/can_simple_messages.hpp"
#include "src/config.hpp"
#include "src/imu_wrapper.hpp"
#include "src/portenta_rgb.hpp"
#include "src/utils.hpp"

#ifdef ARDUINO_PORTENTA_C33
// TODO:  Make it easy to configure this
#endif

// Global object initialization
BotCanClass   bot_can;
BotController controller;
ImuWrapper    imu;
RgbC33        pixel;

// Task handles
static TaskHandle_t taskHandle_1kHz;
static TaskHandle_t taskHandle_100Hz;

void setup() {
    configControllers();

    // Initialize Serial
    Serial.begin(115200);

    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial) {
        delay(1);
    }

    // Init objects
    pixel.setup();
    imu.begin(Wire2);
    bot_can.setup();
    controller.begin();

    // Create RTOS tasks
    xTaskCreate(periodic_1kHz, "IMU Task", 256, nullptr, tskIDLE_PRIORITY + 3, &taskHandle_1kHz);
    xTaskCreate(periodic_100Hz, "CAN Task", 256, nullptr, tskIDLE_PRIORITY + 2, &taskHandle_100Hz);

    // Start RTOS tasks
    Serial.println("Starting Scheduler");
    Serial.flush();

    vTaskStartScheduler();

    for (;;) {
        Serial.println("Scheduler failed!");
        Serial.flush();
        delay(1000);
    }
}

static void periodic_100Hz(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Run this code periodically at 100Hz
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, 10UL);

        controller.step();
        bot_can.send();
    }
}

static void periodic_1kHz(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Run this code periodically at 1kHz
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, 1UL);

        imu.read();
        bot_can.read();
    }
}

// RTOS Idle Loop
void loop() {
    delay(1000);
}
