
#include <FreeRTOS_SAMD21.h>
#include <ServoInput.h>

#include <array>

#include "src/MkrRgb.hpp"
#include "src/ODriveEnums.h"
#include "src/WiFI_server.hpp"
#include "src/balancer.hpp"
#include "src/bot_can.hpp"
#include "src/can_helpers.hpp"
#include "src/can_simple_messages.hpp"
#include "src/config.hpp"
#include "src/imu_wrapper.hpp"
#include "src/utils.hpp"

ServoInputPin<0> rc_pwm_in_0;
ServoInputPin<1> rc_pwm_in_1;

// Global object initialization
BotCanClass   bot_can;
BotController controller;
ImuWrapper    imu;
MKRrgb        pixel;

// Task handles
static TaskHandle_t imu_task;
static TaskHandle_t can_task;
static TaskHandle_t control_task;
static TaskHandle_t wifi_task;

static WiFiServer server{80};

void setup() {
    configControllers();

    // Initialize Serial
    Serial.begin(250000);

    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial) {
        delay(1);
    }

    rc_pwm_in_0.attach();
    rc_pwm_in_1.attach();

    // Initialize MKR RGB LED
    pixel.setup();

    // Create RTOS tasks
    xTaskCreate(controlTask, "control Task", 256, nullptr, tskIDLE_PRIORITY + 4, &control_task);
    xTaskCreate(canTask, "CAN Task", 256, nullptr, tskIDLE_PRIORITY + 3, &can_task);
    xTaskCreate(imuTask, "IMU Task", 256, nullptr, tskIDLE_PRIORITY + 2, &imu_task);
    //xTaskCreate(wifiTask, "wifi Task", 256, nullptr, tskIDLE_PRIORITY + 1, &wifi_task);

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

// RTOS Idle Loop
void loop() {
    delay(1000);
}

static void controlTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Initialize controller
    controller.begin();

    // Run this code periodically at 100Hz
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, 12UL);

        float in0 = rc_pwm_in_0.getPercent();
        float in1 = rc_pwm_in_1.getPercent();
        if (in0 < 0.01f || in0 > 0.99f) in0 = 0.5f; // fix noise glitches TODO: use better hardware
        if (in1 < 0.01f || in1 > 0.99f) in1 = 0.5f; // fix noise glitches TODO: use better hardware

        controller.step(
            2.0f*(+(in1 - 0.5f)),
            2.0f*(-(in0 - 0.5f)));
    }
}

static void canTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Initialize CAN bus
    bot_can.setup();

    // Run this code periodically at 100Hz
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, 12UL);

        bot_can.read();
        bot_can.send();
    }
}

static void imuTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Start the IMU on i2c, at 400kHz
    imu.begin();

    // Run this code periodically at 1kHz
    for (;;) {
        vTaskDelayUntil(&lastWakeTime, 1UL);

        imu.read();
    }
}

static void wifiTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();

    Wifisetup(server);

    // Run this code periodically
    for (;;) {
        Wifiloop(server);
    }
}
