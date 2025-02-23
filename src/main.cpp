#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <ODriveEnums.h>

#include <balancer.hpp>
#include <bot_can.hpp>
#include <can_helpers.hpp>
#include <can_simple_messages.hpp>
#include <config.hpp>
#include <imu_wrapper.hpp>
#include <portenta_rgb.hpp>
#include <utils.hpp>

#ifdef ARDUINO_PORTENTA_C33
// TODO:  Make it easy to configure this
#endif

// Global object initialization
BotCanClass bot_can;
BotController controller;
ImuWrapper imu;
RgbC33 pixel;

// Task handles
static TaskHandle_t taskHandle_1kHz;
static TaskHandle_t taskHandle_100Hz;
static TaskHandle_t taskHandle_1Hz;

static void periodic_1Hz(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t count = 0;
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, 1000UL);
        digitalWrite(LEDB, count++ & 0x1);
    }
}

static void periodic_100Hz(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    // Run this code periodically at 100Hz
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, 10UL);

        controller.step();
        bot_can.send();
    }
}

static void periodic_1kHz(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    static bool flag = false;

    // Run this code periodically at 1kHz
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, 1UL);

        imu.read();
        bot_can.read();
    }
}

void setup()
{
    pinMode(PIN_D7, PinMode::OUTPUT);
    configControllers();

    // Initialize Serial
    Serial.begin(115200);

    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial)
    {
        delay(1);
    }

    // Init objects
    pixel.setup();
    imu.begin(Wire2);
    bot_can.setup();
    controller.begin();

    // Create RTOS tasks
    xTaskCreate(periodic_1kHz, "IMU Task", 1024, nullptr, tskIDLE_PRIORITY + 3, &taskHandle_1kHz);
    xTaskCreate(periodic_100Hz, "CAN Task", 1024, nullptr, tskIDLE_PRIORITY + 2, &taskHandle_100Hz);
    xTaskCreate(periodic_1Hz, "Beep task", 1024, nullptr, tskIDLE_PRIORITY + 1, &taskHandle_1Hz);

    // Start RTOS tasks
    Serial.println("Starting Scheduler");
    Serial.flush();

    vTaskStartScheduler();

    for (;;)
    {
        Serial.println("Scheduler failed!");
        Serial.flush();
        delay(1000);
    }
}

// RTOS Idle Loop
void loop()
{

    delay(1000);
}
