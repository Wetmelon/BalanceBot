// BalanceBot firmware: Arduino MKR WiFi 1010, MKR CAN shield, two ODrives, BNO080.
//
// Three tasks. The control task runs at 100 Hz: CAN in → bot::Controller::tick → CAN out, back to
// back so the torque goes out on fresh data. The IMU task polls the BNO080 at 1 kHz. Telemetry
// prints at 10 Hz.

// Damp first: Arduino.h defines abs, min, max, F, ... as macros, which break any header after it.
// clang-format off
#include "balance_controller.hpp"
#include "damp/toolbox/timing.hpp"

#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "MkrRgb.hpp"
#include "bot_can.hpp"
#include "imu.hpp"
// clang-format on

BotCanClass bot_can;
ImuWrapper  imu;
MKRrgb      pixel;

static constinit bot::Controller controller;
static bot::Measurement          last_measurement;  // telemetry copies; a torn read only misprints
static bot::Output               last_output;

static constexpr float kTwoPi = 2.0f * damp::numbers::pi_v<float>;

// ODrive reports the rotor against the stator, in turns: the wheel against the body. The left
// motor is mirrored, so forward is negative turns on it.
static bot::Measurement measure(const ImuWrapper::Sample& s) {
    const auto& r = bot_can.right_motor.get_encoder_estimates_msg;
    const auto& l = bot_can.left_motor.get_encoder_estimates_msg;
    return {
        .pitch        = s.pitch,
        .pitch_rate   = s.pitch_rate,
        .wheel_r      = +kTwoPi * r.Pos_Estimate,
        .wheel_l      = -kTwoPi * l.Pos_Estimate,
        .wheel_rate_r = +kTwoPi * r.Vel_Estimate,
        .wheel_rate_l = -kTwoPi * l.Vel_Estimate,
    };
}

static void controlTask(void*) {
    bot_can.setup();
    bot_can.setAxisStates(AXIS_STATE_IDLE);

    damp::Periodic<uint32_t> blink{500};  // [ticks] toggle: 1 Hz
    bool                     led = false;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));

        bot_can.read();
        const ImuWrapper::Sample s   = imu.latest();
        const bot::Measurement   m   = measure(s);
        const bot::Output        out = controller.tick(m, {.imu = s.fresh, .drives = bot_can.healthy()}, {});

        if (out.changed) {
            bot_can.setAxisStates(out.mode == bot::Mode::Active ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
        }
        bot_can.sendTorques(-out.torque.left, +out.torque.right);

        if (blink(10U)) {
            led = !led;
            digitalWrite(LED_BUILTIN, led ? HIGH : LOW);
        }
        last_measurement = m;
        last_output      = out;
    }
}

static void imuTask(void*) {
    imu.begin();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
        imu.read();
    }
}

static void telemetryTask(void*) {
    static constexpr const char* kModeName[] = {"idle", "active", "fault"};
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        const bot::Measurement m = last_measurement;
        const bot::Output      o = last_output;
        Serial.print(kModeName[static_cast<int>(o.mode)]);
        Serial.print("\tpitch ");
        Serial.print(m.pitch, 4);
        Serial.print("\tx ");
        Serial.print(bot::Balancer::position(m), 3);
        Serial.print("\ttau ");
        Serial.print(o.torque.right, 2);
        Serial.print(' ');
        Serial.println(o.torque.left, 2);
    }
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial) {
        delay(1);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    pixel.setup();

    xTaskCreate(controlTask, "control", 256, nullptr, tskIDLE_PRIORITY + 3, nullptr);
    xTaskCreate(imuTask, "imu", 256, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(telemetryTask, "telemetry", 256, nullptr, tskIDLE_PRIORITY + 1, nullptr);

    Serial.println("Starting Scheduler");
    Serial.flush();
    vTaskStartScheduler();

    for (;;) {
        Serial.println("Scheduler failed!");
        Serial.flush();
        delay(1000);
    }
}

void loop() {}
