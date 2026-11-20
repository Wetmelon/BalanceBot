// BalanceBot firmware: two ODrives on CAN, BNO080 on I²C, on a Portenta C33 or Feather M4 CAN (board.hpp).
// Control task: CAN in → Controller::tick → CAN out, once per gyro report (100 Hz). IMU task: polls the
// BNO080 at 1 kHz. Telemetry task: 10 Hz.

// Damp first: Arduino.h's abs, min, max macros break any header after it.
// clang-format off
#include "balance_controller.hpp"
#include "damp/toolbox/timing.hpp"

#include "board.hpp"
#include "bot_can.hpp"
#include "imu.hpp"
// clang-format on

BotCanClass bot_can;
ImuWrapper  imu;

static constinit bot::Controller controller;
static bot::Measurement          last_measurement;  // telemetry copies; a torn read only misprints
static bot::Output               last_output;
static volatile uint32_t         tick_cycles_max   = 0;  // since the last telemetry line
static volatile uint32_t         qp_iterations_max = 0;
static TaskHandle_t              control_task      = nullptr;

static constexpr float    kTwoPi        = 2.0f * damp::numbers::pi_v<float>;
static constexpr uint32_t kPeriodMs     = 10;
static constexpr uint32_t kControlStack = 4096;  // [words] the OFMPC's QP takes ~13.4 KB

// ODrive reports the wheel against the body, in turns. The left motor is mirrored.
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

// The tick on CAN for a logger; layout in the README, "CAN log".
static void logToCan(const bot::Measurement& m, const bot::Output& o, bool imu_fresh) {
    uint8_t d[8]   = {};
    auto    floats = [&](uint32_t id, float a, float b) {
        std::memcpy(d, &a, 4);
        std::memcpy(d + 4, &b, 4);
        bot_can.sendLog(id, d);
    };
    floats(0x600, m.pitch, m.pitch_rate);
    floats(0x601, o.torque.left, o.torque.right);
    const float x = bot::Balancer::position(m);
    std::memcpy(d, &x, 4);
    d[4] = static_cast<uint8_t>(o.mode);
    d[5] = (imu_fresh ? 1 : 0) | (bot_can.healthy() ? 2 : 0);
    d[6] = d[7] = 0;
    bot_can.sendLog(0x602, d);
}

static void controlTask(void*) {
    bot_can.setup({.velocity = bot::kDriveVelocityLimit, .current = bot::kDriveCurrentLimit});

    damp::Periodic<uint32_t> blink{500};  // [ms]
    bool                     led = false;

    for (;;) {
        // On the IMU's clock, not the board's: the two drift through each other every 4 s, and a tick
        // that reads one sample twice and then skips one buzzes. Without the IMU, every 1.5 periods.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kPeriodMs + kPeriodMs / 2));

        bot_can.read(kPeriodMs);
        const ImuWrapper::Sample s   = imu.latest();
        const bot::Measurement   m   = measure(s);
        const uint32_t           t0  = DWT->CYCCNT;
        const bot::Output        out = controller.tick(m, {.imu = s.fresh, .drives = bot_can.healthy()}, {});
        const uint32_t           dt  = DWT->CYCCNT - t0;
        const uint32_t           it  = controller.qp_iterations();
        // ponytail: racing telemetry's reset loses one tick at most
        tick_cycles_max   = dt > tick_cycles_max ? dt : tick_cycles_max;
        qp_iterations_max = it > qp_iterations_max ? it : qp_iterations_max;

        if (out.changed) {
            bot_can.setAxisStates(out.mode == bot::Mode::Active ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
        }
        bot_can.sendTorques(-out.torque.left, +out.torque.right);
        logToCan(m, out, s.fresh);

        digitalWrite(board::kFaultPin, bot_can.axisErrors() != 0 ? HIGH : LOW);
        if (blink(kPeriodMs)) {
            led = !led;
            board::blink(led);
        }
        last_measurement = m;
        last_output      = out;
    }
}

static void imuTask(void*) {
    imu.begin(board::imu_wire);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
        if (imu.read()) {
            xTaskNotifyGive(control_task);
        }
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
        Serial.print(o.torque.left, 2);
        Serial.print("\todrive ");
        Serial.print(bot_can.healthy() ? "ok" : "missing or faulted");
        Serial.print("\ttick_us ");
        Serial.print(static_cast<float>(tick_cycles_max) / (static_cast<float>(SystemCoreClock) / 1e6f), 1);
        tick_cycles_max = 0;
        Serial.print("\tqp_it ");
        Serial.println(qp_iterations_max);
        qp_iterations_max = 0;
    }
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial) {
        delay(1);
    }
    pinMode(board::kFaultPin, OUTPUT);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // the cycle counter, for tick_us
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    board::setupLeds();

    const bool tasks = xTaskCreate(controlTask, "control", kControlStack, nullptr, tskIDLE_PRIORITY + 3, &control_task) == pdPASS &&
                       xTaskCreate(imuTask, "imu", 1024, nullptr, tskIDLE_PRIORITY + 2, nullptr) == pdPASS &&
                       xTaskCreate(telemetryTask, "telemetry", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr) == pdPASS;
    for (; !tasks; delay(1000)) {
        Serial.println("No room for the tasks' stacks!");
    }

    Serial.println("Starting Scheduler");
    Serial.flush();
    vTaskStartScheduler();

    for (;;) {
        Serial.println("Scheduler failed!");
        Serial.flush();
        delay(1000);
    }
}

// On FreeRTOS this is never reached
void loop() {}
