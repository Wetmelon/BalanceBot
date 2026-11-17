// BalanceBot firmware: two ODrives on CAN, BNO080 on I²C. Portenta C33 or Feather M4 CAN: see board.hpp.
//
// Three tasks. The control task runs on each BNO080 gyro report (100 Hz, the IMU's clock): CAN in →
// bot::Controller::tick → CAN out, back to back so the torque goes out on fresh data. The IMU task
// polls the BNO080 at 1 kHz. Telemetry prints at 10 Hz.

// Damp first: Arduino.h defines abs, min, max, F, ... as macros, which break any header after it.
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
static volatile uint32_t         tick_cycles_max   = 0;  // the slowest Controller::tick since the last telemetry line
static volatile uint32_t         qp_iterations_max = 0;  // the most OFMPC QP iterations in a tick since then

static constexpr float    kTwoPi    = 2.0f * damp::numbers::pi_v<float>;
static constexpr uint32_t kPeriodMs = 10;
static TaskHandle_t       control_task = nullptr;  // the IMU task wakes it on each gyro report

// [words] The control task's stack. The OFMPC's QP solve takes about 13.4 KB of it (-fstack-usage:
// the MPC's control 6.3 KB, the active-set solver 6.1 KB); 4 KB overflowed it and faulted the CPU on
// the first active tick. The tasks' stacks come out of FreeRTOS's 32 KB heap on the C33.
#if BOT_PITCH_LAW == 2
static constexpr uint32_t kControlStack = 4096;
#else
static constexpr uint32_t kControlStack = 1024;
#endif

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

/// [N·m] total pitch torque added while balancing, to identify the plant from the CAN log; 0 turns it off.
/// docs/derivation.md used 0.3 and 0.8.
static constexpr float kExcitation = 0.0f;

// PRBS9 (x^9 + x^5 + 1), a new bit every two ticks: ±kExcitation, flat to ~25 Hz, repeating every 10.2 s.
static float excitation() {
    static uint16_t lfsr = 0x1FF;
    static uint32_t tick = 0;
    if (tick++ % 2 == 0) {
        lfsr = static_cast<uint16_t>(((lfsr << 1) | (((lfsr >> 8) ^ (lfsr >> 4)) & 1U)) & 0x1FFU);
    }
    return (lfsr & 1U) != 0U ? kExcitation : -kExcitation;
}

// The tick on CAN, for a logger on the bus: IDs no ODrive answers to, behind the drives in priority.
// Little-endian; the wheels are in the drives' own encoder estimates.
//   0x600  pitch [rad], pitch_rate [rad/s]                                    float32, float32
//   0x601  torque left, right [N·m] as sent, before the left motor's mirror   float32, float32
//   0x602  x [m], mode (0 idle, 1 active, 2 fault), flags (bit 0 IMU fresh,    float32, uint8, uint8,
//          bit 1 drives healthy), excitation [mN·m] (in 0x601's torques)      int16
static void logToCan(const bot::Measurement& m, const bot::Output& o, bool imu_fresh, float excite) {
    uint8_t d[8] = {};
    auto    floats = [&](uint32_t id, float a, float b) {
        std::memcpy(d, &a, 4);
        std::memcpy(d + 4, &b, 4);
        bot_can.sendLog(id, d);
    };
    floats(0x600, m.pitch, m.pitch_rate);
    floats(0x601, o.torque.left, o.torque.right);
    const float   x  = bot::Balancer::position(m);
    const int16_t mn = static_cast<int16_t>(std::lround(excite * 1000.0f));  // [mN·m]
    std::memcpy(d, &x, 4);
    d[4] = static_cast<uint8_t>(o.mode);
    d[5] = (imu_fresh ? 1 : 0) | (bot_can.healthy() ? 2 : 0);
    std::memcpy(d + 6, &mn, 2);
    bot_can.sendLog(0x602, d);
}

static void controlTask(void*) {
    bot_can.setup({.velocity = bot::kDriveVelocityLimit, .current = bot::kDriveCurrentLimit});

    damp::Periodic<uint32_t> blink{500};  // [ms] toggle: 1 Hz
    bool                     led = false;

    for (;;) {
        // One tick per gyro report, so every tick reads a sample of the same age. On the C33's own
        // 100 Hz the two clocks drift through each other (0.25 % apart: every 4 s), and near each
        // crossing ticks read a sample twice, then skip one: a tick-to-tick jitter the law turns into
        // a buzz. With no reports (the IMU not up yet, or lost) it ticks every 1.5 periods.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kPeriodMs + kPeriodMs / 2));

        bot_can.read(kPeriodMs);
        const ImuWrapper::Sample s   = imu.latest();
        const bot::Measurement   m   = measure(s);
        const uint32_t           t0  = DWT->CYCCNT;
        bot::Output              out = controller.tick(m, {.imu = s.fresh, .drives = bot_can.healthy()}, {});
        const uint32_t           dt  = DWT->CYCCNT - t0;
        tick_cycles_max              = dt > tick_cycles_max ? dt : tick_cycles_max;  // ponytail: racing telemetry's reset loses one tick at most
        const uint32_t           it  = controller.qp_iterations();
        qp_iterations_max            = it > qp_iterations_max ? it : qp_iterations_max;

        // ponytail: past the 2 N·m traction clamp by at most kExcitation / 2; the drives allow far more
        const float excite = (out.mode == bot::Mode::Active) ? excitation() : 0.0f;
        out.torque.left += 0.5f * excite;
        out.torque.right += 0.5f * excite;

        if (out.changed) {
            bot_can.setAxisStates(out.mode == bot::Mode::Active ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
        }
        bot_can.sendTorques(-out.torque.left, +out.torque.right);
        logToCan(m, out, s.fresh, excite);

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
        Serial.print("\ttick_us ");  // the slowest tick in the last 100 ms
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
    for (; !tasks; delay(1000)) {  // the FreeRTOS heap is too small for the stacks: nothing runs the drives
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

void loop() {}
