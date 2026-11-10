// BalanceBot firmware: two ODrives on CAN, BNO080 on I²C. Portenta C33 or Feather M4 CAN: see board.hpp.
//
// Three tasks. The control task runs at 100 Hz: CAN in → bot::Controller::tick → CAN out, back to
// back so the torque goes out on fresh data. The IMU task polls the BNO080 at 1 kHz. Telemetry
// prints at 10 Hz.

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

static constexpr float    kTwoPi    = 2.0f * damp::numbers::pi_v<float>;
static constexpr uint32_t kPeriodMs = 10;

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
    bot_can.setup();

    damp::Periodic<uint32_t> blink{500};  // [ms] toggle: 1 Hz
    bool                     led = false;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kPeriodMs));

        bot_can.read(kPeriodMs);
        const ImuWrapper::Sample s   = imu.latest();
        const bot::Measurement   m   = measure(s);
        bot::Output              out = controller.tick(m, {.imu = s.fresh, .drives = bot_can.healthy()}, {});

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
        Serial.print(o.torque.left, 2);
        Serial.print("\todrive ");
        Serial.println(bot_can.healthy() ? "ok" : "missing or faulted");
    }
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while ((millis() - start < 1000) && !Serial) {
        delay(1);
    }
    pinMode(board::kFaultPin, OUTPUT);
    board::setupLeds();

    xTaskCreate(controlTask, "control", 1024, nullptr, tskIDLE_PRIORITY + 3, nullptr);
    xTaskCreate(imuTask, "imu", 1024, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(telemetryTask, "telemetry", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr);

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
