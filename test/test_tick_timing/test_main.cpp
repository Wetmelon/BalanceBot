// What one control period costs on the board: bot::Controller::tick (the Damp LQIs, filters and
// mode logic) and the IMU's quaternion-to-pitch, timed with the DWT cycle counter. Board on USB only.
//   pio test -e portenta_c33 -f test_tick_timing
//   pio test -e feather_m4_can -f test_tick_timing
// clang-format off
#include "balance_controller.hpp"
#include <Arduino.h>
#include <unity.h>
// clang-format on

static bot::Controller controller;
static volatile float  sink;  // keeps the timed results alive

struct Stats {
    uint32_t min = UINT32_MAX, max = 0;
    uint64_t sum = 0;
    uint32_t n   = 0;
    void     add(uint32_t c) {
        min = c < min ? c : min;
        max = c > max ? c : max;
        sum += c;
        ++n;
    }
};

static void report(const char* what, const Stats& s) {
    const float mhz = static_cast<float>(SystemCoreClock) / 1e6f;
    char        line[160];
    snprintf(line, sizeof line, "%s: cycles min %lu mean %lu max %lu = %.2f / %.2f / %.2f us at %.0f MHz (%.3f %% of the 10 ms period)", what,
             static_cast<unsigned long>(s.min), static_cast<unsigned long>(s.sum / s.n), static_cast<unsigned long>(s.max),
             static_cast<double>(static_cast<float>(s.min) / mhz), static_cast<double>(static_cast<float>(s.sum / s.n) / mhz),
             static_cast<double>(static_cast<float>(s.max) / mhz), static_cast<double>(mhz),
             static_cast<double>(static_cast<float>(s.max) / mhz / 100.0f));
    TEST_MESSAGE(line);
}

// A small wobble, so no tick sees the same input twice: lean within ±0.01 rad, wheels creeping.
static bot::Measurement wobble(uint32_t i) {
    const float u = static_cast<float>(static_cast<int32_t>((i * 2654435761U) >> 16) - 32768) / 32768.0f;  // [-1, 1)
    return {.pitch = 0.01f * u, .pitch_rate = 0.1f * u, .wheel_r = 1e-4f * static_cast<float>(i), .wheel_l = 1e-4f * static_cast<float>(i),
            .wheel_rate_r = 0.05f * u, .wheel_rate_l = -0.05f * u};
}

static void test_tick() {
    // Hold it upright until it engages (kEngageTime of ticks), then time the balancing ticks.
    uint32_t i = 0;
    while (controller.tick(wobble(i), {.imu = true, .drives = true}, {}).mode != bot::Mode::Active) {
        TEST_ASSERT_LESS_THAN_UINT32(1000, ++i);
    }
    Stats s;
    for (uint32_t k = 0; k < 10000; ++k, ++i) {
        const bot::Measurement m  = wobble(i);
        const uint32_t         t0 = DWT->CYCCNT;
        const bot::Output      o  = controller.tick(m, {.imu = true, .drives = true}, {.speed = 0.0f, .yaw_rate = 0.0f});
        s.add(DWT->CYCCNT - t0);
        sink = o.torque.left + o.torque.right;
        TEST_ASSERT_TRUE(o.mode == bot::Mode::Active);
    }
    report("Controller::tick, balancing", s);
    TEST_ASSERT_LESS_THAN_UINT32(SystemCoreClock / 1000, s.max);  // under 1 ms: a tenth of the period
}

static void test_pitch_from_imu() {
    Stats s;
    for (uint32_t k = 0; k < 10000; ++k) {
        const float                   u  = wobble(k).pitch;  // every component varies, as from the BNO080
        const float                   a  = 1.5708f + u;
        const damp::Quaternion<float> q{cosf(0.5f * a), sinf(0.5f * a), 0.3f * u, -0.2f * u};
        const uint32_t                t0 = DWT->CYCCNT;
        const float                   p  = bot::pitch_from_imu(q);
        s.add(DWT->CYCCNT - t0);
        sink = p;
    }
    report("pitch_from_imu", s);
}

void setup() {
    delay(2000);  // let the USB serial come up
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    UNITY_BEGIN();
    RUN_TEST(test_tick);
    RUN_TEST(test_pitch_from_imu);
    UNITY_END();
}

void loop() {}
