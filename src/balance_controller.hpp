/**
 * @file balance_controller.hpp
 * @brief BalanceBot deploy header: nameplate, plant, design and the control tick.
 *
 * No Arduino here. main.cpp flashes it; sil/balance_sil.cpp closes the loop on the host around
 * the same objects. The plant is derived in docs/derivation.md.
 *
 * Two decoupled axes, each an LQI designed at compile time on the continuous plant sampled at Ts:
 *   pitch  x = [p, v, θ, ω], u = τ (sum of the wheel torques), integral on p
 *   yaw    x = [ψ, ψ̇],       u = Δτ (right minus left),      integral on ψ
 * The integrals hold the spot and the heading against a CoM offset, an IMU mount error or a slope.
 */

#pragma once

#include "damp/controllers/lqi.hpp"
#include "damp/filters/lowpass.hpp"
#include "damp/math/geometry.hpp"
#include "damp/math/math.hpp"
#include "damp/matrix/concat.hpp"
#include "damp/matrix/matrix.hpp"
#include "damp/systems/state_space.hpp"
#include "damp/toolbox/iec61131.hpp"

namespace bot {

using damp::ColVec;

/// Physical parameters. "measure" marks an estimate: weigh and swing the real bot, then rerun the SIL.
struct Nameplate {
    double wheel_radius  = 0.0762;  ///< [m] 6 in wheel
    double track_width   = 0.45;    ///< [m] between the tire contact patches (measure)
    double com_height    = 0.18;    ///< [m] body CoM above the floor
    double body_mass     = 4.0;     ///< [kg] everything that pitches: frame, battery, electronics (measure)
    double body_inertia  = 0.03;    ///< [kg·m²] body about its own CoM, pitch axis (measure)
    double wheel_mass    = 2.5;     ///< [kg] per side, what rides on the axle: hub motor and tire (measure)
    double wheel_inertia = 0.006;   ///< [kg·m²] per side, the rotating part about the axle (measure)
    double yaw_inertia   = 0.30;    ///< [kg·m²] whole bot about the vertical, wheels not spinning (measure)
    double gravity       = 9.81;    ///< [m/s²]
};

inline constexpr Nameplate kBot{};
inline constexpr double    kTs = 0.01;  ///< [s] 100 Hz control period

inline constexpr float kWheelTorqueMax = 5.0f;    ///< [N·m] per wheel
inline constexpr float kEngageTilt     = 0.087f;  ///< [rad] 5°: held this upright ...
inline constexpr float kEngageTime     = 2.0f;    ///< [s] ... this long, the bot engages
inline constexpr float kTipTilt        = 0.52f;   ///< [rad] 30°: past this it has fallen, motors off
inline constexpr float kLeash          = 0.3f;    ///< [m] position error the reference may lag a push by
inline constexpr float kYawLeash       = 0.5f;    ///< [rad] heading error the reference may lag by
inline constexpr float kCommandCutoff  = 0.5f;    ///< [Hz] low-pass on the speed and yaw-rate commands

/// [rad] IMU pitch reading with the CoM over the axle. Calibrate: balance it by hand, read the telemetry.
inline constexpr float kPitchTrim = 0.0f;

/// Coefficients of the pitch-plane mass matrix [a, b cosθ; b cosθ, c] (docs/derivation.md).
struct PitchMass {
    double a = 0.0;  ///< [kg] translating mass, wheel inertia reflected through r
    double b = 0.0;  ///< [kg·m] body mass × CoM height above the axle
    double c = 0.0;  ///< [kg·m²] body pitch inertia about the axle
};

[[nodiscard]] constexpr PitchMass pitch_mass(const Nameplate& n) {
    const double r = n.wheel_radius;
    const double l = n.com_height - r;
    return {
        .a = n.body_mass + 2.0 * (n.wheel_mass + n.wheel_inertia / (r * r)),
        .b = n.body_mass * l,
        .c = n.body_inertia + n.body_mass * l * l,
    };
}

/// [kg·m²] yaw inertia the differential torque sees: the body plus the wheels spinning up.
[[nodiscard]] constexpr double yaw_inertia(const Nameplate& n) {
    const double k = n.track_width / n.wheel_radius;
    return n.yaw_inertia + n.wheel_inertia * k * k / 2.0;
}

/// Pitch plane linearized upright: x = [p, v, θ, ω], u = τ, y = p.
[[nodiscard]] constexpr auto pitch_plant(const Nameplate& n) {
    const auto [a, b, c] = pitch_mass(n);
    const double r       = n.wheel_radius;
    const double g       = n.gravity;
    const double D       = a * c - b * b;
    return damp::StateSpace<4, 1, 1>{
        .A = {{0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, -b * b * g / D, 0.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, a * b * g / D, 0.0}},
        .B = {{0.0}, {(c / r + b) / D}, {0.0}, {-(a + b / r) / D}},
        .C = {{1.0, 0.0, 0.0, 0.0}},
    };
}

/// Yaw: x = [ψ, ψ̇], u = Δτ = τ_R − τ_L, y = ψ.
[[nodiscard]] constexpr auto yaw_plant(const Nameplate& n) {
    return damp::StateSpace<2, 1, 1>{
        .A = {{0.0, 1.0}, {0.0, 0.0}},
        .B = {{0.0}, {n.track_width / (2.0 * n.wheel_radius * yaw_inertia(n))}},
        .C = {{1.0, 0.0}},
    };
}

// Bryson weights, 1 / (acceptable excursion)², on the continuous cost; Van Loan samples it at Ts.
// Pitch closed loop: a 21 rad/s pair (the lean, against the 8.4 rad/s fall), a 2.5 rad/s pair
// (position, under the 6.2 rad/s non-minimum-phase zero of torque to position), 0.7 rad/s (the
// integral). Faster lean poles lose the margin the SIL asks for: ±20 % on the nameplate with every
// reading a period late. A failed design does not compile.
inline constexpr damp::design::LQRWeights<4, 1, 1> kPitchWeights{
    .Q          = damp::diag(400.0, 4.0, 100.0, 0.0),  // p 5 cm, v 0.5 m/s, θ 6°
    .R          = 0.4,                                 // τ 1.6 N·m
    .Q_integral = 200.0,                               // ∫p 0.07 m·s
};
inline constexpr damp::design::LQRWeights<2, 1, 1> kYawWeights{
    .Q          = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R          = 0.0625,                 // Δτ 4 N·m
    .Q_integral = 25.0,                   // ∫ψ 0.2 rad·s
};

inline constexpr auto kPitchDesign = damp::design::lqi(pitch_plant(kBot), kTs, kPitchWeights);
inline constexpr auto kYawDesign   = damp::design::lqi(yaw_plant(kBot), kTs, kYawWeights);

/// Sensor readings, bot frame, forward positive.
struct Measurement {
    float pitch        = 0.0f;  ///< [rad] lean, forward positive
    float pitch_rate   = 0.0f;  ///< [rad/s]
    float wheel_r      = 0.0f;  ///< [rad] right wheel against the body (the motor encoder)
    float wheel_l      = 0.0f;  ///< [rad] left wheel against the body
    float wheel_rate_r = 0.0f;  ///< [rad/s]
    float wheel_rate_l = 0.0f;  ///< [rad/s]
};

struct Command {
    float speed    = 0.0f;  ///< [m/s] forward
    float yaw_rate = 0.0f;  ///< [rad/s] counter-clockwise from above
};

struct WheelTorques {
    float right = 0.0f;  ///< [N·m] forward positive
    float left  = 0.0f;  ///< [N·m] forward positive
};

/// Lean from the IMU's game rotation vector. The IMU is mounted on its side: its x axis is the pitch
/// axis and it reads a roll of +90° upright. Its gyro x is the pitch rate.
[[nodiscard]] constexpr float pitch_from_imu(const damp::Quaternion<float>& q) {
    return q.to_euler().roll() - damp::numbers::pi_v<float> / 2.0f - kPitchTrim;
}

/// The two LQIs, the command filters and the references they track.
class Balancer {
   public:
    /// Hold the pose the bot is in: references at the measurement, integrators and filters cleared.
    constexpr void engage(const Measurement& m) {
        pitch_.reset();
        yaw_.reset();
        speed_cmd_.reset();
        yaw_cmd_.reset();
        p_ref_   = position(m);
        psi_ref_ = heading(m);
    }

    /// One period: update → control → clamp → back_calculate → predict.
    constexpr WheelTorques step(const Measurement& m, const Command& cmd) {
        constexpr float r  = static_cast<float>(kBot.wheel_radius);
        constexpr float d  = static_cast<float>(kBot.track_width);
        constexpr float Ts = static_cast<float>(kTs);

        const float p        = position(m);
        const float v        = r * (0.5f * (m.wheel_rate_r + m.wheel_rate_l) + m.pitch_rate);
        const float psi      = heading(m);
        const float psi_rate = r * (m.wheel_rate_r - m.wheel_rate_l) / d;

        // References ride the filtered commands; a push drags them at most a leash behind.
        const float speed    = speed_cmd_(cmd.speed);
        const float yaw_rate = yaw_cmd_(cmd.yaw_rate);
        p_ref_               = damp::clamp(p_ref_ + speed * Ts, p - kLeash, p + kLeash);
        psi_ref_             = damp::clamp(psi_ref_ + yaw_rate * Ts, psi - kYawLeash, psi + kYawLeash);

        pitch_.update(ColVec<4, float>{p, v, m.pitch, m.pitch_rate});
        yaw_.update(ColVec<2, float>{psi, psi_rate});
        const float tau  = pitch_.control(ColVec<1, float>{{p_ref_}}, ColVec<4, float>{p_ref_, speed, 0.0f, 0.0f}, {})(0);
        const float dtau = yaw_.control(ColVec<1, float>{{psi_ref_}}, ColVec<2, float>{psi_ref_, yaw_rate}, {})(0);

        // Balance first: pitch takes what it needs, steering gets what is left on each wheel.
        constexpr float tau_max  = 2.0f * kWheelTorqueMax;
        const float     tau_sat  = damp::clamp(tau, -tau_max, tau_max);
        const float     dtau_max = tau_max - damp::abs(tau_sat);
        const float     dtau_sat = damp::clamp(dtau, -dtau_max, dtau_max);

        pitch_.back_calculate(ColVec<1, float>{{tau_sat}});
        yaw_.back_calculate(ColVec<1, float>{{dtau_sat}});
        pitch_.predict();
        yaw_.predict();

        return {.right = 0.5f * (tau_sat + dtau_sat), .left = 0.5f * (tau_sat - dtau_sat)};
    }

    /// [m] axle travel: the wheels' turn against the ground is the encoder plus the body's lean.
    [[nodiscard]] static constexpr float position(const Measurement& m) {
        return static_cast<float>(kBot.wheel_radius) * (0.5f * (m.wheel_r + m.wheel_l) + m.pitch);
    }

    /// [rad] heading from odometry, counter-clockwise from above.
    [[nodiscard]] static constexpr float heading(const Measurement& m) {
        return static_cast<float>(kBot.wheel_radius / kBot.track_width) * (m.wheel_r - m.wheel_l);
    }

   private:
    damp::Law<kPitchDesign, float> pitch_{};
    damp::Law<kYawDesign, float>   yaw_{};
    damp::LowPass<1, float>        speed_cmd_{kCommandCutoff, static_cast<float>(kTs)};
    damp::LowPass<1, float>        yaw_cmd_{kCommandCutoff, static_cast<float>(kTs)};
    float                          p_ref_   = 0.0f;
    float                          psi_ref_ = 0.0f;
};

enum class Mode {
    Idle,    ///< motors off; engages once held upright for kEngageTime
    Active,  ///< balancing
    Fault,   ///< IMU lost: latched until reset
};

struct Health {
    bool imu    = false;  ///< fresh IMU data
    bool drives = false;  ///< both ODrives report no axis error
};

struct Output {
    Mode         mode    = Mode::Idle;
    bool         changed = false;  ///< mode differs from the last tick: request the ODrive axis states
    WheelTorques torque{};
};

/// Supervisor plus balancer: the whole tick main.cpp and the SIL run.
class Controller {
   public:
    constexpr Output tick(const Measurement& m, const Health& h, const Command& cmd) {
        const Mode  before = mode_;
        const float tilt   = damp::abs(m.pitch);
        switch (mode_) {
            case Mode::Idle:
                if (upright_(tilt < kEngageTilt && h.imu && h.drives, static_cast<float>(kTs))) {
                    mode_ = Mode::Active;
                    balancer_.engage(m);
                }
                break;
            case Mode::Active:
                if (!h.imu) {
                    mode_ = Mode::Fault;
                } else if (tilt > kTipTilt || !h.drives) {
                    mode_ = Mode::Idle;
                    upright_.reset();
                }
                break;
            case Mode::Fault: break;
        }
        const WheelTorques torque = (mode_ == Mode::Active) ? balancer_.step(m, cmd) : WheelTorques{};
        return {.mode = mode_, .changed = mode_ != before, .torque = torque};
    }

    [[nodiscard]] constexpr Mode mode() const { return mode_; }

   private:
    Mode                  mode_ = Mode::Idle;
    damp::plc::TON<float> upright_{.PT = kEngageTime};
    Balancer              balancer_{};
};

}  // namespace bot
