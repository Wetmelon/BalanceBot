/**
 * @file balance_controller.hpp
 * @brief BalanceBot deploy header: nameplate, plant, design and the control tick.
 *
 * No Arduino here. main.cpp flashes it; sil/balance_sil.cpp closes the loop on the host around
 * the same objects. The plant is derived in docs/derivation.md.
 *
 * Two decoupled axes, each an LQI designed at compile time:
 *   pitch  x = [p, v, θ, ω, τ₋₁, τ₋₂, τ₋₃], u = τ (sum of the wheel torques), integral on p:
 *          on the plant with its measured 22 ms delay, the last three torques sent as states
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
#include "damp/systems/discretization.hpp"
#include "damp/systems/state_space.hpp"
#include "damp/toolbox/iec61131.hpp"

#include <bit>
#include <cstdint>

namespace bot {

using damp::ColVec;

/// Physical parameters. "measure" marks an estimate: weigh and swing the real bot, then rerun the SIL.
///
/// The pitch plane only sees three numbers (pitch_mass): the translating mass a, the body's mass
/// moment b = m·l and its inertia about the axle c. Two are measured: b from the whole bot's CoM,
/// 4.25 in above the axle (the BotWheels' mass sits on the axle, so b = 5.4 kg × 0.108 m), and c
/// from a drop test, the free fall with the drives idle at λ = 3.66 rad/s:
/// c = b²/a + b·g/λ². body_mass only splits b and c into com_height and body_inertia.
struct Nameplate {
    double wheel_radius  = 0.08255;  ///< [m] 6.5 in BotWheel
    double track_width   = 0.400;    ///< [m] between the tire contact patches: 15.75 in
    double com_height    = 0.666;    ///< [m] body CoM above the floor: r + b / body_mass
    double body_mass     = 1.0;      ///< [kg] everything that pitches: 48 in extrusion, acrylic, battery (measure)
    double body_inertia  = 0.134;    ///< [kg·m²] body about its own CoM, pitch axis: c − b² / body_mass
    double wheel_mass    = 2.2;      ///< [kg] per side, what rides on the axle: BotWheel, per ODrive
    double wheel_inertia = 0.006;    ///< [kg·m²] per side, the rotating part about the axle (measure)
    double yaw_inertia   = 0.18;     ///< [kg·m²] whole bot about the vertical: the wheels at ±d/2 (measure)
    double gravity       = 9.81;     ///< [m/s²]
};

inline constexpr Nameplate kBot{};
inline constexpr double    kTs = 0.01;  ///< [s] 100 Hz control period

inline constexpr float kWheelTorqueMax = 2.0f;    ///< [N·m] per wheel: traction, μ ≈ 0.9 × m·g/2 × r
inline constexpr float kEngageTilt     = 0.087f;  ///< [rad] 5°: held this upright ...
inline constexpr float kEngageTime     = 2.0f;    ///< [s] ... this long, the bot engages
inline constexpr float kTipTilt        = 0.52f;   ///< [rad] 30°: past this it has fallen, motors off
inline constexpr float kLeash          = 0.3f;    ///< [m] position error the reference may lag a push by
inline constexpr float kYawLeash       = 0.5f;    ///< [rad] heading error the reference may lag by
inline constexpr float kCommandCutoff  = 0.5f;    ///< [Hz] low-pass on the speed and yaw-rate commands

/// [rad] IMU pitch reading with the CoM over the axle. Calibrate: balance it by hand, read the telemetry,
/// or fit the drop test's falls both ways (here: +0.004 rad).
inline constexpr float kPitchTrim = 0.004f;

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
        .A = {
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, -b * b * g / D, 0.0},
            {0.0, 0.0, 0.0, 1.0},
            {0.0, 0.0, a * b * g / D, 0.0},
        },
        .B = {
            {0.0},
            {(c / r + b) / D},
            {0.0},
            {-(a + b / r) / D},
        },
        .C = {
            {1.0, 0.0, 0.0, 0.0},
        },
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
// Pitch: the tall body falls at 3.66 rad/s and puts the non-minimum-phase zero of torque to position
// at 3.3 rad/s, so position is slow. The torque penalty holds the margin the SIL asks for (±20 % on
// the nameplate with every reading a period late); the weight on ω damps the ring the bot showed
// after a push, which the model does not (docs/derivation.md). A failed design does not compile.
inline constexpr damp::design::LQRWeights<4, 1, 1> kPitchWeights{
    .Q          = damp::diag(400.0, 4.0, 100.0, 128.0),  // p 5 cm, v 0.5 m/s, θ 6°, ω 0.09 rad/s
    .R          = 3.2,                                   // τ 0.56 N·m
    .Q_integral = 200.0,                                 // ∫p 0.07 m·s
};
inline constexpr damp::design::LQRWeights<2, 1, 1> kYawWeights{
    .Q          = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R          = 0.0625,                 // Δτ 4 N·m
    .Q_integral = 25.0,                   // ∫ψ 0.2 rad·s
};

/// [s] torque command to IMU reading, measured on the bot (docs/derivation.md): CAN, the drives'
/// current loop and the IMU together. The pitch design carries it as kDelayTicks input registers.
inline constexpr double kDelay      = 0.022;
inline constexpr size_t kDelayTicks = 3;

/// The pitch plant sampled at Ts with the torque arriving kDelay late: [x; τ₋₁ … τ₋₃], u = τ now.
[[nodiscard]] constexpr auto pitch_model(const Nameplate& n) {
    return damp::discretize(pitch_plant(n), kTs, damp::InputDelay<kDelayTicks>{kDelay}).value();
}

// A discrete design takes the cost per step: the continuous weights × Ts give the same gains
// within 1 % without the delay; the registers carry no weight of their own.
inline constexpr auto kPitchDesign = damp::design::lqi(pitch_model(kBot), damp::design::LQRWeights<4 + kDelayTicks, 1, 1>{
    .Q          = damp::blkdiag(kPitchWeights.Q * kTs, damp::Matrix<kDelayTicks, kDelayTicks>{}),
    .R          = kPitchWeights.R * kTs,
    .Q_integral = kPitchWeights.Q_integral * kTs,
});
inline constexpr auto kYawDesign = damp::design::lqi(yaw_plant(kBot), kTs, kYawWeights);

/// Sensor readings, bot frame, forward positive.
struct Measurement {
    float pitch        = 0.0f;  ///< [rad] lean, forward positive
    float pitch_rate   = 0.0f;  ///< [rad/s]
    float wheel_r      = 0.0f;  ///< [rad] right wheel against the body (the motor encoder)
    float wheel_l      = 0.0f;  ///< [rad] left wheel against the body
    float wheel_rate_r = 0.0f;  ///< [rad/s]
    float wheel_rate_l = 0.0f;  ///< [rad/s]
};

/// Every reading a number. A drive with no position to report (absolute setpoints on and no index,
/// for one) sends NaN, and a NaN state makes the law ask for full torque. Checked on the bits:
/// -ffast-math lets the compiler assume a float is never NaN and drop a comparison that tests it.
[[nodiscard]] constexpr bool finite(const Measurement& m) {
    for (const float v : {m.pitch, m.pitch_rate, m.wheel_r, m.wheel_l, m.wheel_rate_r, m.wheel_rate_l}) {
        if ((std::bit_cast<uint32_t>(v) & 0x7F800000U) == 0x7F800000U) {
            return false;
        }
    }
    return true;
}

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
        sent_    = {};
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

        // The torques still on their way to the wheels are part of the state.
        pitch_.update(ColVec<7, float>{p, v, m.pitch, m.pitch_rate, sent_[0], sent_[1], sent_[2]});
        yaw_.update(ColVec<2, float>{psi, psi_rate});
        const float tau  = pitch_.control(ColVec<1, float>{p_ref_}, ColVec<7, float>{p_ref_, speed, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {})(0);
        const float dtau = yaw_.control(ColVec<1, float>{psi_ref_}, ColVec<2, float>{psi_ref_, yaw_rate}, {})(0);

        // Balance first: pitch takes what it needs, steering gets what is left on each wheel.
        constexpr float tau_max  = 2.0f * kWheelTorqueMax;
        const float     tau_sat  = damp::clamp(tau, -tau_max, tau_max);
        const float     dtau_max = tau_max - damp::abs(tau_sat);
        const float     dtau_sat = damp::clamp(dtau, -dtau_max, dtau_max);

        pitch_.back_calculate(ColVec<1, float>{tau_sat});
        yaw_.back_calculate(ColVec<1, float>{dtau_sat});
        pitch_.predict();
        yaw_.predict();
        sent_ = {tau_sat, sent_[0], sent_[1]};

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
    ColVec<kDelayTicks, float>     sent_{};  ///< [N·m] the last pitch torques sent, newest first
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
        const bool  drives = h.drives && finite(m);  // a reading that is not a number is a drive fault
        switch (mode_) {
            case Mode::Idle:
                if (upright_(tilt < kEngageTilt && h.imu && drives, static_cast<float>(kTs))) {
                    mode_ = Mode::Active;
                    balancer_.engage(m);
                }
                break;
            case Mode::Active:
                if (!h.imu) {
                    mode_ = Mode::Fault;
                } else if (tilt > kTipTilt || !drives) {
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
