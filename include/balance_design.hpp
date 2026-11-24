// BalanceBot's nameplate, plants, weights and compile-time designs (derived in docs/derivation.md). No
// Arduino. Kept apart from the tick so clangd doesn't re-solve the designs on every edit there.
//
//   pitch  x = [p, v, θ, ω], u = τ (sum of the wheel torques), integral on p, 22 ms input delay
//   yaw    x = [ψ, ψ̇],       u = Δτ (right minus left),      integral on ψ

#pragma once

#include <cstddef>

#include "damp/controllers/mpc.hpp"
#include "damp/controllers/state_feedback.hpp"
#include "damp/math/math.hpp"
#include "damp/matrix/concat.hpp"
#include "damp/matrix/matrix.hpp"
#include "damp/systems/discretization.hpp"
#include "damp/systems/state_space.hpp"

namespace bot {

/// Not Arduino's sq(): this header builds on the host too.
[[nodiscard]] constexpr double square(double x) { return x * x; }

/// Physical parameters; "measure" marks an estimate. com_height and body_inertia come from the CoM
/// (4.25 in above the axle) and a drop test; wheel_inertia and torque_gain from the fit on the bot
/// above 2 Hz (docs/derivation.md).
struct Nameplate {
    double wheel_radius  = 0.08255;  ///< [m] 6.5 in BotWheel
    double track_width   = 0.400;    ///< [m] between the tire contact patches: 15.75 in
    double com_height    = 0.666;    ///< [m] body CoM above the floor
    double body_mass     = 1.0;      ///< [kg] everything that pitches (measure)
    double body_inertia  = 0.134;    ///< [kg·m²] body about its own CoM, pitch axis
    double wheel_mass    = 2.2;      ///< [kg] per side, what rides on the axle
    double wheel_inertia = 0.027;    ///< [kg·m²] per side, as fitted: 13.4 kg translating (the rotating part alone is ~0.006)
    double yaw_inertia   = 0.18;     ///< [kg·m²] whole bot about the vertical (measure)
    double gravity       = 9.81;     ///< [m/s²]
    double torque_gain   = 0.88;     ///< the fraction of the asked torque the motors deliver, as fitted
};

inline constexpr Nameplate kBot{};
inline constexpr double    kTs = 0.01;  ///< [s] 100 Hz control period

/// Coefficients of the pitch-plane mass matrix [a, b cosθ; b cosθ, c].
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

/// Pitch plane linearized upright: x = [p, v, θ, ω], u = τ, y = x (the bot reads every state).
[[nodiscard]] constexpr auto pitch_plant(const Nameplate& n) {
    const auto [a, b, c] = pitch_mass(n);
    const double r       = n.wheel_radius;
    const double g       = n.gravity;
    const double D       = a * c - b * b;
    return damp::StateSpace{
        .A = {
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, -b * b * g / D, 0.0},
            {0.0, 0.0, 0.0, 1.0},
            {0.0, 0.0, a * b * g / D, 0.0},
        },
        .B = {
            {0.0},
            {n.torque_gain * (c / r + b) / D},
            {0.0},
            {-n.torque_gain * (a + b / r) / D},
        },
        .C = {
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 1.0},
        },
    };
}

/// Yaw: x = [ψ, ψ̇], u = Δτ = τ_R − τ_L, y = ψ.
[[nodiscard]] constexpr auto yaw_plant(const Nameplate& n) {
    return damp::StateSpace{
        .A = {{0.0, 1.0}, {0.0, 0.0}},
        .B = {{0.0}, {n.torque_gain * n.track_width / (2.0 * n.wheel_radius * yaw_inertia(n))}},
        .C = {{1.0, 0.0}},
    };
}

// Bryson weights, 1 / (acceptable excursion)². ω, R and ∫p are tuned on the SIL's far corner, which
// must hold the spot within 5 cm and ring under 1° after a push. A failed design does not compile.
constexpr double kPmax     = 0.02;                // [m]
constexpr double kVmax     = 0.5;                 // [m/s]
constexpr double kThetaMax = damp::deg2rad(6.0);  // [rad]
constexpr double kOmegaMax = 0.09;                // [rad/s]

inline constexpr float kLeanMax = 0.26f;  ///< [rad] 15°: the most lean position, speed and the integral may ask for

inline constexpr damp::design::LQRWeights kPitchWeights{
    .Q = damp::diag(
        1.0 / square(kPmax),
        1.0 / square(kVmax),
        1.0 / square(kThetaMax),
        1.0 / square(kOmegaMax)
    ),
    .R = {3.2},  // τ 0.56 N·m
};
// The lean p, v and ∫p (states 0, 1, then the integrator, 4) ask for is capped at kLeanMax: uncapped, a bot held off its reference leans on until it falls.
inline constexpr damp::DemandLimit            kPitchLean{.state = 2, .outer = {0, 1, 4}, .lower = -(double)(kLeanMax), .upper = (double)(kLeanMax)};
inline constexpr damp::design::IntegralAction kPitchIntegral{.Q = 200.0, .outputs = {0}};  // ∫p 0.07 m·s

inline constexpr damp::design::LQRWeights kYawWeights{
    .Q = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R = {0.0625},               // Δτ 4 N·m
};

inline constexpr float kWheelTorqueMax = 8.0f;  ///< [N·m] per wheel, the clamp on the law (traction holds about 2)

/// The pitch law's clamp, both wheels' torque: the law holds its integrator against it itself.
inline constexpr damp::design::InputLimits kPitchTorqueLim{.lower = {-2.0 * (double)(kWheelTorqueMax)}, .upper = {2.0 * (double)(kWheelTorqueMax)}};

/// [s] torque command to IMU reading, measured: CAN, the drives' current loop and the IMU together.
inline constexpr double kDelay      = 0.022;
inline constexpr size_t kDelayTicks = 3;

inline constexpr auto kYawDesign = damp::design::state_feedback(yaw_plant(kBot), kTs, kYawWeights, damp::design::IntegralAction{.Q = 25.0});  // ∫ψ 0.2 rad·s

// The pitch law: BOT_PITCH_LAW 0 an LQI (the default), 1 an OFLQG, 2 an OFMPC (envs c33_oflqg,
// c33_ofmpc). 1 and 2 put a filter in place of the integral.
#ifndef BOT_PITCH_LAW
#define BOT_PITCH_LAW 0
#endif

/// What the filter estimates and the target cancels: d₀ a torque on the wheels [N·m] (a slope, a hand,
/// a sticky drivetrain), acting at once; d₁ the CoM's upright [rad], off the IMU's by a mount error or a load.
[[nodiscard]] constexpr auto pitch_disturbance(const Nameplate& n) {
    const auto s = pitch_plant(n);
    return damp::design::DisturbanceModel<4, 4, 2, 1>{.Bd = damp::hcat(s.B, -s.A.col(2)), .outputs = {0}};  // d₁: gravity pulls toward θ = d₁
}

/// Process densities [x²/s], reading variances [y²], the disturbances' walks [d²/s]: guesses.
inline constexpr damp::design::KalmanNoise kPitchNoise{
    .process     = damp::diag(1e-6, 1e-2, 1e-6, 1e-1),
    .measurement = damp::diag(square(1e-4), square(5e-3), square(1e-3), square(5e-3)),  // p 0.1 mm, v 5 mm/s, θ 1 mrad, ω 5 mrad/s
    .disturbance = damp::diag(square(0.3), square(0.02)),                               // 0.3 N·m, 1.1° in a second
};

/// The OFLQG's and OFMPC's cost: the LQI's p, v and θ weights, R 3.2, and ω 0.05 rad/s to damp the
/// loop's 1 Hz mode. kPitchFilterLean caps the lean p and v ask for at kLeanMax, as kPitchLean does the LQI's.
inline constexpr damp::design::LQRWeights kPitchFilterCost{
    .Q = damp::diag(1.0 / square(kPmax), 1.0 / square(kVmax), 1.0 / square(kThetaMax), 1.0 / square(0.05)),
    .R = {3.2},
};
inline constexpr damp::DemandLimit kPitchFilterLean{.state = 2, .outer = {0, 1}, .lower = -(double)(kLeanMax), .upper = (double)(kLeanMax)};

#if BOT_PITCH_LAW == 0
inline constexpr auto kPitchLQI = damp::design::state_feedback(pitch_plant(kBot), kTs, kPitchWeights, kPitchTorqueLim, kPitchLean, kPitchIntegral, damp::InputDelay<kDelayTicks>{kDelay});
#elif BOT_PITCH_LAW == 1
inline constexpr auto kPitchOFLQG = damp::design::state_feedback(
    pitch_plant(kBot), kTs, kPitchFilterCost, kPitchTorqueLim, kPitchFilterLean, kPitchNoise, pitch_disturbance(kBot), damp::InputDelay<kDelayTicks>{kDelay}
);
#elif BOT_PITCH_LAW == 2
// The OFMPC on the OFLQG's plant, noise, disturbance model and delay: p held at r, about the point
// the tick plans (the reference position at the speed command). Its constraints are the LQI's
// clamps: torque, lean, and speed within kSpeedMargin of the command, which the tick moves with it.
inline constexpr size_t kHorizon      = 10;   ///< [steps] 0.1 s, then the DARE's cost-to-go
inline constexpr size_t kMoves        = 5;    ///< [steps] the last move holds to the horizon
inline constexpr size_t kQpIterations = 40;   ///< ~2.1 ms on the C33 if used up; a push takes 10
inline constexpr double kSpeedMargin  = 1.0;  ///< [m/s] the most the bot may run off the speed command

/// kPitchFilterCost per second, as the LQ designs take it (Qy and Ru over each period, RΔu on the
/// torque's rate), scaled as the QP scales each signal: by its limit's range (1 if unbounded).
[[nodiscard]] constexpr auto pitch_mpc_design() {
    const double lean = (double)(kLeanMax), tau = 2.0 * (double)(kWheelTorqueMax);
    const double s[4] = {1.0, 2.0 * kSpeedMargin, 2.0 * lean, 1.0};
    const double s_u  = 2.0 * tau;

    damp::design::MPCWeights<1, 4> w{};
    for (size_t i = 0; i < 4; ++i) {
        w.Qy(i, i) = kPitchFilterCost.Q(i, i) * square(s[i]);
    }
    w.Qy_terminal = w.Qy;
    w.Ru          = kPitchFilterCost.R * square(s_u);
    w.Rdu         = 0.3 * kTs * square(s_u);  // 0.3 per (N·m)² a torque step: less buzzes at the lean limit

    return damp::design::mpc<kHorizon, kMoves>(
        pitch_plant(kBot), kTs, w, damp::design::InputLimits{.lower = {-tau}, .upper = {tau}},
        damp::design::OutputLimits{.outputs = {1, 2}, .lower = {-kSpeedMargin, -lean}, .upper = {kSpeedMargin, lean}}, damp::design::DareCost{},
        damp::design::QPIterations{kQpIterations}, kPitchNoise, pitch_disturbance(kBot), damp::InputDelay<kDelayTicks>{kDelay}
    );
}
inline constexpr auto kPitchOFMPC = pitch_mpc_design();
#endif

}  // namespace bot
