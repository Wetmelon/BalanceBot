// BalanceBot's nameplate, plants, weights and compile-time designs (derived in docs/derivation.md). No
// Arduino. Kept apart from the tick so clangd doesn't re-solve the designs on every edit there.
//
//   pitch  x = [p, v, θ, ω], u = τ (sum of the wheel torques), integral on p, 22 ms input delay
//   yaw    x = [ψ, ψ̇],       u = Δτ (right minus left),      integral on ψ

#pragma once

#include <cstddef>

#include "damp/controllers/lqi.hpp"
#include "damp/controllers/oflqg.hpp"
#include "damp/controllers/ofmpc.hpp"
#include "damp/math/math.hpp"
#include "damp/matrix/concat.hpp"
#include "damp/matrix/matrix.hpp"
#include "damp/systems/discretization.hpp"
#include "damp/systems/state_space.hpp"

namespace bot {

/// Not Arduino's sq(): this header builds on the host too.
[[nodiscard]] constexpr double square(double x) { return x * x; }

/// Physical parameters; "measure" marks an estimate. com_height and body_inertia come from the CoM
/// (4.25 in above the axle) and a drop test (docs/derivation.md).
struct Nameplate {
    double wheel_radius  = 0.08255;  ///< [m] 6.5 in BotWheel
    double track_width   = 0.400;    ///< [m] between the tire contact patches: 15.75 in
    double com_height    = 0.666;    ///< [m] body CoM above the floor
    double body_mass     = 1.0;      ///< [kg] everything that pitches (measure)
    double body_inertia  = 0.134;    ///< [kg·m²] body about its own CoM, pitch axis
    double wheel_mass    = 2.2;      ///< [kg] per side, what rides on the axle
    double wheel_inertia = 0.006;    ///< [kg·m²] per side, the rotating part about the axle (measure)
    double yaw_inertia   = 0.18;     ///< [kg·m²] whole bot about the vertical (measure)
    double gravity       = 9.81;     ///< [m/s²]
    double torque_gain   = 1.0;      ///< the fraction of the asked torque the motors deliver
};

inline constexpr Nameplate kBot{};

/// As identified on the bot above 2 Hz: 0.88 of the asked torque, 13.4 kg translating.
inline constexpr Nameplate kBotFitted{.wheel_inertia = 0.027, .torque_gain = 0.88};
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

/// Pitch plane linearized upright: x = [p, v, θ, ω], u = τ, y = p.
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
constexpr double kOmegaMax = 0.12;                // [rad/s]

inline constexpr float kLeanMax = 0.26f;  ///< [rad] 15°: the most lean position, speed and the integral may ask for

inline constexpr damp::design::LQIConfig kPitchWeights{
    .Q = damp::diag(
        1.0 / square(kPmax),
        1.0 / square(kVmax),
        1.0 / square(kThetaMax),
        1.0 / square(kOmegaMax)
    ),
    .R          = {1.5},    // τ 0.82 N·m
    .Q_integral = {350.0},  // ∫p 0.053 m·s
    // The lean p, v and ∫p ask for is capped at kLeanMax: uncapped, a bot held off its reference leans on until it falls.
    .demand_lim = {.state = 2, .outer = {true, true, false, false, true}, .lim = {.lower = -(double)(kLeanMax), .upper = (double)(kLeanMax)}},
};

inline constexpr damp::design::LQIConfig kYawWeights{
    .Q          = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R          = {0.0625},               // Δτ 4 N·m
    .Q_integral = {25.0},                 // ∫ψ 0.2 rad·s
};

inline constexpr float kWheelTorqueMax = 8.0f;  ///< [N·m] per wheel, the clamp on the law (traction holds about 2)

/// [s] torque command to IMU reading, measured: CAN, the drives' current loop and the IMU together.
inline constexpr double kDelay      = 0.022;
inline constexpr size_t kDelayTicks = 3;

inline constexpr auto kPitchDesign = damp::design::lqi(pitch_plant(kBot), kTs, damp::InputDelay<kDelayTicks>{kDelay}, kPitchWeights);
inline constexpr auto kYawDesign   = damp::design::lqi(yaw_plant(kBot), kTs, kYawWeights);

// Experiment: the pitch law with a filter in place of the integral. BOT_PITCH_LAW 0 is the LQI above,
// 1 an OFLQG, 2 an OFMPC (envs c33_oflqg, c33_ofmpc).
#ifndef BOT_PITCH_LAW
#define BOT_PITCH_LAW 0
#endif

/// The pitch plant with every state read, y = [p, v, θ, ω].
[[nodiscard]] constexpr auto pitch_plant_read(const Nameplate& n) {
    const auto s = pitch_plant(n);
    return damp::StateSpace{.A = s.A, .B = s.B, .C = damp::Matrix<4, 4>::identity()};
}

/// What the filter estimates and the target cancels: d₀ a torque on the wheels [N·m] (a slope, a hand,
/// a sticky drivetrain), acting at once; d₁ the CoM's upright [rad], off the IMU's by a mount error or a load.
[[nodiscard]] constexpr auto pitch_disturbance(const Nameplate& n) {
    const auto                                 s = pitch_plant(n);
    damp::design::DisturbanceModel<4, 4, 2, 1> d{.outputs = {0}};
    for (size_t i = 0; i < 4; ++i) {
        d.Bd(i, 0) = s.B(i, 0);
        d.Bd(i, 1) = -s.A(i, 2);  // gravity pulls toward θ = d₁
    }
    return d;
}

/// Process densities [x²/s], reading variances [y²], the disturbances' walks [d²/s]: guesses.
inline constexpr damp::design::KalmanNoise kPitchNoise{
    .process     = damp::diag(1e-6, 1e-2, 1e-6, 1e-1),
    .measurement = damp::diag(square(1e-4), square(5e-3), square(1e-3), square(5e-3)),  // p 0.1 mm, v 5 mm/s, θ 1 mrad, ω 5 mrad/s
    .disturbance = damp::diag(square(0.3), square(0.02)),                               // 0.3 N·m, 1.1° in a second
};

/// The OFLQG's and OFMPC's cost: the LQI's p, v and θ weights, R 3.2, and ω 0.05 rad/s to damp the
/// loop's 1 Hz mode. demand_lim caps the lean p and v ask for at kLeanMax, as the LQI's does.
inline constexpr damp::design::LQRConfig kPitchFilterCost{
    .Q          = damp::diag(1.0 / square(kPmax), 1.0 / square(kVmax), 1.0 / square(kThetaMax), 1.0 / square(0.05)),
    .R          = {3.2},
    .demand_lim = {.state = 2, .outer = {true, true, false, false}, .lim = {.lower = -(double)(kLeanMax), .upper = (double)(kLeanMax)}},
};

#if BOT_PITCH_LAW == 1
inline constexpr auto kPitchOFLQG = damp::design::oflqg(
    pitch_plant_read(kBotFitted), kTs, damp::InputDelay<kDelayTicks>{kDelay}, kPitchFilterCost, kPitchNoise, pitch_disturbance(kBotFitted)
);
#elif BOT_PITCH_LAW == 2
// The OFMPC regulates y = [p − p_ref, v − speed, θ, ω] to a standstill, in a frame riding the speed
// command. Its constraints are the LQI's clamps: torque, lean, and speed off the command.
inline constexpr size_t kHorizon      = 10;   ///< [steps] 0.1 s, then the DARE's cost-to-go
inline constexpr size_t kMoves        = 5;    ///< [steps] the last move holds to the horizon
inline constexpr size_t kQpIterations = 40;   ///< ~2.1 ms on the C33 if used up; a push takes 10
inline constexpr double kSpeedMargin  = 1.0;  ///< [m/s] the most the bot may run off the speed command

// Its 4 noise inputs named: discretize keeps them off the delay states, which a model that names
// none would put noise on.
inline constexpr auto kPitchRead    = pitch_plant_read(kBotFitted);
inline constexpr auto kPitchSampled = damp::discretize(damp::StateSpace<4, 1, 4, double, 4, 4>{.A = kPitchRead.A, .B = kPitchRead.B, .C = kPitchRead.C}, kTs, damp::InputDelay<kDelayTicks>{kDelay}).value();

/// pitch_disturbance, sampled: per step, through the plant's rows (at once), none on the delay states.
[[nodiscard]] constexpr auto pitch_disturbance_sampled(const Nameplate& n) {
    const auto                                 s  = pitch_plant(n);
    const auto                                 dd = damp::discretize(damp::StateSpace{.A = s.A, .B = pitch_disturbance(n).Bd, .C = s.C}, kTs).value();
    damp::design::DisturbanceModel<7, 4, 2, 0> d{};
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            d.Bd(i, j) = dd.B(i, j);
        }
    }
    return d;
}

/// kPitchFilterCost per step, scaled as the QP scales each signal: by its limit's range (1 if unbounded).
[[nodiscard]] constexpr auto pitch_mpc_design() {
    constexpr double big  = damp::numeric_limits<double>::max();
    const double     lean = (double)(kLeanMax), tau = 2.0 * (double)(kWheelTorqueMax);
    const double     s[4] = {1.0, 2.0 * kSpeedMargin, 2.0 * lean, 1.0};
    const double     s_u  = 2.0 * tau;

    damp::design::MPCWeights<1, 4> w{};
    for (size_t i = 0; i < 4; ++i) {
        w.Qy(i, i) = kPitchFilterCost.Q(i, i) * kTs * square(s[i]);
    }
    w.Qy_terminal = w.Qy;
    w.Ru          = kPitchFilterCost.R * (kTs * square(s_u));
    w.Rdu         = 0.3 * square(s_u);  // [per (N·m)²] torque step: less buzzes at the lean limit

    damp::design::MPCConstraints<1, 4> c{.terminal = damp::design::MPCTerminal::dare_cost, .max_qp_iterations = kQpIterations};
    c.u_lim = {.lower = {-tau}, .upper = {tau}};
    c.y_lim = {.lower = {-big, -kSpeedMargin, -lean, -big}, .upper = {big, kSpeedMargin, lean, big}};

    // Discrete noise: G ≈ Ts·I, so the density over Ts; the walks per step.
    auto noise        = kPitchNoise;
    noise.process     = kPitchNoise.process * (1.0 / kTs);
    noise.disturbance = kPitchNoise.disturbance * kTs;
    return damp::design::ofmpc<kHorizon, kMoves>(kPitchSampled, w, c, noise, pitch_disturbance_sampled(kBotFitted));
}
inline constexpr auto kPitchOFMPC = pitch_mpc_design();
#endif

}  // namespace bot
