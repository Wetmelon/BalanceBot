/**
 * @file balance_design.hpp
 * @brief BalanceBot's nameplate, plants, weights and the compile-time designs.
 *
 * No Arduino here. balance_controller.hpp runs the designs; docs/derivation.md derives the plant.
 * They have a header of their own because the designs are solved at compile time, and clang
 * (clangd) takes seconds to redo them: an edit to the control tick no longer reparses them.
 *
 * Two decoupled axes, each an LQI:
 *   pitch  x = [p, v, θ, ω], u = τ (sum of the wheel torques), integral on p: designed for
 *          the measured 22 ms delay, the law holds the last three torques sent
 *   yaw    x = [ψ, ψ̇],       u = Δτ (right minus left),      integral on ψ
 * The integrals hold the spot and the heading against a CoM offset, an IMU mount error or a slope.
 */

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

/// x², for the Bryson weights. Not Arduino's sq(): this header builds on the host too.
[[nodiscard]] constexpr double square(double x) { return x * x; }

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
constexpr double kPmax     = 0.02;                // [m]
constexpr double kVmax     = 0.5;                 // [m/s]
constexpr double kThetaMax = damp::deg2rad(6.0);  // [rad]
constexpr double kOmegaMax = 0.09;                // [rad/s]

inline constexpr damp::design::LQIConfig<4, 1, 1> kPitchWeights{
    .Q = damp::diag(
        1.0 / square(kPmax),
        1.0 / square(kVmax),
        1.0 / square(kThetaMax),
        1.0 / square(kOmegaMax)
    ),                    // p 5 cm, v 0.5 m/s, θ 6°, ω 0.09 rad/s
    .R          = 3.2,    // τ 0.56 N·m
    .Q_integral = 200.0,  // ∫p 0.07 m·s
};

inline constexpr damp::design::LQIConfig<2, 1, 1> kYawWeights{
    .Q          = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R          = 0.0625,                 // Δτ 4 N·m
    .Q_integral = 25.0,                   // ∫ψ 0.2 rad·s
};

inline constexpr float kWheelTorqueMax = 8.0f;   ///< [N·m] per wheel, the clamp on the law (traction alone holds about 2: μ ≈ 0.9 × m·g/2 × r)
inline constexpr float kLeanMax        = 0.26f;  ///< [rad] 15°: the most lean position, speed and the integral may ask for

/// [s] torque command to IMU reading, measured on the bot (docs/derivation.md): CAN, the drives'
/// current loop and the IMU together. The pitch law holds the torques still on their way in
/// kDelayTicks registers.
inline constexpr double kDelay      = 0.022;
inline constexpr size_t kDelayTicks = 3;

// Integral action on each plant's output y = C·x: p for pitch, ψ for yaw. The pitch design runs on
// the plant with the torque arriving kDelay late, its cost discretized exactly on that model.
inline constexpr auto kPitchDesign = damp::design::lqi(pitch_plant(kBot), kTs, damp::InputDelay<kDelayTicks>{kDelay}, kPitchWeights);
inline constexpr auto kYawDesign   = damp::design::lqi(yaw_plant(kBot), kTs, kYawWeights);

// ---- Experiment: the pitch law with a filter in place of the integral ------------------------------
// BOT_PITCH_LAW 0 is the LQI above, 1 an OFLQG, 2 an OFMPC (envs c33_oflqg, c33_ofmpc).
#ifndef BOT_PITCH_LAW
#define BOT_PITCH_LAW 0
#endif

/// The pitch plant with every state read, y = [p, v, θ, ω], and noise on each state and reading.
[[nodiscard]] constexpr auto pitch_plant_read(const Nameplate& n) {
    const auto s = pitch_plant(n);
    return damp::StateSpace<4, 1, 4, double, 4, 4>{.A = s.A, .B = s.B, .C = damp::Matrix<4, 4>::identity()};
}

/// What the filter estimates and the target cancels, constant until the readings say otherwise: d₀ a
/// torque on the wheels [N·m] (a slope, a hand, a sticky drivetrain), acting at once, not through the
/// drives' delay; d₁ the CoM's upright [rad], off the IMU's by a mount error or a load. The target holds p.
[[nodiscard]] constexpr auto pitch_disturbance(const Nameplate& n) {
    const auto                                 s = pitch_plant(n);
    damp::design::DisturbanceModel<4, 4, 2, 1> d{.outputs = {0}};
    for (size_t i = 0; i < 4; ++i) {
        d.Bd(i, 0) = s.B(i, 0);
        d.Bd(i, 1) = -s.A(i, 2);  // gravity pulls toward θ = d₁
    }
    return d;
}

/// Process densities [x²/s] (through G = I), reading variances [y²], the disturbances' walks [d²/s]:
/// guesses, until a log of the bot standing says better.
inline constexpr damp::design::KalmanNoise<4, 4, double, 2> kPitchNoise{
    .process     = damp::diag(1e-6, 1e-2, 1e-6, 1e-1),
    .measurement = damp::diag(square(1e-4), square(5e-3), square(1e-3), square(5e-3)),  // p 0.1 mm, v 5 mm/s, θ 1 mrad, ω 5 mrad/s
    .disturbance = damp::diag(square(0.3), square(0.01)),                                // 0.3 N·m, 0.6° in a second: faster rings the far corners
};

#if BOT_PITCH_LAW == 1
inline constexpr auto kPitchOFLQG = damp::design::oflqg(
    pitch_plant_read(kBot), kTs, damp::InputDelay<kDelayTicks>{kDelay}, {.Q = kPitchWeights.Q, .R = kPitchWeights.R}, kPitchNoise, pitch_disturbance(kBot)
);
#elif BOT_PITCH_LAW == 2
// The OFMPC sees the pitch plane from a frame riding the speed command, y = [p − p_ref, v − speed, θ, ω],
// so it regulates to a standstill: the plant is the same at any constant speed. It has the LQI's cost
// per step, and as constraints what the LQI gets from clamps: the torque, the lean, and how far off the
// speed command a push may take it (the drives run out of voltage at 1.05 m/s on 19 V).
inline constexpr size_t kHorizon      = 20;    ///< [steps] 0.2 s, the DARE's cost-to-go after it
inline constexpr double kSpeedMargin  = 1.0;   ///< [m/s] the most the bot may run off the speed command
inline constexpr auto   kPitchSampled = damp::discretize(pitch_plant_read(kBot), kTs, damp::InputDelay<kDelayTicks>{kDelay}).value();

/// pitch_disturbance, sampled: per step, through the plant's rows (at once), none on the registers.
[[nodiscard]] constexpr auto pitch_disturbance_sampled(const Nameplate& n) {
    const auto                                 s  = pitch_plant(n);
    const auto                                 dd = damp::discretize(damp::StateSpace<4, 2, 1>{.A = s.A, .B = pitch_disturbance(n).Bd, .C = s.C}, kTs).value();
    damp::design::DisturbanceModel<7, 4, 2, 0> d{};  // every reading held to its reference
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            d.Bd(i, j) = dd.B(i, j);
        }
    }
    return d;
}

/// The QP weighs signals scaled by each limit's range (1 for an unbounded one): the LQI's Q and R,
/// per step, scaled to match.
[[nodiscard]] constexpr auto pitch_mpc_design() {
    constexpr double big = damp::numeric_limits<double>::max();  // no limit
    const double     lean = (double)(kLeanMax), tau = 2.0 * (double)(kWheelTorqueMax);
    const double     s[4] = {1.0, 2.0 * kSpeedMargin, 2.0 * lean, 1.0};
    const double     s_u  = 2.0 * tau;

    damp::design::MPCWeights<1, 4> w{};
    for (size_t i = 0; i < 4; ++i) {
        w.Qy(i, i) = kPitchWeights.Q(i, i) * kTs * square(s[i]);
    }
    w.Qy_terminal = w.Qy;
    w.Ru          = kPitchWeights.R * (kTs * square(s_u));
    w.Rdu         = damp::Matrix<1, 1>{1e-4 * square(s_u)};  // [per (N·m)²] a little on each step, which the QP needs

    damp::design::MPCConstraints<1, 4> c{.terminal = damp::design::MPCTerminal::dare_cost};
    c.u_lim = {.lower = {-tau}, .upper = {tau}};
    c.y_lim = {.lower = {-big, -kSpeedMargin, -lean, -big}, .upper = {big, kSpeedMargin, lean, big}};

    // Discrete noise: G is ∫e^{Aτ}dτ ≈ Ts·I, so the density over Ts; the walks are per step.
    damp::design::KalmanNoise<4, 4, double, 2> noise = kPitchNoise;
    noise.process                                    = kPitchNoise.process * (1.0 / kTs);
    noise.disturbance                                = kPitchNoise.disturbance * kTs;
    return damp::design::ofmpc<kHorizon, kHorizon>(kPitchSampled, w, c, noise, pitch_disturbance_sampled(kBot));
}
inline constexpr auto kPitchOFMPC = pitch_mpc_design();
#endif

}  // namespace bot
