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

inline constexpr damp::design::LQRWeights<4, 1, 1> kPitchWeights{
    .Q = damp::diag(
        1.0 / square(kPmax),
        1.0 / square(kVmax),
        1.0 / square(kThetaMax),
        1.0 / square(kOmegaMax)
    ),                    // p 5 cm, v 0.5 m/s, θ 6°, ω 0.09 rad/s
    .R          = 3.2,    // τ 0.56 N·m
    .Q_integral = 200.0,  // ∫p 0.07 m·s
};

inline constexpr damp::design::LQRWeights<2, 1, 1> kYawWeights{
    .Q          = damp::diag(25.0, 1.0),  // ψ 0.2 rad, ψ̇ 1 rad/s
    .R          = 0.0625,                 // Δτ 4 N·m
    .Q_integral = 25.0,                   // ∫ψ 0.2 rad·s
};

/// [s] torque command to IMU reading, measured on the bot (docs/derivation.md): CAN, the drives'
/// current loop and the IMU together. The pitch law holds the torques still on their way in
/// kDelayTicks registers.
inline constexpr double kDelay      = 0.022;
inline constexpr size_t kDelayTicks = 3;

// Integral action on each plant's output y = C·x: p for pitch, ψ for yaw. The pitch design runs on
// the plant with the torque arriving kDelay late, its cost discretized exactly on that model.
inline constexpr auto kPitchDesign = damp::design::lqi(pitch_plant(kBot), kTs, damp::InputDelay<kDelayTicks>{kDelay}, kPitchWeights);
inline constexpr auto kYawDesign   = damp::design::lqi(yaw_plant(kBot), kTs, kYawWeights);

}  // namespace bot
