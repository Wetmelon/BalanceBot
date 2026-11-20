// BalanceBot's limits and control tick. No Arduino: main.cpp flashes it, sil/balance_sil.cpp runs it on
// the host. The plants and designs are in balance_design.hpp.

#pragma once

#include <bit>
#include <cstdint>

#include "balance_design.hpp"
#include "damp/controllers/lqi.hpp"
#include "damp/filters/lowpass.hpp"
#include "damp/math/geometry.hpp"
#include "damp/math/math.hpp"
#include "damp/matrix/matrix.hpp"
#include "damp/toolbox/iec61131.hpp"

namespace bot {

using damp::ColVec;

// The drives' own limits, sent at each engage: a backstop, the law never asks past kWheelTorqueMax.
inline constexpr float kDriveVelocityLimit = 10.0f;  ///< [rev/s] 5.2 m/s
inline constexpr float kDriveCurrentLimit  = 80.0f;  ///< [A] the S1's maximum

inline constexpr float kEngageTilt    = 0.087f;  ///< [rad] 5°: held this upright ...
inline constexpr float kEngageTime    = 2.0f;    ///< [s] ... this long, the bot engages
inline constexpr float kTipTilt       = 0.52f;   ///< [rad] 30°: past this it has fallen, motors off
inline constexpr float kLeash         = 0.3f;    ///< [m] position error the reference may lag a push by
inline constexpr float kYawLeash      = 0.5f;    ///< [rad] heading error the reference may lag by
inline constexpr float kIntegralLean  = 0.14f;   ///< [rad] 8°: the most lean the integral may hold
inline constexpr float kCommandCutoff = 0.5f;    ///< [Hz] low-pass on the speed and yaw-rate commands

/// [N·m, rad] the most d̂ the OFLQG or OFMPC holds: past it, a wall reads as a disturbance to lean on.
inline constexpr damp::Bounds<2, float> kDisturbanceLim{.lower = {-2.0f, -kIntegralLean}, .upper = {2.0f, kIntegralLean}};

/// [rad] IMU pitch with the CoM over the axle: balance the bot by hand and read the telemetry.
inline constexpr float kPitchTrim = 0.004f;

/// Sensor readings, bot frame, forward positive.
struct Measurement {
    float pitch        = 0.0f;  ///< [rad]
    float pitch_rate   = 0.0f;  ///< [rad/s]
    float wheel_r      = 0.0f;  ///< [rad] right wheel against the body
    float wheel_l      = 0.0f;  ///< [rad] left wheel against the body
    float wheel_rate_r = 0.0f;  ///< [rad/s]
    float wheel_rate_l = 0.0f;  ///< [rad/s]
};

/// No NaN or inf in any reading: a drive with no position sends NaN, and the law turns it into full
/// torque. Checked on the bits because -ffast-math folds isnan away.
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

/// Lean from the game rotation vector. The IMU is on its side: x is the pitch axis, roll is +90° upright.
[[nodiscard]] constexpr float pitch_from_imu(const damp::Quaternion<float>& q) {
    return q.to_euler().roll() - damp::numbers::pi_v<float> / 2.0f - kPitchTrim;
}

/// The pitch and yaw laws, the command filters and the references they track.
class Balancer {
   public:
    /// Hold the pose the bot is in: references at the measurement, integrators and filters cleared.
    constexpr void engage(const Measurement& m) {
        pitch_.reset();
#if BOT_PITCH_LAW != 0
        pitch_.disturbance_lim = kDisturbanceLim;
        restart(filter(), m.pitch);
        p0_ = position(m);
#endif
        yaw_.reset();
        speed_cmd_.reset();
        yaw_cmd_.reset();
        p_ref_   = position(m);
        psi_ref_ = heading(m);
    }

    constexpr WheelTorques step(const Measurement& m, const Command& cmd) {
        constexpr float r  = (float)(kBot.wheel_radius);
        constexpr float d  = (float)(kBot.track_width);
        constexpr float Ts = (float)(kTs);

        const float p        = position(m);
        const float v        = r * (0.5f * (m.wheel_rate_r + m.wheel_rate_l) + m.pitch_rate);
        const float psi      = heading(m);
        const float psi_rate = r * (m.wheel_rate_r - m.wheel_rate_l) / d;

        // References ride the filtered commands; a push drags them at most a leash behind.
        const float speed    = speed_cmd_(cmd.speed);
        const float yaw_rate = yaw_cmd_(cmd.yaw_rate);
        p_ref_               = damp::clamp(p_ref_ + speed * Ts, p - kLeash, p + kLeash);
        psi_ref_             = damp::clamp(psi_ref_ + yaw_rate * Ts, psi - kYawLeash, psi + kYawLeash);

#if BOT_PITCH_LAW == 1
        pitch_.update({p - p0_, v, m.pitch, m.pitch_rate});
        yaw_.update({psi, psi_rate});
        const float tau  = pitch_.control(0.0f, {p_ref_ - p0_, speed}, {})(0);
        const float dtau = yaw_.control(psi_ref_, {psi_ref_, yaw_rate}, {})(0);
#elif BOT_PITCH_LAW == 2
        // In the frame riding the speed command: a standstill at the CoM's upright θ = d̂₁, the torque
        // cancelling d̂₀.
        pitch_.update({p - p_ref_, v - speed, m.pitch, m.pitch_rate});
        yaw_.update({psi, psi_rate});
        const auto d_hat = pitch_.disturbance_estimate();
        pitch_.set_input_target(ColVec<1, float>{-d_hat(0)});
        const float tau  = pitch_.control(ColVec<4, float>{0.0f, 0.0f, d_hat(1), 0.0f})(0);
        const float dtau = yaw_.control(psi_ref_, {psi_ref_, yaw_rate}, {})(0);
#else
        // The lean the p, v and integral terms ask for is capped at kLeanMax, the integral's share at
        // kIntegralLean: uncapped, a bot held off its reference leans on until it falls.
        constexpr auto& K        = kPitchDesign.K;  // [p, v, θ, ω, τ₋₁, τ₋₂, τ₋₃, ξ]
        constexpr float k_p      = (float)(K(0, 0));
        constexpr float k_v      = (float)(K(0, 1));
        constexpr float k_th     = damp::abs((float)(K(0, 2)));
        constexpr float k_xi     = (float)(K(0, 7));
        constexpr float xi_max   = k_th * kIntegralLean / damp::abs(k_xi);
        constexpr float lean_tau = k_th * kLeanMax;
        float&          xi       = pitch_.integral.xi(0);
        xi                       = damp::clamp(xi, -xi_max, xi_max);

        pitch_.update({p, v, m.pitch, m.pitch_rate});
        yaw_.update({psi, psi_rate});
        const float u_law = pitch_.control(p_ref_, {p_ref_, speed}, {})(0);
        const float outer = -((k_p * (p - p_ref_)) + (k_v * (v - speed)) + (k_xi * xi));
        float       tau   = u_law;  // untouched unless clamped: back_calculate reads any difference as a clamp
        if (damp::abs(outer) > lean_tau) {
            tau += damp::clamp(outer, -lean_tau, lean_tau) - outer;
        }
        const float dtau = yaw_.control(psi_ref_, {psi_ref_, yaw_rate}, {})(0);
#endif

        // Balance first: pitch takes what it needs, steering gets what is left on each wheel.
        constexpr float tau_max  = 2.0f * kWheelTorqueMax;
        const float     tau_sat  = damp::clamp(tau, -tau_max, tau_max);
        const float     dtau_max = tau_max - damp::abs(tau_sat);
        const float     dtau_sat = damp::clamp(dtau, -dtau_max, dtau_max);

        pitch_.back_calculate(tau_sat);
        yaw_.back_calculate(dtau_sat);
        pitch_.predict();
        yaw_.predict();

        return {.right = 0.5f * (tau_sat + dtau_sat), .left = 0.5f * (tau_sat - dtau_sat)};
    }

    /// [m] axle travel: the wheels against the body plus the body's lean.
    [[nodiscard]] static constexpr float position(const Measurement& m) {
        return (float)(kBot.wheel_radius) * (0.5f * (m.wheel_r + m.wheel_l) + m.pitch);
    }

    /// [rad] heading from odometry, counter-clockwise from above.
    [[nodiscard]] static constexpr float heading(const Measurement& m) {
        return (float)(kBot.wheel_radius / kBot.track_width) * (m.wheel_r - m.wheel_l);
    }

    /// The OFMPC's QP iterations on the last tick (0 for the other laws).
    [[nodiscard]] constexpr size_t qp_iterations() const {
#if BOT_PITCH_LAW == 2
        return pitch_.last_iterations();
#else
        return 0;
#endif
    }

   private:
#if BOT_PITCH_LAW == 0
    damp::Law<kPitchDesign, float> pitch_{};
#else
#if BOT_PITCH_LAW == 1
    damp::Law<kPitchOFLQG, float> pitch_{};
    using Filter = decltype(pitch_.estimator);
    constexpr Filter& filter() { return pitch_.estimator; }
#else
    damp::Law<kPitchOFMPC, float> pitch_{};
    using Filter = damp::remove_cvref_t<decltype(pitch_.estimator())>;
    constexpr Filter& filter() { return pitch_.estimator(); }
#endif
    float p0_ = 0.0f;  ///< [m] where it engaged: the OFLQG's p is from here

    /// Start the filter over at a standstill leaning θ, d̂ = 0. Its state, [p, v, θ, ω, τ₋₁, τ₋₂, τ₋₃,
    /// d₀, d₁], is held as a deviation from an anchor.
    template <typename F>
    static constexpr void restart(F& f, float theta) {
        ColVec<9, float> x{};
        for (size_t i = 0; i < 7; ++i) {
            x(i) = -f.anchor_state()(i);
        }
        x(2) += theta;
        for (size_t i = 0; i < 2; ++i) {
            x(7 + i) = -f.anchor_disturbance()(i);
        }
        f.reset(x);
    }
#endif
    damp::Law<kYawDesign, float> yaw_{};
    damp::LowPass<1, float>      speed_cmd_{kCommandCutoff, (float)(kTs)};
    damp::LowPass<1, float>      yaw_cmd_{kCommandCutoff, (float)(kTs)};
    float                        p_ref_   = 0.0f;
    float                        psi_ref_ = 0.0f;
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
        const bool  drives = h.drives && finite(m);
        switch (mode_) {
            case Mode::Idle:
                if (upright_(tilt < kEngageTilt && h.imu && drives, (float)(kTs))) {
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

    [[nodiscard]] constexpr size_t qp_iterations() const { return balancer_.qp_iterations(); }

   private:
    Mode                  mode_ = Mode::Idle;
    damp::plc::TON<float> upright_{.PT = kEngageTime};
    Balancer              balancer_{};
};

}  // namespace bot
