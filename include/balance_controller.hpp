/**
 * @file balance_controller.hpp
 * @brief BalanceBot deploy header: the limits and the control tick.
 *
 * No Arduino here. main.cpp flashes it; sil/balance_sil.cpp closes the loop on the host around
 * the same objects. The nameplate, plants, weights and designs are balance_design.hpp.
 */

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

inline constexpr float kWheelTorqueMax = 8.0f;  ///< [N·m] per wheel, the clamp on the law (traction alone holds about 2: μ ≈ 0.9 × m·g/2 × r)

// The drives' own limits, a backstop under the clamp above: the controller never asks for more than
// kWheelTorqueMax (about 8 A), so these only act if it misbehaves. Sent at each engage (bot_can.hpp).
inline constexpr float kDriveVelocityLimit = 10.0f;  ///< [rev/s] 5.2 m/s: a runaway guard, past anything the bot asks
inline constexpr float kDriveCurrentLimit  = 80.0f;  ///< [A] the S1's own maximum (config.inverter0.current_soft_max)

inline constexpr float kEngageTilt    = 0.087f;  ///< [rad] 5°: held this upright ...
inline constexpr float kEngageTime    = 2.0f;    ///< [s] ... this long, the bot engages
inline constexpr float kTipTilt       = 0.52f;   ///< [rad] 30°: past this it has fallen, motors off
inline constexpr float kLeash         = 0.3f;    ///< [m] position error the reference may lag a push by
inline constexpr float kYawLeash      = 0.5f;    ///< [rad] heading error the reference may lag by
inline constexpr float kLeanMax       = 0.26f;   ///< [rad] 15°: the most lean position, speed and the integral may ask for
inline constexpr float kIntegralLean  = 0.14f;   ///< [rad] 8° of it the integral may hold: a CoM offset, an IMU error, a slope
inline constexpr float kCommandCutoff = 0.5f;    ///< [Hz] low-pass on the speed and yaw-rate commands

/// [rad] IMU pitch reading with the CoM over the axle. Calibrate: balance it by hand, read the telemetry,
/// or fit the drop test's falls both ways (here: +0.004 rad).
inline constexpr float kPitchTrim = 0.004f;

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
        p_ref_   = position(m);
        psi_ref_ = heading(m);
    }

    /// One period: update → control → clamp → back_calculate → predict.
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

        // Read as a cascade, u = −K_θ·(θ − θ_c) − …: the lean θ_c is what p, v and the integral ask for.
        // The integral holds at most kIntegralLean of it and θ_c at most kLeanMax, or a bot held off
        // its reference (a wall, a hand) leans on until it falls. back_calculate sees the clamp, so the
        // integral stops winding against it.
        constexpr auto& K        = kPitchDesign.K;  // [p, v, θ, ω, τ₋₁, τ₋₂, τ₋₃, ξ]
        constexpr float k_p      = (float)(K(0, 0));
        constexpr float k_v      = (float)(K(0, 1));
        constexpr float k_th     = damp::abs((float)(K(0, 2)));
        constexpr float k_xi     = (float)(K(0, 7));
        constexpr float xi_max   = k_th * kIntegralLean / damp::abs(k_xi);
        constexpr float lean_tau = k_th * kLeanMax;  // [N·m] the torque that asks for kLeanMax
        float&          xi       = pitch_.integral.xi(0);
        xi                       = damp::clamp(xi, -xi_max, xi_max);

        // The pitch law holds the torques still on their way to the wheels: what back_calculate recorded.
        pitch_.update({p, v, m.pitch, m.pitch_rate});
        yaw_.update({psi, psi_rate});
        const float u_law = pitch_.control(p_ref_, {p_ref_, speed}, {})(0);
        const float outer = -((k_p * (p - p_ref_)) + (k_v * (v - speed)) + (k_xi * xi));
        float       tau   = u_law;  // untouched unless clamped: back_calculate reads any difference as a clamp
        if (damp::abs(outer) > lean_tau) {
            tau += damp::clamp(outer, -lean_tau, lean_tau) - outer;
        }
        const float dtau = yaw_.control(psi_ref_, {psi_ref_, yaw_rate}, {})(0);

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

    /// [m] axle travel: the wheels' turn against the ground is the encoder plus the body's lean.
    [[nodiscard]] static constexpr float position(const Measurement& m) {
        return (float)(kBot.wheel_radius) * (0.5f * (m.wheel_r + m.wheel_l) + m.pitch);
    }

    /// [rad] heading from odometry, counter-clockwise from above.
    [[nodiscard]] static constexpr float heading(const Measurement& m) {
        return (float)(kBot.wheel_radius / kBot.track_width) * (m.wheel_r - m.wheel_l);
    }

   private:
    damp::Law<kPitchDesign, float> pitch_{};
    damp::Law<kYawDesign, float>   yaw_{};
    damp::LowPass<1, float>        speed_cmd_{kCommandCutoff, (float)(kTs)};
    damp::LowPass<1, float>        yaw_cmd_{kCommandCutoff, (float)(kTs)};
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

    [[nodiscard]] constexpr Mode mode() const { return mode_; }

   private:
    Mode                  mode_ = Mode::Idle;
    damp::plc::TON<float> upright_{.PT = kEngageTime};
    Balancer              balancer_{};
};

}  // namespace bot
