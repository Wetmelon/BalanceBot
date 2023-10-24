#pragma once

#include "pid.hpp"

namespace odrv {

struct BalanceControl {
    struct Settings_t {
        PIDController::Settings_t& vel_controller;
        PIDController::Settings_t& pitch_controller;
        PIDController::Settings_t& pitch_rate_controller;

        float J;
        float Ts;
    };

    float update(float vel_target, float vel, float pitch, float pitch_rate, bool enable) {
        // Velocity controller outputs a pitch command
        const float pitch_cmd = vel_controller.update(enable, vel_target, vel, settings.Ts);

        // Pitch controller outputs a pitch rate command
        const float pitch_rate_cmd = pitch_controller.update(enable, pitch_cmd, pitch, settings.Ts);

        // Pitch rate controller outputs an acceleration, and Tau = J * alpha
        const float torque_cmd = -1.0f * pitch_rate_controller.update(enable, pitch_rate_cmd, pitch_rate, settings.Ts) * settings.J;

        return torque_cmd;
    }

    Settings_t settings = {
        .vel_controller        = vel_controller.settings,
        .pitch_controller      = pitch_controller.settings,
        .pitch_rate_controller = pitch_rate_controller.settings,

        .J  = 1.0f,
        .Ts = 0.01f,
    };

   private:
    PIDController vel_controller{};
    PIDController pitch_controller{};
    PIDController pitch_rate_controller{};
};

};  // namespace odrv
