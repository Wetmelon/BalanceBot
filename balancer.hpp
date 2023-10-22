#pragma once

#include "pid.hpp"

namespace odrv {

struct BalanceControl {
    struct Inputs_t {
        float vel_target;
        float vel;
        float pitch;
        float pitch_rate;
        bool  enable;
    };

    struct Outputs_t {
        float torque_cmd;
    };

    struct Settings_t {
        PIDController::Settings_t& vel_controller;
        PIDController::Settings_t& pitch_controller;
        PIDController::Settings_t& pitch_rate_controller;

        float J;
        float Ts;
    };

    Inputs_t inputs;

    Settings_t settings = {
        .vel_controller        = vel_controller.settings,
        .pitch_controller      = pitch_controller.settings,
        .pitch_rate_controller = pitch_rate_controller.settings,

        .J  = 1.0f,
        .Ts = 0.01f,
    };

    const Outputs_t& getOutputs() {
        return outputs;
    }

    void update() {
        // Pitch rate controller outputs an acceleration, and Tau = J * alpha

        const float pitch_cmd      = vel_controller.update(inputs.enable, inputs.vel_target, inputs.vel, settings.Ts);
        const float pitch_rate_cmd = pitch_controller.update(inputs.enable, pitch_cmd, inputs.pitch, settings.Ts);
        const float torque_cmd     = pitch_rate_controller.update(inputs.enable, pitch_rate_cmd, inputs.pitch_rate, settings.Ts) * settings.J;

        outputs.torque_cmd = -1.0f * torque_cmd;
    }

   private:
    Outputs_t outputs;

    PIDController vel_controller;
    PIDController pitch_controller;
    PIDController pitch_rate_controller;
};

};  // namespace odrv
