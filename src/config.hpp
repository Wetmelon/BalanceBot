#pragma once

#include "balancer.hpp"
#include "bot_can.hpp"
#include "can_simple_messages.hpp"
#include "utils.hpp"

void configControllers() {
    // Setup our desired control and input modes
    bot_can.left_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    bot_can.left_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

    bot_can.right_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    bot_can.right_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

    // Setup the velocity and current limits that we'll use
    bot_can.left_motor.set_limits_msg.Velocity_Limit = 10.0f;
    bot_can.left_motor.set_limits_msg.Current_Limit  = 3.0f;

    bot_can.right_motor.set_limits_msg.Velocity_Limit = 10.0f;
    bot_can.right_motor.set_limits_msg.Current_Limit  = 3.0f;

    // Balancing controller settings
    controller.settings.balancing.vel_controller.Kp         = 50.0f;  // [deg/(m/s)] - body pitch / CoM velocity error
    controller.settings.balancing.vel_controller.Ki         = 10.0f;
    controller.settings.balancing.vel_controller.iterm_min  = -6.0f;   // [deg]
    controller.settings.balancing.vel_controller.iterm_max  = 6.0f;    // [deg]
    controller.settings.balancing.vel_controller.output_min = -20.0f;  // [deg]
    controller.settings.balancing.vel_controller.output_max = 20.0f;   // [deg]

    controller.settings.balancing.pitch_controller.Kp         = 8.0f;  // [(deg/s) / deg] - body pitch rate / body pitch error
    controller.settings.balancing.pitch_controller.Ki         = 0.0f;
    controller.settings.balancing.pitch_controller.iterm_min  = -10.0f;   // [deg/s]
    controller.settings.balancing.pitch_controller.iterm_max  = 10.0f;    // [deg/s]
    controller.settings.balancing.pitch_controller.output_min = -100.0f;  // [deg/s]
    controller.settings.balancing.pitch_controller.output_max = 100.0f;   // [deg/s]

    controller.settings.balancing.pitch_rate_controller.Kp         = 0.0016f;  // [Nm/(deg/s)] - wheel torque / body pitch rate error
    controller.settings.balancing.pitch_rate_controller.Ki         = 0.0f;
    controller.settings.balancing.pitch_rate_controller.output_min = -1.0f;  // [Nm]
    controller.settings.balancing.pitch_rate_controller.output_max = 1.0f;   // [Nm]

    controller.settings.balancing.J  = 1.0f;  // [placeholder for inertia factors, lave at 1 for now]
    controller.settings.balancing.Ts = bot::kControlLoopPeriod;

    // // Yaw control settings
    controller.settings.steering.Kp         = 0.1f;
    controller.settings.steering.Ki         = 0.0f;
    controller.settings.steering.iterm_min  = -1.0f;  // [Nm]
    controller.settings.steering.iterm_max  = 1.0f;  // [Nm]
    controller.settings.steering.output_min = -0.5f;  // [Nm]
    controller.settings.steering.output_max = 0.5f;  // [Nm]
}
