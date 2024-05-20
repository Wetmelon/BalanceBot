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
    bot_can.left_motor.set_limits_msg.Velocity_Limit = 10.0f; // Not used!
    bot_can.left_motor.set_limits_msg.Current_Limit  = 3.0f;  // Not used!

    bot_can.right_motor.set_limits_msg.Velocity_Limit = 10.0f; // Not used!
    bot_can.right_motor.set_limits_msg.Current_Limit  = 3.0f;  // Not used!

    // Vel limit of micros was set to 13.585 rps. Use around 13 rps for margin.
    // Vel limit for 90mm dia wheels: 3.67 m/s

    // Balancing controller settings
    controller.settings.balancing.vel_controller.Kp         = 70.0f;  // [deg/(m/s)] - body pitch / CoM velocity error
    controller.settings.balancing.vel_controller.Ki         = 30.0f;
    controller.settings.balancing.vel_controller.iterm_min  = -6.0f;   // [deg]
    controller.settings.balancing.vel_controller.iterm_max  = 6.0f;    // [deg]
    controller.settings.balancing.vel_controller.output_min = -40.0f;  // [deg]
    controller.settings.balancing.vel_controller.output_max = 40.0f;   // [deg]

    controller.settings.balancing.pitch_controller.Kp         = 8.0f;  // [(deg/s) / deg] - body pitch rate / body pitch error
    controller.settings.balancing.pitch_controller.Ki         = 0.0f;
    controller.settings.balancing.pitch_controller.iterm_min  = -100.0f;  // [deg/s]
    controller.settings.balancing.pitch_controller.iterm_max  = 100.0f;   // [deg/s]
    controller.settings.balancing.pitch_controller.output_min = -800.0f;  // [deg/s]
    controller.settings.balancing.pitch_controller.output_max = 800.0f;   // [deg/s]

    controller.settings.balancing.pitch_rate_controller.Kp         = 0.0016f;  // [Nm/(deg/s)] - wheel torque / body pitch rate error
    controller.settings.balancing.pitch_rate_controller.Ki         = 0.0f;
    controller.settings.balancing.pitch_rate_controller.output_min = -1.5f;  // [Nm]
    controller.settings.balancing.pitch_rate_controller.output_max = 1.5f;   // [Nm]

    controller.settings.balancing.J  = 1.0f;  // [placeholder for inertia factors, lave at 1 for now]
    controller.settings.balancing.Ts = bot::kControlLoopPeriod;

    // // Yaw control settings
    controller.settings.steering.Kp         = 0.2f;
    controller.settings.steering.Ki         = 0.4f;
    controller.settings.steering.iterm_min  = -0.3f;  // [Nm]
    controller.settings.steering.iterm_max  = 0.3f;  // [Nm]
    controller.settings.steering.output_min = -0.5f;  // [Nm]
    controller.settings.steering.output_max = 0.5f;  // [Nm]
}
