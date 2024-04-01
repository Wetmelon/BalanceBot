#pragma once

#include "bot_can.hpp"
#include "imu_wrapper.hpp"
#include "pid.hpp"
#include "utils.hpp"

struct BalanceController {
    struct Settings_t {
        PIDController::Settings_t& vel_controller;
        PIDController::Settings_t& pitch_controller;
        PIDController::Settings_t& pitch_rate_controller;

        float J;
        float Ts;
    };

    float update(bool enable, float vel_target, float vel, float pitch, float pitch_rate) {
        // Velocity controller outputs a pitch command
        pitch_cmd = vel_controller.update(enable, vel_target, vel, settings.Ts);

        // Pitch controller outputs a pitch rate command
        pitch_rate_cmd = pitch_controller.update(enable, pitch_cmd, pitch, settings.Ts);

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

    float pitch_cmd      = 0.0f;
    float pitch_rate_cmd = 0.0f;

   private:
    PIDController vel_controller{};
    PIDController pitch_controller{};
    PIDController pitch_rate_controller{};
};

struct BotController {
    struct Settings_t {
        // Vehicle parameters
        const float kWheelDiameter;  // [m] Wheel Diameter
        const float kComHeight;      // [m] Height of center of mass from floor
        const float kTrackWidth;     // [m] Distance between wheels
        const float kJ;              // [Nm/(rad/s^2)] Rotational inertia

        BalanceController::Settings_t& balancing;
        PIDController::Settings_t&     steering;
    };

    struct CmdPair {
        float drive;
        float steer;
    };

    enum class State {
        Idle,
        Active,
        Error
    };

    void begin() {
        vertical_timer.reset();
    }

    void step() {
        // Blink orange LED at 1 sec
        bot::blink(1000);

        // Run State Machine
        state = run_state_machine(state);

        // Only run our controllers if we're in the active state
        const bool enable = (state == State::Active);

        // Get forward and left/right commands from joystick
        CmdPair rx = getControllerCmds(enable);

        // Yaw-priority ISO pattern mixer (maintains 100% wheel speed differential at 100% joystick steer position)
        const float steer_cmd = 0.0f;  // rx.steer * 0.5f;
        const float drive_cmd = 0.0f;  // rx.drive * (1.0f - abs(steer_cmd));

        // Motor speeds
        const float vel_right = bot_can.right_motor.get_encoder_estimates_msg.Vel_Estimate * (settings.kWheelDiameter * bot::kPi);         // [m/s] right wheel speed
        const float vel_left  = -1.0f * bot_can.left_motor.get_encoder_estimates_msg.Vel_Estimate * (settings.kWheelDiameter * bot::kPi);  // [m/s] left wheel speed

        // TODO:  Verify yaw rate calculation matches gyro reading
        const float vel_actual = (vel_right + vel_left) / 2.0f + (d2r(imu.pitch_rate) * settings.kComHeight);  // [m/s] Vehicle speed
        const float yaw_rate   = (vel_right - vel_left) / (2.0f * settings.kTrackWidth);                       // [rad/s] Vehicle yaw rate, per motor sensors

        // Run balancing and steering controllers
        const float drive_torque = balancing.update(enable, drive_cmd, vel_actual, imu.pitch, imu.pitch_rate);
        const float steer_torque = steering.update(enable, steer_cmd, yaw_rate, 0.01f);

        // Convert from drive/steer to left/right motor torques
        bot_can.right_motor.set_input_torque_msg.Input_Torque = +0.5f * (drive_torque + steer_torque);
        bot_can.left_motor.set_input_torque_msg.Input_Torque  = -0.5f * (drive_torque - steer_torque);

        // Set commands to 0 if we're not in the active state
        if (state != State::Active) {
            bot_can.right_motor.set_input_torque_msg.Input_Torque = 0.0f;
            bot_can.left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
        }

        Serial.print("Pitch: ");
        Serial.print(imu.pitch);

        Serial.print("\tVel: ");
        Serial.println(vel_actual);
    }

    State run_state_machine(State state) {
        State next_state = state;

        switch (state) {
            case State::Idle: {
                // If pitch is within 5 degrees for 2 seconds, enable motors
                if (fabsf(imu.pitch) > 5.0f) {
                    vertical_timer.reset();
                } else if (vertical_timer.isExpired() && (!imu.getIsTimedOut())) {
                    bot_can.setAxisStates(AXIS_STATE_CLOSED_LOOP_CONTROL);
                    next_state = State::Active;
                } else {
                    bot_can.setAxisStates(AXIS_STATE_IDLE);
                }

            } break;

            case State::Active: {
                // Check for errors
                bool pitch_over  = fabsf(imu.pitch) > 30.0f;
                bool imu_timeout = imu.getIsTimedOut();

                bool left_error  = bot_can.left_motor.heartbeat_msg.Axis_Error != 0;
                bool right_error = bot_can.right_motor.heartbeat_msg.Axis_Error != 0;

                if (pitch_over || left_error || right_error) {
                    vertical_timer.reset();

                    bot_can.setAxisStates(AXIS_STATE_IDLE);
                    next_state = State::Idle;
                }

                if (imu_timeout) {
                    bot_can.setAxisStates(AXIS_STATE_IDLE);
                    next_state = State::Error;
                }

            } break;

            case State::Error:
            default: {
                bot_can.setAxisStates(AXIS_STATE_IDLE);
            } break;
        }

        return next_state;
    }

    CmdPair filterCmds(const CmdPair& rx) {
        // Lowpass filters
        return {
            drive_lpf.step(rx.drive),
            steer_lpf.step(rx.steer),
        };
    }

    CmdPair getControllerCmds(bool enable) {
        // TODO:  Read controller
        (void)enable;

        CmdPair rx{};
        return filterCmds(rx);
    }

    Settings_t settings{
        .kWheelDiameter = 0.1524f,
        .kComHeight     = 0.18f,
        .kTrackWidth    = 1.0f,
        .kJ             = 1.0f,
        .balancing      = balancing.settings,
        .steering       = steering.settings,
    };

   private:
    bot::LPF   drive_lpf{bot::kControlLoopPeriod, 1.0f};
    bot::LPF   steer_lpf{bot::kControlLoopPeriod, 1.0f};
    bot::Timer vertical_timer{2000};

    State             state{};
    BalanceController balancing{};
    PIDController     steering{};
};

extern BotController controller;
