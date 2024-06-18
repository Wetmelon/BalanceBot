#pragma once

#include <ServoInput.h>

#include <initializer_list>

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

    float update(bool enable, float vel_target, float vel_meas, float pitch_meas, float pitch_rate_meas) {
        // Velocity controller outputs a pitch command
        // [deg/(m/s)] - body pitch / CoM velocity error
        pitch_cmd = -1.0f * vel_controller.update(enable, vel_target, vel_meas, settings.Ts);

        // Pitch controller outputs a pitch rate command
        // [(deg/s) / deg] - body pitch rate / body pitch error
        pitch_rate_cmd = pitch_controller.update(enable, pitch_cmd, pitch_meas, settings.Ts);

        // Pitch rate controller outputs wheel torque / body pitch rate error.
        // J is a placeholder for inertia factors, leave at 1 for now
        // [Nm/(deg/s)] - wheel torque / body pitch rate error
        const float torque_cmd = pitch_rate_controller.update(enable, pitch_rate_cmd, pitch_rate_meas, settings.Ts) * settings.J;

        return torque_cmd;  //+ Tboost_lpfstate;
    }

    Settings_t settings = {
        .vel_controller        = vel_controller.settings,
        .pitch_controller      = pitch_controller.settings,
        .pitch_rate_controller = pitch_rate_controller.settings,

        .J  = 1.0f,  // [placeholder for inertia factors, leave at 1 for now]
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
        const float kJ;              // [placeholder for inertia factors, leave at 1 for now]

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

    void step(float drive_cmd_raw, float steer_cmd_raw) {
        // Blink orange LED at 1 sec
        bot::blink(1000);

        // Run State Machine
        state = run_state_machine(state);

        // Only run our controllers if we're in the active state
        const bool enable = (state == State::Active);

        // Get forward and left/right commands from joystick
        const CmdPair rx         = {drive_cmd_raw, steer_cmd_raw};
        const CmdPair mapped_cmd = filterConditionCmds(rx);

        const float drive_cmd = mapped_cmd.drive;
        const float steer_cmd = mapped_cmd.steer;

        // Motor speeds
        const float vel_right = +1.0f * bot_can.right_motor.get_encoder_estimates_msg.Vel_Estimate * (settings.kWheelDiameter * bot::kPi);  // [m/s] right wheel speed
        const float vel_left  = -1.0f * bot_can.left_motor.get_encoder_estimates_msg.Vel_Estimate * (settings.kWheelDiameter * bot::kPi);   // [m/s] left wheel speed

        // TODO:  Verify yaw rate calculation matches gyro reading
        const float vel_actual = (vel_right + vel_left) / 2.0f - (d2r(imu.pitch_rate) * settings.kComHeight);  // [m/s] Vehicle speed
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

        Serial.print("\t d cmd raw: ");
        Serial.print(drive_cmd_raw);
        Serial.print("\t s cmd raw: ");
        Serial.print(steer_cmd_raw);

        // Serial.print("\t d cmd: ");
        // Serial.print(drive_cmd);
        // Serial.print("\t s cmd: ");
        // Serial.print(steer_cmd);

        // Serial.print("Pitch: ");
        // Serial.print(imu.pitch);

        // Serial.print("\t P Rate: ");
        // Serial.print(imu.pitch_rate);

        // Serial.print("\t T: ");
        // Serial.print(drive_torque);

        // Serial.print("\t Vl: ");
        // Serial.print(vel_left);

        // Serial.print("\t Vr: ");
        // Serial.print(vel_right);

        // Serial.print("\t Vel: ");
        // Serial.print(vel_actual);

        Serial.println();
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
                bool pitch_over  = fabsf(imu.pitch) > 60.0f;
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

    // Commands assumed to be in range [-1, 1]
    // Outputs scaled to robot units [m/s, rad/s]
    CmdPair filterConditionCmds(const CmdPair& rx) {
        // Speed and turn rate joystick range TODO: move to settings
        float drive_range = 2.0f;  // [m/s]
        float steer_range = 3.0f;  // [rad/s]

        // Deadband then,
        // "expo curve" 2nd order map (quadratic) (with abs to keep sign)
        const CmdPair mapped_cmd = {
            .drive = bot::remap_joystick(rx.drive) * drive_range,
            .steer = bot::remap_joystick(rx.steer) * steer_range,
        };

        // Lowpass filters (tau set in LPF constructor)
        return {
            drive_lpf.step(mapped_cmd.drive),
            steer_lpf.step(mapped_cmd.steer),
        };
    }

    Settings_t settings{
        .kWheelDiameter = 0.0900f,
        .kComHeight     = 0.065f,
        .kTrackWidth    = 0.171f,
        .kJ             = 1.0f,
        .balancing      = balancing.settings,
        .steering       = steering.settings,
    };

   private:
    bot::LPF   drive_lpf{bot::kControlLoopPeriod, 0.1f};
    bot::LPF   steer_lpf{bot::kControlLoopPeriod, 0.1f};
    bot::Timer vertical_timer{2000};

    State             state{};
    BalanceController balancing{};
    PIDController     steering{};
};

extern BotController controller;
