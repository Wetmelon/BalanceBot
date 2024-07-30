#pragma once

#include "bot_can.hpp"
#include "imu_wrapper.hpp"
#include "utils.hpp"

struct ReactionWheelController {

    struct Settings_t {
        const float kDefaultGain = 10.0f/360.0f; // [Nm / deg/s]
        const float kDefaultIntegratorGainFactor = 2.0f; // [1/s]
        const float kDefaultSlip = 5.0f/10.0f; // [deg/s(body) / rev/s(wheel)]
    } settings;

    void begin() {
        bot_can.setAxisStates(AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    void step (float in0, float in1) {
        bot::blink(1000);

        auto odrv_error = bot_can.left_motor.heartbeat_msg.Axis_Error; 
        if (odrv_error != 0) Serial.println(odrv_error);

        float Gain = settings.kDefaultGain;
        float IntegratorGainFactor = settings.kDefaultIntegratorGainFactor;
        float Slip = settings.kDefaultSlip;

        // TODO: subtract gyro bias if needed
        // Positive roll rate is rotation around body +x axis, which is forward
        float roll_rate = imu.pitch_rate; // lol orientation config somewhere
        // Positive wheel speed and torque is rotation around body +x axis
        float wheel_speed = +bot_can.left_motor.get_encoder_estimates_msg.Vel_Estimate; // [rev/s]

        float roll_rate_setpoint = Slip * wheel_speed; // [deg/s]
        float roll_rate_error = roll_rate_setpoint - roll_rate; // [deg/s]
        float torque_cmd = -Gain * roll_rate_error; // [Nm]
        
        static float roll_rate_error_integrator = 0.0f;
        roll_rate_error_integrator += roll_rate_error * 0.01f; // [deg/s * s = deg]
        roll_rate_error_integrator = bot::clamp(roll_rate_error_integrator, -10.0f, 10.0f); // [deg]
        torque_cmd += IntegratorGainFactor * -Gain * roll_rate_error_integrator; // [Nm]

        bot_can.left_motor.set_input_torque_msg.Input_Torque = torque_cmd;

        // Serial.print("\t R rate: ");
        // Serial.print(roll_rate);

        // Serial.print("\t R rate sp: ");
        // Serial.print(roll_rate_setpoint);

        // Serial.print("\t T: ");
        Serial.print(torque_cmd);

        Serial.println();
    }

}; 



// {
//     "config.dc_bus_overvoltage_trip_level": 30,
//     "config.dc_bus_undervoltage_trip_level": 19.2,
//     "config.dc_max_positive_current": 10,
//     "config.dc_max_negative_current": -10,
//     "axis0.config.motor.motor_type": 0,
//     "axis0.config.motor.torque_constant": 0.15036363636363637,
//     "axis0.config.motor.pole_pairs": 14,
//     "axis0.config.motor.current_soft_max": 5,
//     "axis0.config.motor.current_hard_max": 7,
//     "axis0.config.motor.calibration_current": 3,
//     "axis0.config.motor.resistance_calib_max_voltage": 5,
//     "axis0.config.calibration_lockin.current": 3,
//     "axis0.controller.config.control_mode": 1,
//     "axis0.controller.config.input_mode": 1,
//     "axis0.controller.config.vel_ramp_rate": 50,
//     "axis0.controller.config.vel_limit": 17.16,
//     "axis0.controller.config.vel_limit_tolerance": 1.4285714285714286,
//     "axis0.config.torque_soft_min": -10,
//     "axis0.config.torque_soft_max": 10,
//     "axis0.config.load_encoder": 13,
//     "axis0.config.commutation_encoder": 13,
//     "axis0.controller.config.vel_gain":0.5
//     "axis0.controller.config.vel_integrator_gain":0.03
// }