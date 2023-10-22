
#pragma once

#include <cstdint>

#include "can_helpers.hpp"

struct Get_Version_msg_t final {
    uint8_t Protocol_Version      = 0;
    uint8_t Hw_Version_Major      = 0;
    uint8_t Hw_Version_Minor      = 0;
    uint8_t Hw_Version_Variant    = 0;
    uint8_t Fw_Version_Major      = 0;
    uint8_t Fw_Version_Minor      = 0;
    uint8_t Fw_Version_Revision   = 0;
    uint8_t Fw_Version_Unreleased = 0;

    Get_Version_msg_t() = default;

    Get_Version_msg_t(const can_Message_t& msg) {
        Protocol_Version      = can_getSignal<uint8_t>(msg.data, 0, 8, true);
        Hw_Version_Major      = can_getSignal<uint8_t>(msg.data, 8, 8, true);
        Hw_Version_Minor      = can_getSignal<uint8_t>(msg.data, 16, 8, true);
        Hw_Version_Variant    = can_getSignal<uint8_t>(msg.data, 24, 8, true);
        Fw_Version_Major      = can_getSignal<uint8_t>(msg.data, 32, 8, true);
        Fw_Version_Minor      = can_getSignal<uint8_t>(msg.data, 40, 8, true);
        Fw_Version_Revision   = can_getSignal<uint8_t>(msg.data, 48, 8, true);
        Fw_Version_Unreleased = can_getSignal<uint8_t>(msg.data, 56, 8, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint8_t>(msg.data, Protocol_Version, 0, 8, true);
        can_setSignal<uint8_t>(msg.data, Hw_Version_Major, 8, 8, true);
        can_setSignal<uint8_t>(msg.data, Hw_Version_Minor, 16, 8, true);
        can_setSignal<uint8_t>(msg.data, Hw_Version_Variant, 24, 8, true);
        can_setSignal<uint8_t>(msg.data, Fw_Version_Major, 32, 8, true);
        can_setSignal<uint8_t>(msg.data, Fw_Version_Minor, 40, 8, true);
        can_setSignal<uint8_t>(msg.data, Fw_Version_Revision, 48, 8, true);
        can_setSignal<uint8_t>(msg.data, Fw_Version_Unreleased, 56, 8, true);
    }

    void decode(const can_Message_t& msg) {
        Protocol_Version      = can_getSignal<uint8_t>(msg.data, 0, 8, true);
        Hw_Version_Major      = can_getSignal<uint8_t>(msg.data, 8, 8, true);
        Hw_Version_Minor      = can_getSignal<uint8_t>(msg.data, 16, 8, true);
        Hw_Version_Variant    = can_getSignal<uint8_t>(msg.data, 24, 8, true);
        Fw_Version_Major      = can_getSignal<uint8_t>(msg.data, 32, 8, true);
        Fw_Version_Minor      = can_getSignal<uint8_t>(msg.data, 40, 8, true);
        Fw_Version_Revision   = can_getSignal<uint8_t>(msg.data, 48, 8, true);
        Fw_Version_Unreleased = can_getSignal<uint8_t>(msg.data, 56, 8, true);
    }

    static constexpr uint8_t cmd_id     = 0x000;
    static constexpr uint8_t msg_length = 8;
};

struct Heartbeat_msg_t final {
    uint32_t Axis_Error           = 0;
    uint8_t  Axis_State           = 0;
    uint8_t  Procedure_Result     = 0;
    uint8_t  Trajectory_Done_Flag = 0;

    Heartbeat_msg_t() = default;

    Heartbeat_msg_t(const can_Message_t& msg) {
        Axis_Error           = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Axis_State           = can_getSignal<uint8_t>(msg.data, 32, 8, true);
        Procedure_Result     = can_getSignal<uint8_t>(msg.data, 40, 8, true);
        Trajectory_Done_Flag = can_getSignal<uint8_t>(msg.data, 48, 1, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Axis_Error, 0, 32, true);
        can_setSignal<uint8_t>(msg.data, Axis_State, 32, 8, true);
        can_setSignal<uint8_t>(msg.data, Procedure_Result, 40, 8, true);
        can_setSignal<uint8_t>(msg.data, Trajectory_Done_Flag, 48, 1, true);
    }

    void decode(const can_Message_t& msg) {
        Axis_Error           = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Axis_State           = can_getSignal<uint8_t>(msg.data, 32, 8, true);
        Procedure_Result     = can_getSignal<uint8_t>(msg.data, 40, 8, true);
        Trajectory_Done_Flag = can_getSignal<uint8_t>(msg.data, 48, 1, true);
    }

    static constexpr uint8_t cmd_id     = 0x001;
    static constexpr uint8_t msg_length = 8;
};

struct Estop_msg_t final {
    Estop_msg_t() = default;

    Estop_msg_t(const can_Message_t& msg) {
    }

    void encode(can_Message_t& msg) const {
    }

    void decode(const can_Message_t& msg) {
    }

    static constexpr uint8_t cmd_id     = 0x002;
    static constexpr uint8_t msg_length = 0;
};

struct Get_Error_msg_t final {
    uint32_t Active_Errors = 0;
    uint32_t Disarm_Reason = 0;

    Get_Error_msg_t() = default;

    Get_Error_msg_t(const can_Message_t& msg) {
        Active_Errors = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Disarm_Reason = can_getSignal<uint32_t>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Active_Errors, 0, 32, true);
        can_setSignal<uint32_t>(msg.data, Disarm_Reason, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Active_Errors = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Disarm_Reason = can_getSignal<uint32_t>(msg.data, 32, 32, true);
    }

    static constexpr uint8_t cmd_id     = 0x003;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Axis_Node_ID_msg_t final {
    uint32_t Axis_Node_ID = 0;

    Set_Axis_Node_ID_msg_t() = default;

    Set_Axis_Node_ID_msg_t(const can_Message_t& msg) {
        Axis_Node_ID = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Axis_Node_ID, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Axis_Node_ID = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    static constexpr uint8_t cmd_id     = 0x006;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Axis_State_msg_t final {
    uint32_t Axis_Requested_State = 0;

    Set_Axis_State_msg_t() = default;

    Set_Axis_State_msg_t(const can_Message_t& msg) {
        Axis_Requested_State = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Axis_Requested_State, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Axis_Requested_State = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    static constexpr uint8_t cmd_id     = 0x007;
    static constexpr uint8_t msg_length = 8;
};

struct Get_Encoder_Estimates_msg_t final {
    float Pos_Estimate = 0.0f;  // [rev]
    float Vel_Estimate = 0.0f;  // [rev/s]

    Get_Encoder_Estimates_msg_t() = default;

    Get_Encoder_Estimates_msg_t(const can_Message_t& msg) {
        Pos_Estimate = can_getSignal<float>(msg.data, 0, 32, true);
        Vel_Estimate = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Pos_Estimate, 0, 32, true);
        can_setSignal<float>(msg.data, Vel_Estimate, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Pos_Estimate = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Vel_Estimate = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x009;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Controller_Mode_msg_t final {
    uint32_t Control_Mode = 0;
    uint32_t Input_Mode   = 0;

    Set_Controller_Mode_msg_t() = default;

    Set_Controller_Mode_msg_t(const can_Message_t& msg) {
        Control_Mode = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Input_Mode   = can_getSignal<uint32_t>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Control_Mode, 0, 32, true);
        can_setSignal<uint32_t>(msg.data, Input_Mode, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Control_Mode = can_getSignal<uint32_t>(msg.data, 0, 32, true);
        Input_Mode   = can_getSignal<uint32_t>(msg.data, 32, 32, true);
    }

    static constexpr uint8_t cmd_id     = 0x00B;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Input_Pos_msg_t final {
    float Input_Pos = 0.0f;  // [rev]
    float Vel_FF    = 0.0f;  // [rev/s]
    float Torque_FF = 0.0f;  // [Nm]

    Set_Input_Pos_msg_t() = default;

    Set_Input_Pos_msg_t(const can_Message_t& msg) {
        Input_Pos = can_getSignal<float>(msg.data, 0, 32, true);
        Vel_FF    = can_getSignal<int16_t>(msg.data, 32, 16, true, 0.001f, 0.0f);
        Torque_FF = can_getSignal<int16_t>(msg.data, 48, 16, true, 0.001f, 0.0f);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Input_Pos, 0, 32, true);
        can_setSignal<int16_t>(msg.data, Vel_FF, 32, 16, true, 0.001f, 0.0f);
        can_setSignal<int16_t>(msg.data, Torque_FF, 48, 16, true, 0.001f, 0.0f);
    }

    void decode(const can_Message_t& msg) {
        Input_Pos = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Vel_FF    = can_getSignal<int16_t>(msg.data, 32, 16, true, 0.001f, 0.0f);
        Torque_FF = can_getSignal<int16_t>(msg.data, 48, 16, true, 0.001f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x00C;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Input_Vel_msg_t final {
    float Input_Vel       = 0.0f;  // [rev/s]
    float Input_Torque_FF = 0.0f;  // [Nm]

    Set_Input_Vel_msg_t() = default;

    Set_Input_Vel_msg_t(const can_Message_t& msg) {
        Input_Vel       = can_getSignal<float>(msg.data, 0, 32, true);
        Input_Torque_FF = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Input_Vel, 0, 32, true);
        can_setSignal<float>(msg.data, Input_Torque_FF, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Input_Vel       = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Input_Torque_FF = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x00D;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Input_Torque_msg_t final {
    float Input_Torque = 0.0f;  // [Nm]

    Set_Input_Torque_msg_t() = default;

    Set_Input_Torque_msg_t(const can_Message_t& msg) {
        Input_Torque = can_getSignal<float>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Input_Torque, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Input_Torque = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x00E;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Limits_msg_t final {
    float Velocity_Limit = 0.0f;  // [rev/s]
    float Current_Limit  = 0.0f;  // [A]

    Set_Limits_msg_t() = default;

    Set_Limits_msg_t(const can_Message_t& msg) {
        Velocity_Limit = can_getSignal<float>(msg.data, 0, 32, true);
        Current_Limit  = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Velocity_Limit, 0, 32, true);
        can_setSignal<float>(msg.data, Current_Limit, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Velocity_Limit = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Current_Limit  = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x00F;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Traj_Vel_Limit_msg_t final {
    float Traj_Vel_Limit = 0.0f;  // [rev/s]

    Set_Traj_Vel_Limit_msg_t() = default;

    Set_Traj_Vel_Limit_msg_t(const can_Message_t& msg) {
        Traj_Vel_Limit = can_getSignal<float>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Traj_Vel_Limit, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Traj_Vel_Limit = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x011;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Traj_Accel_Limits_msg_t final {
    float Traj_Accel_Limit = 0.0f;  // [rev/s^2]
    float Traj_Decel_Limit = 0.0f;  // [rev/s^2]

    Set_Traj_Accel_Limits_msg_t() = default;

    Set_Traj_Accel_Limits_msg_t(const can_Message_t& msg) {
        Traj_Accel_Limit = can_getSignal<float>(msg.data, 0, 32, true);
        Traj_Decel_Limit = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Traj_Accel_Limit, 0, 32, true);
        can_setSignal<float>(msg.data, Traj_Decel_Limit, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Traj_Accel_Limit = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Traj_Decel_Limit = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x012;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Traj_Inertia_msg_t final {
    float Traj_Inertia = 0.0f;  // [Nm/(rev/s^2)]

    Set_Traj_Inertia_msg_t() = default;

    Set_Traj_Inertia_msg_t(const can_Message_t& msg) {
        Traj_Inertia = can_getSignal<float>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Traj_Inertia, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Traj_Inertia = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x013;
    static constexpr uint8_t msg_length = 8;
};

struct Get_Iq_msg_t final {
    float Iq_Setpoint = 0.0f;  // [A]
    float Iq_Measured = 0.0f;  // [A]

    Get_Iq_msg_t() = default;

    Get_Iq_msg_t(const can_Message_t& msg) {
        Iq_Setpoint = can_getSignal<float>(msg.data, 0, 32, true);
        Iq_Measured = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Iq_Setpoint, 0, 32, true);
        can_setSignal<float>(msg.data, Iq_Measured, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Iq_Setpoint = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Iq_Measured = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x014;
    static constexpr uint8_t msg_length = 8;
};

struct Get_Temperature_msg_t final {
    float FET_Temperature   = 0.0f;  // [deg C]
    float Motor_Temperature = 0.0f;  // [deg C]

    Get_Temperature_msg_t() = default;

    Get_Temperature_msg_t(const can_Message_t& msg) {
        FET_Temperature   = can_getSignal<float>(msg.data, 0, 32, true);
        Motor_Temperature = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, FET_Temperature, 0, 32, true);
        can_setSignal<float>(msg.data, Motor_Temperature, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        FET_Temperature   = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Motor_Temperature = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x015;
    static constexpr uint8_t msg_length = 8;
};

struct Reboot_msg_t final {
    Reboot_msg_t() = default;

    Reboot_msg_t(const can_Message_t& msg) {
    }

    void encode(can_Message_t& msg) const {
    }

    void decode(const can_Message_t& msg) {
    }

    static constexpr uint8_t cmd_id     = 0x016;
    static constexpr uint8_t msg_length = 0;
};

struct Get_Bus_Voltage_Current_msg_t final {
    float Bus_Voltage = 0.0f;  // [V]
    float Bus_Current = 0.0f;  // [A]

    Get_Bus_Voltage_Current_msg_t() = default;

    Get_Bus_Voltage_Current_msg_t(const can_Message_t& msg) {
        Bus_Voltage = can_getSignal<float>(msg.data, 0, 32, true);
        Bus_Current = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Bus_Voltage, 0, 32, true);
        can_setSignal<float>(msg.data, Bus_Current, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Bus_Voltage = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Bus_Current = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x017;
    static constexpr uint8_t msg_length = 8;
};

struct Clear_Errors_msg_t final {
    Clear_Errors_msg_t() = default;

    Clear_Errors_msg_t(const can_Message_t& msg) {
    }

    void encode(can_Message_t& msg) const {
    }

    void decode(const can_Message_t& msg) {
    }

    static constexpr uint8_t cmd_id     = 0x018;
    static constexpr uint8_t msg_length = 0;
};

struct Set_Absolute_Position_msg_t final {
    float Position = 0.0f;  // [rev]

    Set_Absolute_Position_msg_t() = default;

    Set_Absolute_Position_msg_t(const can_Message_t& msg) {
        Position = can_getSignal<float>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Position, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Position = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x019;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Pos_Gain_msg_t final {
    float Pos_Gain = 0.0f;  // [(rev/s) / rev]

    Set_Pos_Gain_msg_t() = default;

    Set_Pos_Gain_msg_t(const can_Message_t& msg) {
        Pos_Gain = can_getSignal<float>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Pos_Gain, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Pos_Gain = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x01A;
    static constexpr uint8_t msg_length = 8;
};

struct Set_Vel_Gains_msg_t final {
    float Vel_Gain            = 0.0f;  // [Nm / (rev/s)]
    float Vel_Integrator_Gain = 0.0f;  // [Nm / rev]

    Set_Vel_Gains_msg_t() = default;

    Set_Vel_Gains_msg_t(const can_Message_t& msg) {
        Vel_Gain            = can_getSignal<float>(msg.data, 0, 32, true);
        Vel_Integrator_Gain = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Vel_Gain, 0, 32, true);
        can_setSignal<float>(msg.data, Vel_Integrator_Gain, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Vel_Gain            = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Vel_Integrator_Gain = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x01B;
    static constexpr uint8_t msg_length = 8;
};

struct Get_Torques_msg_t final {
    float Torque_Target   = 0.0f;  // [Nm]
    float Torque_Estimate = 0.0f;  // [Nm]

    Get_Torques_msg_t() = default;

    Get_Torques_msg_t(const can_Message_t& msg) {
        Torque_Target   = can_getSignal<float>(msg.data, 0, 32, true);
        Torque_Estimate = can_getSignal<float>(msg.data, 32, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<float>(msg.data, Torque_Target, 0, 32, true);
        can_setSignal<float>(msg.data, Torque_Estimate, 32, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Torque_Target   = can_getSignal<float>(msg.data, 0, 32, true, 1.0f, 0.0f);
        Torque_Estimate = can_getSignal<float>(msg.data, 32, 32, true, 1.0f, 0.0f);
    }

    static constexpr uint8_t cmd_id     = 0x01C;
    static constexpr uint8_t msg_length = 8;
};

struct Get_Controller_Error_msg_t final {
    uint32_t Controller_Error = 0;

    Get_Controller_Error_msg_t() = default;

    Get_Controller_Error_msg_t(const can_Message_t& msg) {
        Controller_Error = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    void encode(can_Message_t& msg) const {
        can_setSignal<uint32_t>(msg.data, Controller_Error, 0, 32, true);
    }

    void decode(const can_Message_t& msg) {
        Controller_Error = can_getSignal<uint32_t>(msg.data, 0, 32, true);
    }

    static constexpr uint8_t cmd_id     = 0x01D;
    static constexpr uint8_t msg_length = 8;
};

struct Enter_DFU_Mode_msg_t final {
    Enter_DFU_Mode_msg_t() = default;

    Enter_DFU_Mode_msg_t(const can_Message_t& msg) {
    }

    void encode(can_Message_t& msg) const {
    }

    void decode(const can_Message_t& msg) {
    }

    static constexpr uint8_t cmd_id     = 0x01F;
    static constexpr uint8_t msg_length = 0;
};

struct ODriveArduinoCAN {
    ODriveArduinoCAN(uint32_t axis_id) : axis_id_(axis_id) {}

    Get_Version_msg_t             get_version_msg;
    Heartbeat_msg_t               heartbeat_msg;
    Estop_msg_t                   estop_msg;
    Get_Error_msg_t               get_error_msg;
    Set_Axis_Node_ID_msg_t        set_axis_node_id_msg;
    Set_Axis_State_msg_t          set_axis_state_msg;
    Get_Encoder_Estimates_msg_t   get_encoder_estimates_msg;
    Set_Controller_Mode_msg_t     set_controller_mode_msg;
    Set_Input_Pos_msg_t           set_input_pos_msg;
    Set_Input_Vel_msg_t           set_input_vel_msg;
    Set_Input_Torque_msg_t        set_input_torque_msg;
    Set_Limits_msg_t              set_limits_msg;
    Set_Traj_Vel_Limit_msg_t      set_traj_vel_limit_msg;
    Set_Traj_Accel_Limits_msg_t   set_traj_accel_limits_msg;
    Set_Traj_Inertia_msg_t        set_traj_inertia_msg;
    Get_Iq_msg_t                  get_iq_msg;
    Get_Temperature_msg_t         get_temperature_msg;
    Reboot_msg_t                  reboot_msg;
    Get_Bus_Voltage_Current_msg_t get_bus_voltage_current_msg;
    Clear_Errors_msg_t            clear_errors_msg;
    Set_Absolute_Position_msg_t   set_absolute_position_msg;
    Set_Pos_Gain_msg_t            set_pos_gain_msg;
    Set_Vel_Gains_msg_t           set_vel_gains_msg;
    Get_Torques_msg_t             get_torques_msg;
    Get_Controller_Error_msg_t    get_controller_error_msg;
    Enter_DFU_Mode_msg_t          enter_dfu_mode_msg;

    enum CmdList {
        kGetVersionMsg           = 0x000,
        kHeartbeatMsg            = 0x001,
        kEstopMsg                = 0x002,
        kGetErrorMsg             = 0x003,
        kSetAxisNodeIDMsg        = 0x006,
        kSetAxisStateMsg         = 0x007,
        kGetEncoderEstimatesMsg  = 0x009,
        kSetControllerModeMsg    = 0x00B,
        kSetInputPosMsg          = 0x00C,
        kSetInputVelMsg          = 0x00D,
        kSetInputTorqueMsg       = 0x00E,
        kSetLimitsMsg            = 0x00F,
        kSetTrajVelLimitMsg      = 0x011,
        kSetTrajAccelLimitsMsg   = 0x012,
        kSetTrajInertiaMsg       = 0x013,
        kGetIqMsg                = 0x014,
        kGetTemperatureMsg       = 0x015,
        kRebootMsg               = 0x016,
        kGetBusVoltageCurrentMsg = 0x017,
        kClearErrorsMsg          = 0x018,
        kSetAbsolutePositionMsg  = 0x019,
        kSetPosGainMsg           = 0x01A,
        kSetVelGainsMsg          = 0x01B,
        kGetTorquesMsg           = 0x01C,
        kGetControllerErrorMsg   = 0x01D,
        kEnterDFUModeMsg         = 0x01F,
    };

    void decode(const can_Message_t& msg) {
        switch (get_cmd_id(msg.id)) {
            case kGetVersionMsg: get_version_msg.decode(msg); break;
            case kHeartbeatMsg: heartbeat_msg.decode(msg); break;
            case kEstopMsg: estop_msg.decode(msg); break;
            case kGetErrorMsg: get_error_msg.decode(msg); break;
            case kSetAxisNodeIDMsg: set_axis_node_id_msg.decode(msg); break;
            case kSetAxisStateMsg: set_axis_state_msg.decode(msg); break;
            case kGetEncoderEstimatesMsg: get_encoder_estimates_msg.decode(msg); break;
            case kSetControllerModeMsg: set_controller_mode_msg.decode(msg); break;
            case kSetInputPosMsg: set_input_pos_msg.decode(msg); break;
            case kSetInputVelMsg: set_input_vel_msg.decode(msg); break;
            case kSetInputTorqueMsg: set_input_torque_msg.decode(msg); break;
            case kSetLimitsMsg: set_limits_msg.decode(msg); break;
            case kSetTrajVelLimitMsg: set_traj_vel_limit_msg.decode(msg); break;
            case kSetTrajAccelLimitsMsg: set_traj_accel_limits_msg.decode(msg); break;
            case kSetTrajInertiaMsg: set_traj_inertia_msg.decode(msg); break;
            case kGetIqMsg: get_iq_msg.decode(msg); break;
            case kGetTemperatureMsg: get_temperature_msg.decode(msg); break;
            case kRebootMsg: reboot_msg.decode(msg); break;
            case kGetBusVoltageCurrentMsg: get_bus_voltage_current_msg.decode(msg); break;
            case kClearErrorsMsg: clear_errors_msg.decode(msg); break;
            case kSetAbsolutePositionMsg: set_absolute_position_msg.decode(msg); break;
            case kSetPosGainMsg: set_pos_gain_msg.decode(msg); break;
            case kSetVelGainsMsg: set_vel_gains_msg.decode(msg); break;
            case kGetTorquesMsg: get_torques_msg.decode(msg); break;
            case kGetControllerErrorMsg: get_controller_error_msg.decode(msg); break;
            case kEnterDFUModeMsg: enter_dfu_mode_msg.decode(msg); break;
            default: break;
        }
    };

    can_Message_t encode(const CmdList cmd) const {
        can_Message_t msg;
        msg.id  = (axis_id_ << kNumCmdIdBits) | cmd;
        msg.len = 8;

        switch (cmd) {
            case kGetVersionMsg: get_version_msg.encode(msg); break;
            case kHeartbeatMsg: heartbeat_msg.encode(msg); break;
            case kEstopMsg: estop_msg.encode(msg); break;
            case kGetErrorMsg: get_error_msg.encode(msg); break;
            case kSetAxisNodeIDMsg: set_axis_node_id_msg.encode(msg); break;
            case kSetAxisStateMsg: set_axis_state_msg.encode(msg); break;
            case kGetEncoderEstimatesMsg: get_encoder_estimates_msg.encode(msg); break;
            case kSetControllerModeMsg: set_controller_mode_msg.encode(msg); break;
            case kSetInputPosMsg: set_input_pos_msg.encode(msg); break;
            case kSetInputVelMsg: set_input_vel_msg.encode(msg); break;
            case kSetInputTorqueMsg: set_input_torque_msg.encode(msg); break;
            case kSetLimitsMsg: set_limits_msg.encode(msg); break;
            case kSetTrajVelLimitMsg: set_traj_vel_limit_msg.encode(msg); break;
            case kSetTrajAccelLimitsMsg: set_traj_accel_limits_msg.encode(msg); break;
            case kSetTrajInertiaMsg: set_traj_inertia_msg.encode(msg); break;
            case kGetIqMsg: get_iq_msg.encode(msg); break;
            case kGetTemperatureMsg: get_temperature_msg.encode(msg); break;
            case kRebootMsg: reboot_msg.encode(msg); break;
            case kGetBusVoltageCurrentMsg: get_bus_voltage_current_msg.encode(msg); break;
            case kClearErrorsMsg: clear_errors_msg.encode(msg); break;
            case kSetAbsolutePositionMsg: set_absolute_position_msg.encode(msg); break;
            case kSetPosGainMsg: set_pos_gain_msg.encode(msg); break;
            case kSetVelGainsMsg: set_vel_gains_msg.encode(msg); break;
            case kGetTorquesMsg: get_torques_msg.encode(msg); break;
            case kGetControllerErrorMsg: get_controller_error_msg.encode(msg); break;
            case kEnterDFUModeMsg: enter_dfu_mode_msg.encode(msg); break;
            default: break;
        }

        return msg;
    }

    // Utility functions
    static uint8_t get_node_id(uint32_t msg_id) {
        return (msg_id >> kNumCmdIdBits);  // Upper 6 or more bits
    };

    static uint8_t get_cmd_id(uint32_t msg_id) {
        return static_cast<uint8_t>(msg_id & 0x01F);  // Bottom 5 bits
    }

   private:
    static const uint8_t kNumNodeIdBits = 6U;
    static const uint8_t kNumCmdIdBits  = 5U;

    const uint32_t axis_id_ = 0;
};
