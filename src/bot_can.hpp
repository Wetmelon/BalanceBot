#pragma once

// MKR CAN shield (MCP2515) at 500 kbps to two ODrives in torque mode.
#include <CAN.h>

#include "ODriveEnums.h"
#include "can_simple_messages.hpp"

inline void sendCanMsg(const can_Message_t& msg) {
    CAN.beginPacket(msg.id, msg.len);
    CAN.write(msg.data, msg.len);
    CAN.endPacket();
}

struct BotCanClass {
    void setup() {
        for (ODriveArduinoCAN* m : {&left_motor, &right_motor}) {
            m->set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
            m->set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;
            m->set_limits_msg.Velocity_Limit        = 1000.0f;  // [rev/s]
            m->set_limits_msg.Current_Limit         = 40.0f;    // [A]
        }
        left_motor.axis_id_  = 0;
        right_motor.axis_id_ = 1;

        if (!CAN.begin(500000)) {
            Serial.println("CAN Begin Failed!");
        } else {
            Serial.println("Connected to CAN at 500kbps");
        }

        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kClearErrorsMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kClearErrorsMsg));
    }

    /// Request an axis state on both drives, with the controller mode and limits it needs.
    void setAxisStates(ODriveAxisState state) {
        for (ODriveArduinoCAN* m : {&left_motor, &right_motor}) {
            m->set_axis_state_msg.Axis_Requested_State = state;
            sendCanMsg(m->encode(ODriveArduinoCAN::kSetControllerModeMsg));
            sendCanMsg(m->encode(ODriveArduinoCAN::kSetLimitsMsg));
            sendCanMsg(m->encode(ODriveArduinoCAN::kSetAxisStateMsg));
        }
    }

    /// Drain the receive buffer into the per-axis messages.
    void read() {
        while (CAN.parsePacket()) {
            can_Message_t rxmsg;
            rxmsg.id = CAN.packetId();
            CAN.readBytes(rxmsg.data, 8);
            switch (ODriveArduinoCAN::get_node_id(rxmsg.id)) {
                case 0: left_motor.decode(rxmsg); break;
                case 1: right_motor.decode(rxmsg); break;
                default: break;
            }
        }
    }

    /// Motor torques [N·m], each in its own motor's sense.
    void sendTorques(float left, float right) {
        left_motor.set_input_torque_msg.Input_Torque  = left;
        right_motor.set_input_torque_msg.Input_Torque = right;
        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
    }

    [[nodiscard]] bool healthy() const {
        return left_motor.heartbeat_msg.Axis_Error == 0 && right_motor.heartbeat_msg.Axis_Error == 0;
    }

    ODriveArduinoCAN left_motor;   // Node ID 0
    ODriveArduinoCAN right_motor;  // Node ID 1
};

extern BotCanClass bot_can;
