#pragma once

#include <CAN.h>
#include <FreeRTOS_SAMD21.h>

#include "ODriveEnums.h"
#include "can_simple_messages.hpp"

// Helper function for sending can messages
void sendCanMsg(const can_Message_t &msg) {
    CAN.beginPacket(msg.id, msg.len);
    CAN.write(msg.data, msg.len);
    CAN.endPacket();
}

struct BotCanClass {
    void setup() {
        left_motor.axis_id_  = 0;
        right_motor.axis_id_ = 1;

        // Start CAN at 250kbps
        if (!CAN.begin(250000)) {
            Serial.println("CAN Begin Failed!");
        } else {
            Serial.println("Connected to CAN at 500kbps");
        }

        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kClearErrorsMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kClearErrorsMsg));
    }

    void setAxisStates(ODriveAxisState state) {
        left_motor.set_axis_state_msg.Axis_Requested_State  = state;
        right_motor.set_axis_state_msg.Axis_Requested_State = state;

        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));

        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));

        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));
    }

    void read() {
        while (CAN.parsePacket()) {
            can_Message_t rxmsg;

            rxmsg.id = CAN.packetId();
            CAN.readBytes(rxmsg.data, 8);

            // pixel.setColor(0, led, led);
            switch (ODriveArduinoCAN::get_node_id(rxmsg.id)) {
                case 0: left_motor.decode(rxmsg); break;
                case 1: right_motor.decode(rxmsg); break;
                default: break;
            }
        }
    }

    void send() {
        // Send the periodic CAN messages
        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
    }

    // CAN communication objects for the ODrives
    ODriveArduinoCAN left_motor;   // Node ID 0
    ODriveArduinoCAN right_motor;  // Node ID 1
};

extern BotCanClass bot_can;
