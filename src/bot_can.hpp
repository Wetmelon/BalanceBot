#pragma once

#include <Arduino_CAN.h>

#include "ODriveEnums.h"
#include "can_simple_messages.hpp"

// Helper function for sending can messages
void sendCanMsg(const can_Message_t &msg) {
    const CanMsg c33msg{CanStandardId(msg.id), msg.len, msg.data};

    auto ret = CAN1.write(c33msg);

    if (ret != 1) {
        Serial.print("Tx Failed: ");
        Serial.print(ret, HEX);
        Serial.print(" - ");
        Serial.println(c33msg);
    } else {
        Serial.println("Tx Success");
    }

    // TODO:  Figure out why this is needed - no FIFO in R7FA6M5_CAN ???
    delayMicroseconds(500);
}

struct BotCanClass {
    void setup() {
// Portenta C33 FD transceiver pins
#ifdef ARDUINO_PORTENTA_C33
        pinMode(PIN_CAN1_STBY, OUTPUT);
        digitalWrite(PIN_CAN1_STBY, LOW);
#endif

        left_motor.axis_id_  = 0;
        right_motor.axis_id_ = 1;

        // Start CAN at 500kbps
        CAN1.disableInternalLoopback();

        if (!CAN1.begin(CanBitRate::BR_500k)) {
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

        m_axis_state_update = true;
    }

    void read() {
        while (CAN1.available()) {
            const CanMsg rxmsg = CAN1.read();

            can_Message_t odrv_msg = {
                .id  = rxmsg.id,
                .len = rxmsg.data_length
            };

            std::memcpy(odrv_msg.data, rxmsg.data, rxmsg.data_length);

            // pixel.setColor(0, led, led);
            switch (ODriveArduinoCAN::get_node_id(rxmsg.id)) {
                case 0: left_motor.decode(odrv_msg); break;
                case 1: right_motor.decode(odrv_msg); break;
                default: break;
            }
        }
    }

    void send() {
        // Send the periodic CAN messages
        sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
        sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));

        if (m_axis_state_update) {
            sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));
            sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));

            sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));
            sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));

            sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));
            sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));

            m_axis_state_update = false;
        }
    }

    // CAN communication objects for the ODrives
    ODriveArduinoCAN left_motor;   // Node ID 0
    ODriveArduinoCAN right_motor;  // Node ID 1

    bool m_axis_state_update = false;
};

extern BotCanClass bot_can;
