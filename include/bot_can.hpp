#pragma once

// Portenta C33 CAN1 through the Hat Carrier's transceiver, 500 kbps, to two ODrives in torque mode.
// <Arduino_CAN.h> is the patched copy in lib/Arduino_CAN: RX from the 16-deep FIFO, all 16 TX buffers.
#include <Arduino_CAN.h>

#include "ODriveEnums.h"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "damp/toolbox/timing.hpp"

struct BotCanClass {
    /// [ms] A drive is gone after this long without a heartbeat (ODrive sends one every 100 ms by default).
    static constexpr uint32_t kHeartbeatTimeoutMs = 250;

    void setup() {
        pinMode(PIN_CAN1_STBY, OUTPUT);
        digitalWrite(PIN_CAN1_STBY, LOW);  // transceiver out of standby

        for (ODriveArduinoCAN* m : {&left_motor, &right_motor}) {
            m->set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
            m->set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;
            m->set_limits_msg.Velocity_Limit        = 1000.0f;  // [rev/s]
            m->set_limits_msg.Current_Limit         = 40.0f;    // [A]
        }
        left_motor.axis_id_  = 0;
        right_motor.axis_id_ = 1;
        for (damp::Timeout<uint32_t>* hb : {&left_heartbeat_, &right_heartbeat_}) {
            hb->tick(kHeartbeatTimeoutMs);  // unheard until the first heartbeat
        }

        if (!CAN1.begin(CanBitRate::BR_500k)) {
            Serial.println("CAN Begin Failed!");
        } else {
            Serial.println("Connected to CAN at 500kbps");
        }
    }

    /// Request an axis state on both drives, with the controller mode and limits it needs.
    void setAxisStates(ODriveAxisState state) {
        for (ODriveArduinoCAN* m : {&left_motor, &right_motor}) {
            m->set_axis_state_msg.Axis_Requested_State = state;
            send(*m, ODriveArduinoCAN::kSetControllerModeMsg);
            send(*m, ODriveArduinoCAN::kSetLimitsMsg);
            send(*m, ODriveArduinoCAN::kSetAxisStateMsg);
        }
    }

    /// Age the heartbeats by @p dt_ms, then drain the receive FIFO into the per-axis messages.
    void read(uint32_t dt_ms) {
        left_heartbeat_.tick(dt_ms);
        right_heartbeat_.tick(dt_ms);
        while (CAN1.available() > 0) {
            const CanMsg        rx = CAN1.read();
            const can_Message_t msg{rx.id, rx.data_length, rx.data};
            const bool          heartbeat = ODriveArduinoCAN::get_cmd_id(rx.id) == ODriveArduinoCAN::kHeartbeatMsg;
            switch (ODriveArduinoCAN::get_node_id(rx.id)) {
                case 0: receive(left_motor, left_heartbeat_, msg, heartbeat); break;
                case 1: receive(right_motor, right_heartbeat_, msg, heartbeat); break;
                default: break;
            }
        }
    }

    /// Motor torques [N·m], each in its own motor's sense.
    void sendTorques(float left, float right) {
        left_motor.set_input_torque_msg.Input_Torque  = left;
        right_motor.set_input_torque_msg.Input_Torque = right;
        send(left_motor, ODriveArduinoCAN::kSetInputTorqueMsg);
        send(right_motor, ODriveArduinoCAN::kSetInputTorqueMsg);
    }

    /// Both drives heard from within kHeartbeatTimeoutMs, neither reporting an axis error.
    [[nodiscard]] bool healthy() const {
        return heard(left_heartbeat_) && heard(right_heartbeat_) && axisErrors() == 0;
    }

    [[nodiscard]] uint32_t axisErrors() const {
        return left_motor.heartbeat_msg.Axis_Error | right_motor.heartbeat_msg.Axis_Error;
    }

    ODriveArduinoCAN left_motor;   // Node ID 0
    ODriveArduinoCAN right_motor;  // Node ID 1

   private:
    [[nodiscard]] static bool heard(const damp::Timeout<uint32_t>& hb) { return !hb.expired(); }

    void receive(ODriveArduinoCAN& m, damp::Timeout<uint32_t>& hb, const can_Message_t& msg, bool heartbeat) {
        m.decode(msg);
        if (!heartbeat) {
            return;
        }
        const bool appeared = !heard(hb);
        hb.reset();
        if (appeared) {
            send(m, ODriveArduinoCAN::kClearErrorsMsg);  // clear whatever it booted with
        }
    }

    /// Transmit only to a drive that is there: a frame to an absent node gets no ACK and keeps
    /// retrying in its TX buffer until the drive appears.
    void send(const ODriveArduinoCAN& m, ODriveArduinoCAN::CmdList cmd) {
        if (!heard(&m == &left_motor ? left_heartbeat_ : right_heartbeat_)) {
            return;
        }
        const can_Message_t msg = m.encode(cmd);
        if (CAN1.write(CanMsg{CanStandardId(msg.id), msg.len, msg.data}) < 0) {
            Serial.println(msg);
        }
    }

    damp::Timeout<uint32_t> left_heartbeat_{kHeartbeatTimeoutMs};
    damp::Timeout<uint32_t> right_heartbeat_{kHeartbeatTimeoutMs};
};

extern BotCanClass bot_can;
