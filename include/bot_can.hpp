#pragma once

// CAN at 500 kbps to two ODrives in torque mode.
//   Portenta C33: CAN1 through the Hat Carrier's transceiver. <Arduino_CAN.h> is the patched copy in
//                 lib/Arduino_CAN: RX from the 16-deep FIFO, all 16 TX buffers.
//   Feather M4 CAN: the SAME51's CAN0 through the on-board TCAN1051, Adafruit's CAN fork, stock: an
//                 8-deep RX FIFO, and one TX buffer that each write busy-waits on until it is sent.
#if defined(ARDUINO_PORTENTA_C33)
#include <Arduino_CAN.h>
#else
#include <CAN.h>
#endif

#include "ODriveEnums.h"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "damp/toolbox/timing.hpp"

struct BotCanClass {
    /// [ms] A drive is gone after this long without a heartbeat (ODrive sends one every 100 ms by default).
    static constexpr uint32_t kHeartbeatTimeoutMs = 250;

    void setup() {
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

        if (!canBegin()) {
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
        can_Message_t msg;
        while (canRead(msg)) {
            const bool heartbeat = ODriveArduinoCAN::get_cmd_id(msg.id) == ODriveArduinoCAN::kHeartbeatMsg;
            switch (ODriveArduinoCAN::get_node_id(msg.id)) {
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
        if (!canWrite(msg)) {
            Serial.println(msg);
        }
    }

#if defined(ARDUINO_PORTENTA_C33)
    static bool canBegin() {
        pinMode(PIN_CAN1_STBY, OUTPUT);
        digitalWrite(PIN_CAN1_STBY, LOW);  // transceiver out of standby
        return CAN1.begin(CanBitRate::BR_500k);
    }

    static bool canRead(can_Message_t& msg) {
        if (CAN1.available() == 0) {
            return false;
        }
        const CanMsg rx = CAN1.read();
        msg             = {rx.id, rx.data_length, rx.data};
        return true;
    }

    static bool canWrite(const can_Message_t& msg) { return CAN1.write(CanMsg{CanStandardId(msg.id), msg.len, msg.data}) >= 0; }
#else
    static bool canBegin() {
        pinMode(PIN_CAN_STANDBY, OUTPUT);
        digitalWrite(PIN_CAN_STANDBY, LOW);  // transceiver out of standby
        pinMode(PIN_CAN_BOOSTEN, OUTPUT);
        digitalWrite(PIN_CAN_BOOSTEN, HIGH);  // the transceiver's 5 V, also on battery
        if (CAN.begin(500000) == 0) {
            return false;
        }
        CAN.filter(0, 0);  // every standard ID, no extended: an extended ID would alias a node
        return true;
    }

    // ponytail: parsePacket() returns the DLC, so a zero-length frame ends this tick's drain early;
    // the rest wait in the FIFO for the next tick. ODrive sends none.
    static bool canRead(can_Message_t& msg) {
        if (CAN.parsePacket() <= 0) {
            return false;
        }
        msg     = {};
        msg.id  = static_cast<uint32_t>(CAN.packetId());
        msg.len = static_cast<uint8_t>(CAN.available());
        CAN.readBytes(msg.data, msg.len);  // only what arrived: readBytes would wait 1 s for more
        return true;
    }

    // The driver reports nothing: endPacket() is 1 whether or not the frame went out.
    static bool canWrite(const can_Message_t& msg) {
        CAN.beginPacket(static_cast<int>(msg.id), msg.len);
        CAN.write(msg.data, msg.len);
        return CAN.endPacket() != 0;
    }
#endif

    damp::Timeout<uint32_t> left_heartbeat_{kHeartbeatTimeoutMs};
    damp::Timeout<uint32_t> right_heartbeat_{kHeartbeatTimeoutMs};
};

extern BotCanClass bot_can;
