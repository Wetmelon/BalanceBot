// The C33's CAN1 with nothing to ACK its frames: the condition that used to hang the board at boot.
// The Portenta on its Hat Carrier, on USB, and no other powered node on the bus.
//   pio test -e portenta_c33 -f test_can_no_ack
//
// Checks the two fixes in lib/Arduino_CAN/src/r_canfd.c (arduino/fsp#2 and #3), not BalanceBot's
// own TX FIFO: R_CANFD_Write is called directly.
#include <Arduino.h>
#include <Arduino_CAN.h>
#include <unity.h>

static can_frame_t frame(uint32_t id) {
    can_frame_t f{};
    f.id               = id;
    f.id_mode          = CAN_ID_MODE_STANDARD;
    f.type             = CAN_FRAME_TYPE_DATA;
    f.data_length_code = 1;
    f.data[0]          = static_cast<uint8_t>(id);
    return f;
}

// arduino/fsp#2: a frame nobody ACKs stays pending in its TX buffer, retried forever. Every write
// to that buffer must be refused; FSP 4.0 accepted one whenever it caught the buffer between
// retries, and overwrote the pending frame. (Buffer 1: buffer 0 carries BalanceBot's TX FIFO.)
static void test_write_refuses_pending_buffer() {
    can_frame_t first = frame(0x101);
    TEST_ASSERT_EQUAL_MESSAGE(FSP_SUCCESS, R_CANFD_Write(CAN1.fspCtrl(), 1, &first), "first write");

    int accepted = 0;
    for (const uint32_t start = millis(); millis() - start < 20;) {
        can_frame_t second = frame(0x102);
        accepted += R_CANFD_Write(CAN1.fspCtrl(), 1, &second) == FSP_SUCCESS ? 1 : 0;
    }
    TEST_ASSERT_EQUAL_MESSAGE(0, accepted, "a write replaced the pending frame (or something on the bus ACKed it)");
}

// arduino/fsp#3: those retries take the transmit error count past error warning and error
// passive, and each raises a channel error interrupt. The handler must clear the flags and
// return; FSP 4.0's re-entered forever, and this test never got here.
static void test_error_interrupts_return() {
    delay(50);  // ~16 retries reach error passive; they take a few ms
    int code = 0;
    TEST_ASSERT_TRUE_MESSAGE(CAN1.isError(code), "no channel error reported");
    TEST_ASSERT_BITS(CANFD_ERROR_CHANNEL_WARNING | CANFD_ERROR_CHANNEL_PASSIVE,
                     CANFD_ERROR_CHANNEL_WARNING | CANFD_ERROR_CHANNEL_PASSIVE, code);
}

void setup() {
    delay(2000);  // USB serial up
    pinMode(PIN_CAN1_STBY, OUTPUT);
    digitalWrite(PIN_CAN1_STBY, LOW);  // transceiver on: frames go out on the bus, and nobody ACKs
    CAN1.begin(CanBitRate::BR_500k);

    UNITY_BEGIN();
    RUN_TEST(test_write_refuses_pending_buffer);
    RUN_TEST(test_error_interrupts_return);
    UNITY_END();
}

void loop() {}
