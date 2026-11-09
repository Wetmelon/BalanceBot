// Each board's patched CAN driver in its controller's loopback mode: the board on USB, no bus needed.
//   pio test -e portenta_c33
//   pio test -e feather_m4_can
#include <Arduino.h>
#include <unity.h>

#if defined(ARDUINO_PORTENTA_C33)
#include <Arduino_CAN.h>

static bool beginLoopback() { return CAN1.begin(CanBitRate::BR_500k) && CAN1.enableInternalLoopback() == 1; }

static bool send(int id) {
    const uint8_t data = static_cast<uint8_t>(id);
    return CAN1.write(CanMsg(CanStandardId(id), 1, &data)) == 1;
}

static bool receive(int& id, int& data) {
    if (CAN1.available() == 0) {
        return false;
    }
    const CanMsg msg = CAN1.read();
    id               = static_cast<int>(msg.getStandardId());
    data             = msg.data[0];
    return true;
}
#else
#include <CAN.h>

static bool beginLoopback() { return CAN.begin(500000) == 1 && CAN.loopback() == 1; }

static bool send(int id) {
    CAN.beginPacket(id);
    CAN.write(static_cast<uint8_t>(id));
    return CAN.endPacket() != 0;
}

static bool receive(int& id, int& data) {
    if (CAN.parsePacket() <= 0) {
        return false;
    }
    id   = static_cast<int>(CAN.packetId());
    data = CAN.read();
    return true;
}
#endif

static void test_fifos() {
    TEST_ASSERT_TRUE(beginLoopback());

    // A burst fills the 32-frame TX FIFO, plus any frame sent meanwhile (one takes ~120 us), and
    // the next write is refused at once. Both stock drivers lose or block on the second frame.
    int queued = 0;
    while (queued < 64 && send(queued)) {
        ++queued;
    }
    TEST_ASSERT_GREATER_OR_EQUAL(32, queued);
    TEST_ASSERT_LESS_THAN(40, queued);

    // Top up to 64 as the FIFO drains; the 64-frame RX FIFO keeps all of them, in order.
    for (const uint32_t start = millis(); queued < 64 && millis() - start < 100;) {
        queued += send(queued) ? 1 : 0;
    }
    TEST_ASSERT_EQUAL(64, queued);
    delay(20);  // 64 one-byte frames: ~8 ms at 500 kbps
    for (int expected = 0; expected < 64; ++expected) {
        int id   = -1;
        int data = -1;
        TEST_ASSERT_TRUE(receive(id, data));
        TEST_ASSERT_EQUAL(expected, id);
        TEST_ASSERT_EQUAL(expected, data);
    }
    int id   = -1;
    int data = -1;
    TEST_ASSERT_FALSE(receive(id, data));
}

void setup() {
    delay(2000);  // USB serial up
    UNITY_BEGIN();
    RUN_TEST(test_fifos);
    UNITY_END();
}

void loop() {}
