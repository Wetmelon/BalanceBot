#pragma once

// What differs between the boards, apart from the CAN driver (bot_can.hpp).
#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_PORTENTA_C33)
#include <Arduino_FreeRTOS.h>

namespace board {
inline TwoWire&      imu_wire  = Wire2;
constexpr pin_size_t kFaultPin = PIN_D7;  // high on an ODrive axis error

inline void setupLeds() {
    for (pin_size_t led : {LEDR, LEDG, LEDB}) {
        pinMode(led, OUTPUT);
        digitalWrite(led, HIGH);  // active low: off
    }
}
inline void blink(bool on) { digitalWrite(LEDB, on ? LOW : HIGH); }
}  // namespace board

#elif defined(ADAFRUIT_FEATHER_M4_CAN)
#include <FreeRTOS_SAMD51.h>

namespace board {
inline TwoWire& imu_wire  = Wire;  // SDA/SCL, also the STEMMA QT port
constexpr int   kFaultPin = 5;     // high on an ODrive axis error; D7 is the NeoPixel's power here

inline void setupLeds() { pinMode(LED_BUILTIN, OUTPUT); }
inline void blink(bool on) { digitalWrite(LED_BUILTIN, on ? HIGH : LOW); }
}  // namespace board

#else
#error "BalanceBot builds for the Portenta C33 or the Feather M4 CAN"
#endif
