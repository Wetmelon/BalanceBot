#pragma once

#include <WiFiNINA.h>
#include <utility/wifi_drv.h>

struct MKRrgb {
    void setup() {
        WiFiDrv::pinMode(GREEN, OUTPUT);
        WiFiDrv::pinMode(RED, OUTPUT);
        WiFiDrv::pinMode(BLUE, OUTPUT);
        setColor(0, 0, 0);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        WiFiDrv::analogWrite(GREEN, r);  // GREEN
        WiFiDrv::analogWrite(RED, g);    // RED
        WiFiDrv::analogWrite(BLUE, b);   // BLUE
    }

   private:
    enum {
        GREEN = 25,
        RED   = 26,
        BLUE  = 27,
    };
};

// Global object
extern MKRrgb pixel;
