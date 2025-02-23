#pragma once

struct RgbC33 {
    void setup() {
        pinMode(LEDR, OUTPUT);
        pinMode(LEDG, OUTPUT);
        pinMode(LEDB, OUTPUT);

        setColor(0, 50, 0);
    }

    void setColor(uint8_t R, uint8_t G, uint8_t B) {
        analogWrite(LEDR, 255 - R);
        analogWrite(LEDG, 255 - G);
        analogWrite(LEDB, 255 - B);
    }
};

extern RgbC33 pixel;
