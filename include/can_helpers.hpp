#pragma once

#include <cstdint>
#include <cstring>

struct can_Message_t : public Printable {
    can_Message_t() = default;
    can_Message_t(const uint32_t id, const uint8_t len, const uint8_t (&data)[8], const bool is_extended_id = false, const bool rtr = false)
        : id(id), len(len), is_extended_id(is_extended_id), rtr(rtr) {
        std::memcpy(this->data, data, 8);
    }

    uint32_t id             = 0x000;
    uint8_t  len            = 0;
    bool     is_extended_id = false;
    bool     rtr            = false;
    uint8_t  data[8]        = {0};

    size_t printTo(Print& p) const override {
        size_t n = 0;

        n += p.print("ID: 0x");
        n += p.print(id < 0x100 ? "0" : "");
        n += p.print(id < 0x10 ? "0" : "");
        n += p.print(id, HEX);
        n += p.print(is_extended_id ? "x" : "");
        n += p.print(" Len: ");
        n += p.print(len);
        n += p.print(" Data: ");

        for (int i = 0; i < len; i++) {
            if (data[i] < 0x10) {
                n += p.print('0');  // Add leading zero for single digit hex values
            }
            n += p.print(data[i], HEX);
            n += p.print(' ');
        }

        if (rtr) {
            n += p.print("(RTR)");
        }

        return n;
    }
};

template <typename T, size_t N>
T can_getSignal(const uint8_t (&buf)[N], const size_t startBit, const size_t length, const bool isIntel) {
    union {
        T retVal;
    };

    const uint64_t mask  = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t  shift = isIntel ? startBit : (64 - startBit) - length;

    uint64_t tempVal = 0U;
    std::memcpy(&tempVal, buf, N);
    if (isIntel) {
        tempVal = (tempVal >> shift) & mask;
    } else {
        tempVal = __builtin_bswap64(tempVal);
        tempVal = (tempVal >> shift) & mask;
    }

    return retVal;
}

template <typename T, size_t N>
void can_setSignal(uint8_t (&buf)[N], const T& val, const size_t startBit, const size_t length, const bool isIntel) {
    const uint64_t mask  = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t  shift = isIntel ? startBit : (64 - startBit) - length;

    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &val, sizeof(T));

    uint64_t data = 0;
    std::memcpy(&data, buf, N);
    if (isIntel) {
        data &= ~(mask << shift);
        data |= valAsBits << shift;
    } else {
        data = __builtin_bswap64(data);
        data &= ~(mask << shift);
        data |= valAsBits << shift;
        data = __builtin_bswap64(data);
    }

    std::memcpy(buf, &data, N);
}

template <typename T, size_t N>
float can_getSignal(const uint8_t (&buf)[N], const size_t startBit, const size_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(buf, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T, size_t N>
void can_setSignal(uint8_t (&buf)[N], const float& val, const size_t startBit, const size_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(buf, scaledVal, startBit, length, isIntel);
}
