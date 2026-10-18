// CST816S touch controller: only taps and long presses are needed.
#pragma once
#include <Arduino.h>
#include <Wire.h>

enum class Press { None, Tap, Long };

class Touch {
public:
    bool begin(TwoWire &wire);
    // Call often (every loop).  Reports a tap on release, a long press once held.
    Press poll(uint32_t now_ms);

private:
    bool fingerDown();
    TwoWire *wire_ = nullptr;
    bool ok_ = false, down_ = false, long_sent_ = false;
    uint32_t down_ms_ = 0, last_poll_ms_ = 0;
};
