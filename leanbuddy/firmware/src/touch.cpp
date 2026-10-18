#include "touch.h"

#include "config.h"

namespace {
constexpr uint8_t REG_FINGER_NUM = 0x02;
constexpr uint8_t REG_DIS_AUTO_SLEEP = 0xFE;  // non-zero: stay awake, so polling works
constexpr uint32_t LONG_MS = 1500;
constexpr uint32_t POLL_MS = 25;

volatile bool g_irq = false;
void IRAM_ATTR onTouchIrq() { g_irq = true; }
}  // namespace

bool Touch::begin(TwoWire &wire) {
    wire_ = &wire;
    pinMode(PIN_TP_RST, OUTPUT);
    digitalWrite(PIN_TP_RST, LOW);
    delay(10);
    digitalWrite(PIN_TP_RST, HIGH);
    delay(60);
    pinMode(PIN_TP_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_TP_INT), onTouchIrq, FALLING);
    wire_->beginTransmission(TOUCH_ADDR);
    wire_->write(REG_DIS_AUTO_SLEEP);
    wire_->write(1);
    ok_ = wire_->endTransmission() == 0;
    return ok_;
}

bool Touch::fingerDown() {
    wire_->beginTransmission(TOUCH_ADDR);
    wire_->write(REG_FINGER_NUM);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)1) != 1) return false;
    return wire_->read() > 0;
}

Press Touch::poll(uint32_t now) {
    if (!ok_) return Press::None;
    // only talk to the chip while a touch is in progress (the interrupt starts one)
    if (!down_ && !g_irq) return Press::None;
    if (now - last_poll_ms_ < POLL_MS) return Press::None;
    last_poll_ms_ = now;
    g_irq = false;
    bool f = fingerDown();
    Press out = Press::None;
    if (f && !down_) { down_ = true; long_sent_ = false; down_ms_ = now; }
    else if (f && down_ && !long_sent_ && now - down_ms_ >= LONG_MS) { long_sent_ = true; out = Press::Long; }
    else if (!f && down_) { down_ = false; if (!long_sent_) out = Press::Tap; }
    return out;
}
