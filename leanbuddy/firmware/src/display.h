// GC9A01 round LCD: pushes LeanBuddy's RGB565 frame buffer with DMA.
#pragma once
#include <stdint.h>

namespace display {
bool begin();
// Starts sending `fb` (240x240 RGB565, native byte order) and returns at once;
// the previous frame's transfer is waited for first.
void push(const uint16_t *fb);
void brightness(uint8_t level);
}  // namespace display
