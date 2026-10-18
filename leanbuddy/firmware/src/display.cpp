#include "display.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <esp_heap_caps.h>

#include "config.h"
#include "gfx.h"

namespace {

class Lcd : public lgfx::LGFX_Device {
    lgfx::Panel_GC9A01 panel_;
    lgfx::Bus_SPI bus_;
    lgfx::Light_PWM light_;

public:
    Lcd() {
        {
            auto cfg = bus_.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 80000000;
            cfg.freq_read = 20000000;
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = PIN_LCD_SCLK;
            cfg.pin_mosi = PIN_LCD_MOSI;
            cfg.pin_miso = -1;
            cfg.pin_dc = PIN_LCD_DC;
            bus_.config(cfg);
            panel_.setBus(&bus_);
        }
        {
            auto cfg = panel_.config();
            cfg.pin_cs = PIN_LCD_CS;
            cfg.pin_rst = PIN_LCD_RST;
            cfg.pin_busy = -1;
            cfg.panel_width = LB_W;
            cfg.panel_height = LB_H;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.readable = false;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            panel_.config(cfg);
        }
        {
            auto cfg = light_.config();
            cfg.pin_bl = PIN_LCD_BL;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 7;
            light_.config(cfg);
            panel_.setLight(&light_);
        }
        setPanel(&panel_);
    }
};

Lcd lcd;
uint16_t *tx = nullptr;  // byte-swapped copy being sent while the next frame renders

}  // namespace

namespace display {

bool begin() {
    if (!lcd.init()) return false;
    lcd.setRotation(0);
    lcd.fillScreen(TFT_BLACK);
    tx = (uint16_t *)heap_caps_malloc(LB_W * LB_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    return tx != nullptr;
}

void push(const uint16_t *fb) {
    lcd.waitDMA();
    // the panel wants big-endian RGB565
    for (int i = 0; i < LB_W * LB_H; i++) tx[i] = (uint16_t)((fb[i] >> 8) | (fb[i] << 8));
    lcd.startWrite();
    lcd.pushImageDMA(0, 0, LB_W, LB_H, (const lgfx::swap565_t *)tx);
    lcd.endWrite();
}

void brightness(uint8_t level) { lcd.setBrightness(level); }

}  // namespace display
