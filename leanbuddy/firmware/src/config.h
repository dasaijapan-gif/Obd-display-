// Board and tuning settings for the Waveshare ESP32-S3-Touch-LCD-1.28.
// Pins are from the board's Rev3 schematic.  The "-B" revision of the board may
// differ: check its schematic before flashing.
#pragma once

// ---- LCD: GC9A01, 240x240 round, SPI ----
#define PIN_LCD_SCLK 10
#define PIN_LCD_MOSI 11
#define PIN_LCD_CS 9
#define PIN_LCD_DC 8
#define PIN_LCD_RST 14
#define PIN_LCD_BL 2

// ---- I2C bus shared by the IMU and the touch controller ----
#define PIN_I2C_SDA 6
#define PIN_I2C_SCL 7
#define I2C_HZ 400000

// ---- IMU: QMI8658 ----
#define IMU_ADDR_PRIMARY 0x6B
#define IMU_ADDR_ALT 0x6A
#define IMU_HZ 200  // sample rate the lean estimator is fed at

// ---- touch: CST816S ----
#define TOUCH_ADDR 0x15
#define PIN_TP_INT 5
#define PIN_TP_RST 13

// ---- buttons, battery ----
#define PIN_BOOT_BUTTON 0     // hold 2 s: recalibrate "upright"; tap: next page
#define PIN_BAT_ADC 1         // through a 1:3 divider (check your board's schematic)
#define BAT_DIVIDER 3.0f

// ---- how the IMU sits relative to the screen (sensor axes) ----
// Defaults assume the chip's +Z points out of the screen and +Y to the top of the
// picture.  If the toy shows lean the wrong way round, run the orientation wizard
// (send 'o' over the serial monitor) instead of editing these: it measures them and
// saves the result.
#define IMU_SCREEN_NORMAL_X 0.0f
#define IMU_SCREEN_NORMAL_Y 0.0f
#define IMU_SCREEN_NORMAL_Z 1.0f
#define IMU_SCREEN_TOP_X 0.0f
#define IMU_SCREEN_TOP_Y 1.0f
#define IMU_SCREEN_TOP_Z 0.0f

// ---- display ----
#define FRAME_MS 33            // ~30 fps
#define BACKLIGHT_MAX 255
#define BACKLIGHT_ASLEEP 20

// ---- saving to flash (wear-friendly: not more often than this) ----
#define SAVE_MOUNT_EVERY_MS 10000
#define SAVE_RECORDS_EVERY_MS 30000
