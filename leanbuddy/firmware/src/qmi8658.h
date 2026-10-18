// Minimal QMI8658 6-axis IMU driver (accelerometer + gyroscope) over I2C.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Qmi8658 {
public:
    // Finds the chip at either address, resets and configures it:
    // accel +-8 g, gyro +-512 deg/s, ~235 Hz output, on-chip low-pass on.
    bool begin(TwoWire &wire);
    // Latest sample in g and deg/s, chip axes.  False on a bus error.
    bool read(float acc_g[3], float gyro_dps[3]);
    uint8_t address() const { return addr_; }

private:
    bool write(uint8_t reg, uint8_t v);
    bool readBytes(uint8_t reg, uint8_t *buf, size_t n);
    TwoWire *wire_ = nullptr;
    uint8_t addr_ = 0;
    bool big_endian_ = false;
};
