#include "qmi8658.h"

#include "config.h"

namespace {
// registers (QMI8658 datasheet)
constexpr uint8_t REG_WHO_AM_I = 0x00;  // reads 0x05
constexpr uint8_t REG_CTRL1 = 0x02;     // serial interface
constexpr uint8_t REG_CTRL2 = 0x03;     // accelerometer: full scale, output rate
constexpr uint8_t REG_CTRL3 = 0x04;     // gyroscope: full scale, output rate
constexpr uint8_t REG_CTRL5 = 0x06;     // low-pass filters
constexpr uint8_t REG_CTRL7 = 0x08;     // sensor enable
constexpr uint8_t REG_AX_L = 0x35;      // AX, AY, AZ, GX, GY, GZ: 12 bytes
constexpr uint8_t REG_RESET = 0x60;
constexpr uint8_t WHO_AM_I = 0x05;

constexpr uint8_t CTRL1_ADDR_AI = 0x40;        // auto-increment register address
constexpr uint8_t CTRL2_ACC_8G_235HZ = 0x25;   // aFS=010 (+-8 g), aODR=0101
constexpr uint8_t CTRL3_GYR_512_235HZ = 0x55;  // gFS=101 (+-512 dps), gODR=0101
constexpr uint8_t CTRL5_LPF = 0x77;            // both low-pass filters on, widest mode
constexpr uint8_t CTRL7_ACC_GYR_ON = 0x03;
constexpr float ACC_LSB_PER_G = 4096.0f;       // at +-8 g
constexpr float GYR_LSB_PER_DPS = 64.0f;       // at +-512 dps
}  // namespace

bool Qmi8658::write(uint8_t reg, uint8_t v) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(v);
    return wire_->endTransmission() == 0;
}

bool Qmi8658::readBytes(uint8_t reg, uint8_t *buf, size_t n) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom(addr_, (uint8_t)n) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = wire_->read();
    return true;
}

bool Qmi8658::begin(TwoWire &wire) {
    wire_ = &wire;
    for (uint8_t a : {IMU_ADDR_PRIMARY, IMU_ADDR_ALT}) {
        addr_ = a;
        uint8_t id = 0;
        if (readBytes(REG_WHO_AM_I, &id, 1) && id == WHO_AM_I) break;
        addr_ = 0;
    }
    if (!addr_) return false;
    write(REG_RESET, 0xB0);
    delay(30);
    bool ok = write(REG_CTRL1, CTRL1_ADDR_AI) && write(REG_CTRL2, CTRL2_ACC_8G_235HZ) &&
              write(REG_CTRL3, CTRL3_GYR_512_235HZ) && write(REG_CTRL5, CTRL5_LPF) &&
              write(REG_CTRL7, CTRL7_ACC_GYR_ON);
    if (!ok) return false;
    delay(50);

    // Byte order differs between datasheet revisions; at rest |a| must be ~1 g,
    // so whichever order gives that is the right one.
    uint8_t b[12];
    float le = 0, be = 0;
    for (int k = 0; k < 10; k++) {
        delay(10);
        if (!readBytes(REG_AX_L, b, 6)) continue;
        for (int i = 0; i < 3; i++) {
            float l = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8)) / ACC_LSB_PER_G;
            float g = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / ACC_LSB_PER_G;
            le += l * l;
            be += g * g;
        }
    }
    big_endian_ = fabsf(sqrtf(be / 10) - 1.0f) < fabsf(sqrtf(le / 10) - 1.0f);
    return true;
}

bool Qmi8658::read(float acc_g[3], float gyro_dps[3]) {
    uint8_t b[12];
    if (!readBytes(REG_AX_L, b, sizeof b)) return false;
    for (int i = 0; i < 6; i++) {
        int16_t v = big_endian_ ? (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) : (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        if (i < 3) acc_g[i] = v / ACC_LSB_PER_G;
        else gyro_dps[i - 3] = v / GYR_LSB_PER_DPS;
    }
    return true;
}
