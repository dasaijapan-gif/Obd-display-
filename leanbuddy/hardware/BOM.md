# Bill of materials (prototype)

One development board has almost everything on it.

| Part | Notes | Approx. cost |
|---|---|---|
| **Waveshare ESP32-S3-Touch-LCD-1.28** | ESP32-S3R2 (2 MB PSRAM, 16 MB flash), 1.28" 240×240 round IPS (GC9A01), QMI8658 6-axis IMU, CST816S touch, LiPo charger, USB-C. The firmware's pin map is for this board; the later "-B" revision may differ, so check its schematic. | US$18–25 |
| LiPo cell, 3.7 V, 400–1000 mAh, JST 1.25 mm | Battery-powered build. Fits behind the board in a case. | US$5–8 |
| *or* 5 V USB from the bike | A USB socket on the bars, or a 12 V → 5 V converter on a switched (ignition) feed so it sleeps with the bike. Fuse it. | US$5–10 |
| Case and mount | 3D-printed round case with a 17 mm ball or a 1" ball (RAM-style) socket on the back, or a bar clamp. See [mounting.md](mounting.md). | US$5–15 |
| Clear front window | Polycarbonate, not acrylic (it shatters). Anti-glare helps in sun. | US$2 |
| Foam or silicone gasket | Damps vibration and keeps rain out. | US$1 |

## Choosing other parts

The core code only needs a 6-axis IMU (accelerometer + gyroscope) at ~200 Hz and a
240×240 RGB565 display. To move to other hardware, write a driver that fills
`lb_imu_sample` (g and deg/s in the chip's axes) and a function that pushes the frame
buffer; nothing else changes.

- **IMU**: any 6-axis part works (LSM6DSOX, ICM-42688, BMI270, MPU-6050 clones).
  A gyro range of ±500 °/s and accelerometer range of ±8 g suit a bike.
- **Display**: round 1.28" GC9A01 modules are cheap and common. A larger round AMOLED
  (e.g. 1.43"/1.75" 466×466) would need the renderer scaled.
- **For a product**: an ESP32-S3 module (or ESP32-C3 if 30 fps is not needed), a 6-axis
  IMU, a round LCD, a LiPo charger and a fuel gauge on one PCB.
