# How LeanBuddy measures lean

Everything here is implemented in [`core/lean.c`](../core/lean.c) and checked by
[`tests/test_lean.c`](../tests/test_lean.c) against the ride simulator's ground truth.

## Why an accelerometer alone cannot do it

The obvious idea, "tilt = the direction of gravity", works on a bike parked on its side
stand and fails completely while riding. In a steady corner a motorbike leans exactly so
that gravity plus the cornering force points straight down through the bike; that
balance is the reason it doesn't fall over. So an accelerometer on the bike measures:

- straight down through the bike (no sideways component at all), and
- a little more than 1 g: `1 / cos(lean)`, 1.41 g at 45°.

That is the same reading as an upright bike going through a dip. The browser simulator
draws this as the grey "accelerometer alone" line: flat at zero while the true lean
reaches 47°. The test `test_accelerometer_alone_reads_upright_in_a_corner` checks it.

## What works: a gyro, kept honest

Axes: x forward, y left, z up. Lean is positive to the right, pitch positive nose-up.

**1. Integrate the gyro.** The gyro measures rotation rates directly. Turning the body
rates into the rates of change of lean φ and pitch θ (yaw → pitch → roll order):

```
yaw_c = q·sin φ + r·cos φ          (yaw rate × cos θ)
φ'    = p − yaw_c · tan θ
θ'    = r·sin φ − q·cos φ
```

That is exact from moment to moment but drifts: a cheap MEMS gyro is off by a degree
or two per second until its bias is learnt (every time the bike is still), and even
then small errors add up.

**2. Correct the drift with a reference that holds while riding.** Two of them, each
trusted only when it is valid:

- **Cornering: the turn-rate ratio.** A bike in a corner rotates about the world's
  vertical axis. Seen from the leaned-over bike, that axis is tilted by the lean, so
  the gyro sees the turn partly on its pitch axis and partly on its yaw axis:
  `q = ω sin φ`, `r = ω cos φ`, which gives

  ```
  φ = atan(q / r)
  ```

  It needs no speed, no GPS and no accelerometer. It is trusted once the turn rate is
  above a few degrees per second, and ignored while the bike is rolling quickly from
  side to side or pitching (bumps, wheelies), when the "steady turn" picture breaks.

- **Going straight or standing still: the accelerometer.** With no turn rate the bike
  is either upright and moving, or stationary, and then the tilt of gravity *is* the
  lean (the side-stand angle when parked). Trusted only when the measured acceleration
  is close to 1 g (not braking or accelerating hard) and the bike is not rolling fast.

The estimate is pulled towards whichever references are valid, with time constants
around 0.8 s, so the gyro handles everything quick and the references remove drift.
With neither valid it slowly relaxes towards upright.

**Pitch** for wheelies comes from the gyro too. While riding, forward acceleration
looks exactly like pitch to an accelerometer, so pitch is only re-zeroed slowly
(relative to the road), and from the accelerometer when stationary.

**G-forces** are the accelerometer reading rotated into the road plane using the lean
and pitch estimate, minus gravity: forward/braking G, and cornering G, which in a
steady corner equals `tan(lean)` (1 g at 45°).

## Learning how it's mounted

The toy can be mounted at any angle, so the firmware needs to know which way is "up"
and which way is "forward" in the sensor's axes.

- **Up**: whatever gravity says the first time the bike is held still (or, with the
  engine idling, quiet for 3 s); and again whenever you hold the BOOT button for
  recalibration.
- **Forward**: the rider looks at the screen, so the screen's right-hand edge points to
  the bike's right whatever the tilt; that gives forward from up. The orientation wizard
  measures which way the screen faces relative to the chip once.
- **Auto-trim of up**: riding in a straight line, the bike is upright on average. If
  the first calibration happened on the side stand (12° off), this pulls "up" back over
  the first half-minute of riding.
- **Auto-forward**: accelerating and braking in a straight line push along the bike's
  forward axis, which corrects a toy mounted slightly twisted.

Changes are saved to flash so the next ride starts calibrated.

## How good is it

On the simulated canyon ride (200 Hz samples; realistic sensor noise, engine
vibration, up to 2°/s gyro bias; the toy tilted 25° back and twisted 3°):

| | |
|---|---|
| Lean error, RMS | 0.6° |
| Lean error, worst moment | 1.7° (quick changes of direction) |
| Cornering G error | < 0.03 g |
| Mounts from upright to nearly flat, twisted up to 15° | < 2.5° RMS |
| Booted on the side stand | corrects itself within ~30 s of riding |

Real bikes are noisier than the simulator (handlebar vibration in particular), so the
first job on real hardware is logging rides (`d` on the serial monitor) and tuning.
