/* Lean-angle and G-force estimation from a 6-axis IMU (accelerometer + gyro).
 *
 * Why this is not just atan2(accel): in a steady corner a motorbike leans exactly
 * so that gravity plus the cornering force points straight down through the bike.
 * An accelerometer on the bike therefore reads "upright, 1/cos(lean) g" at 45 deg
 * of lean, the same as an upright bike going over a crest.  Lean has to come from
 * the gyro, corrected for drift by references that hold while riding.  See
 * docs/how-lean-works.md for the derivation.
 *
 *   - the gyro's roll rate is integrated: exact over a few seconds, drifts slowly
 *   - in a turn the bike rotates about the world's vertical axis, and that axis is
 *     tilted by the lean in the bike's frame: pitch-axis rate / yaw-axis rate =
 *     tan(lean).  This is the drift reference while cornering.
 *   - going straight (no yaw) the accelerometer's tilt is the lean (~0 riding,
 *     the side-stand angle when parked).  This is the reference otherwise.
 *
 * Bike axes: x forward, y left, z up.  Lean (roll) is positive leaning RIGHT,
 * pitch positive nose UP (wheelie).  Accelerations are in g, rates in deg/s.
 */
#ifndef LB_LEAN_H
#define LB_LEAN_H

#include "lb_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float t;           /* seconds; may wrap round (a step backwards is skipped) */
    lb_vec3 acc_g;     /* accelerometer, sensor axes, g */
    lb_vec3 gyro_dps;  /* gyro, sensor axes, deg/s */
} lb_imu_sample;

/* How the sensor sits on the bike, in sensor axes.  Persisted by the firmware. */
typedef struct {
    lb_vec3 up;   /* unit, points up when the bike is upright */
    lb_vec3 fwd;  /* unit, points forward, perpendicular to up */
    int valid;
} lb_mount;

typedef enum {
    LB_CAL_NONE = 0,   /* no mounting known yet: waits for the bike to be still */
    LB_CAL_WAITING,    /* recalibration requested: hold the bike upright and still */
    LB_CAL_DONE,
} lb_cal_state;

typedef struct {
    /* --- configuration (set before/after init) --- */
    lb_vec3 screen_normal;  /* sensor-axis direction the screen faces (towards the rider) */
    lb_vec3 screen_top;     /* sensor-axis direction of the top edge of the picture */
    int auto_trim;          /* refine the mounting while riding (default on): "upright"
                               from riding straight, "forward" from accelerating/braking */

    /* --- outputs, updated every sample --- */
    float lean_deg;       /* + right */
    float pitch_deg;      /* + nose up, relative to the road (slowly re-zeroed while moving) */
    float lean_rate_dps;
    float yaw_rate_dps;   /* + turning left */
    float lon_g;          /* + accelerating, - braking (low-passed ~0.12 s) */
    float lat_g;          /* cornering G, + turning right (low-passed) */
    float vert_g;         /* ~1.0 on a flat road; spikes over bumps, ~0 in the air */
    float vert_g_fast;    /* the same, barely filtered: bumps and jumps */
    float vib_g;          /* RMS vibration: engine running / road surface */
    int still;            /* no motion at all (parked, engine off or rider off) */
    float still_s;        /* how long it has been still */
    float quiet_s;        /* how long the bike has not moved (engine may be running) */
    int moving;           /* riding, as opposed to parked or waiting */
    float turn_conf;      /* 0..1 confidence in the cornering reference right now */
    lb_cal_state cal;
    int mount_changed;    /* set when calibration/auto-trim changed the mount; firmware clears after saving */

    /* --- internal --- */
    lb_mount mount;
    lb_vec3 rx, ry, rz;   /* bike axes in sensor coordinates (rows of sensor->bike) */
    float roll, pitch;    /* rad */
    lb_vec3 bias;         /* gyro bias, sensor axes, deg/s */
    lb_vec3 acc_lp, acc_slow, gyro_lp;
    float acc_dev2, gyro_dev2, vib2;
    float last_t;
    int have_t;
    float since_activity_s;
    float trim_ay, trim_az, trim_w;
    lb_vec3 lin_lp;
    float vert_fast_lp;
    lb_vec3 fwd_sum;
    float fwd_w;
    float turn_q, turn_r; /* filtered pitch/yaw-axis rates for the cornering reference */
} lb_lean;

void lb_lean_init(lb_lean *l);
/* screen_normal: sensor axis the screen faces; screen_top: sensor axis of the picture's top */
void lb_lean_set_orientation(lb_lean *l, lb_vec3 screen_normal, lb_vec3 screen_top);
void lb_lean_set_mount(lb_lean *l, const lb_mount *m);
/* Ask for a fresh calibration: completes the next time the bike is held still. */
void lb_lean_request_calibration(lb_lean *l);
void lb_lean_update(lb_lean *l, const lb_imu_sample *s);

#ifdef __cplusplus
}
#endif

#endif
