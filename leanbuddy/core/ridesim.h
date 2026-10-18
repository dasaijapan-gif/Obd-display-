/* A motorbike ride simulator that produces what the IMU would measure.
 *
 * Used by the tests (estimated lean vs. the true lean) and by the browser
 * simulator, so the animations can be designed against realistic sensor data:
 * coordinated turns (the accelerometer reads "upright" mid-corner, exactly as on a
 * real bike), braking dive, wheelies, bumps, airtime, engine vibration, sensor
 * noise, gyro bias and a tilted, rotated mounting.
 */
#ifndef LB_RIDESIM_H
#define LB_RIDESIM_H

#include "lean.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    LB_SEG_ENGINE_OFF = 1 << 0,
    LB_SEG_BUMP = 1 << 1,      /* pothole at the start of the segment */
    LB_SEG_JUMP = 1 << 2,      /* airtime at the start of the segment */
    LB_SEG_WHEELIE = 1 << 3,   /* front wheel up for the segment */
    LB_SEG_STAND = 1 << 4,     /* parked on the side stand (leaning left) */
};

typedef struct {
    float dur_s;
    float lean_deg;  /* target lean, + right */
    float throttle;  /* 0..1 */
    float brake;     /* 0..1, 1 = ~1 g */
    int flags;
} lb_ride_seg;

typedef enum { LB_SCRIPT_MANUAL = 0, LB_SCRIPT_CANYON, LB_SCRIPT_SIDESTAND_BOOT } lb_script;

typedef struct {
    float t;
    /* ---- truth ---- */
    float v;                     /* m/s */
    float lean, lean_rate;       /* rad, + right */
    float pitch, pitch_rate;     /* rad, + nose up */
    float yaw_rate;              /* rad/s, + left */
    float lon_acc;               /* m/s^2 */
    float vert_extra;            /* m/s^2 on top of gravity (bumps, airtime) */
    int engine_on;

    /* ---- controls: set directly in manual mode, from the script otherwise ---- */
    float target_lean_deg, throttle, brake;
    int wheelie;

    /* ---- sensor model ---- */
    lb_vec3 up_s, fwd_s, left_s; /* true mounting: bike axes in sensor coordinates */
    lb_vec3 gyro_bias_dps;
    float acc_noise_g, gyro_noise_dps;
    lb_rng rng;

    /* ---- events and script ---- */
    float bump_t, jump_t;       /* time since the event started, < 0 = none */
    float vib_phase;
    lb_script script;
    const lb_ride_seg *segs;
    int nsegs, seg;
    float seg_t;
    int loops;
} lb_ridesim;

void lb_ridesim_init(lb_ridesim *r, lb_script script, uint32_t seed);
/* mounting: screen tilted back by tilt_deg from vertical, turned by yaw_deg */
void lb_ridesim_set_mount(lb_ridesim *r, float tilt_deg, float yaw_deg);
/* one-shot events in manual mode */
void lb_ridesim_bump(lb_ridesim *r);
void lb_ridesim_jump(lb_ridesim *r);
/* advance by dt and fill in the IMU sample */
void lb_ridesim_step(lb_ridesim *r, float dt, lb_imu_sample *out);
float lb_ridesim_script_length(const lb_ridesim *r);

#ifdef __cplusplus
}
#endif

#endif
