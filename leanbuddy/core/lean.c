#include "lean.h"

/* stillness: parked with the engine off, or the bike on the bench */
#define STILL_GYRO_DEV_DPS 0.8f  /* RMS gyro wobble */
#define STILL_GYRO_MAG_DPS 6.0f  /* mean rate after bias: cheap gyros start up to ~3 deg/s off */
#define STILL_ACC_DEV_G 0.02f    /* RMS accel wobble: an idling engine is well above this */
#define STILL_ACC_NORM_G 0.06f   /* |a| must be ~1 g */
#define CAL_STILL_S 1.0f         /* still this long before (re)calibrating */
#define QUIET_GYRO_DEV_DPS 4.0f  /* engine idling but the bike not moving */
#define CAL_QUIET_S 3.0f         /* ... this long also calibrates */
#define BIAS_STILL_S 0.3f
#define BIAS_TAU_S 1.0f

/* attitude filter */
#define TURN_TAU_S 0.8f     /* pull towards the cornering reference */
#define ACC_TAU_S 0.8f      /* pull towards the accelerometer when going straight */
#define STILL_TAU_S 0.15f   /* ... and when still: the accelerometer is exact then */
#define LEAK_TAU_S 20.0f    /* no reference at all: drift back to upright, very slowly */
#define PITCH_LEAK_TAU_S 4.0f
#define TURN_MIN_DPS 2.5f   /* yaw rates for the cornering reference to start / be trusted */
#define TURN_FULL_DPS 8.0f
#define MAX_TURN_REF_RAD (70.0f * LB_DEG)

/* auto-trim: riding straight averages to upright */
#define TRIM_WINDOW_S 3.0f
#define TRIM_GAIN 0.5f
#define TRIM_MAX_RAD (15.0f * LB_DEG) /* side stands are ~12 deg */
#define TRIM_MIN_VIB_G 0.03f /* engine running */

/* auto-forward: straight-line acceleration and braking point along the bike */
#define FWD_MIN_G 0.12f
#define FWD_WINDOW_GS 1.5f   /* g x seconds of evidence per correction */
#define FWD_GAIN 0.5f
#define FWD_MAX_RAD (40.0f * LB_DEG)

#define ACTIVITY_HOLD_S 30.0f

static lb_vec3 lp3(lb_vec3 y, lb_vec3 x, float a) {
    return lb_v3_add(y, lb_v3_scale(lb_v3_sub(x, y), a));
}

static lb_vec3 to_bike(const lb_lean *l, lb_vec3 v) {
    return lb_v3(lb_v3_dot(l->rx, v), lb_v3_dot(l->ry, v), lb_v3_dot(l->rz, v));
}

static void rebuild_axes(lb_lean *l) {
    l->rz = l->mount.up;
    l->rx = l->mount.fwd;
    l->ry = lb_v3_cross(l->rz, l->rx); /* z x x = y (left) */
}

static lb_vec3 reject(lb_vec3 v, lb_vec3 unit) {
    return lb_v3_sub(v, lb_v3_scale(unit, lb_v3_dot(v, unit)));
}

/* Forward from the screen's orientation: the rider looks at the screen, so the
 * screen's "right" edge direction is the bike's right, whatever the tilt. */
static lb_mount mount_from(lb_vec3 up, lb_vec3 normal, lb_vec3 top) {
    lb_mount m;
    m.up = lb_v3_norm(up);
    lb_vec3 right = lb_v3_cross(top, normal);
    lb_vec3 rh = reject(right, m.up);
    if (lb_v3_len(rh) > 0.3f) {
        m.fwd = lb_v3_norm(lb_v3_cross(m.up, lb_v3_norm(rh))); /* left x up = fwd, left = -right */
    } else {
        /* screen turned on its side: fall back to "top and away from the rider" */
        lb_vec3 f = reject(lb_v3_sub(top, normal), m.up);
        m.fwd = lb_v3_len(f) > 1e-3f ? lb_v3_norm(f) : lb_v3_norm(reject(lb_v3(1, 0, 0), m.up));
    }
    m.valid = 1;
    return m;
}

void lb_lean_init(lb_lean *l) {
    lb_lean z = {0};
    *l = z;
    l->screen_normal = lb_v3(0, 0, 1);
    l->screen_top = lb_v3(0, 1, 0);
    l->auto_trim = 1;
    l->acc_lp = l->acc_slow = lb_v3(0, 0, 1);
    l->lin_lp = lb_v3(0, 0, 1);
    l->vert_g = l->vert_g_fast = l->vert_fast_lp = 1.0f;
    l->since_activity_s = ACTIVITY_HOLD_S;
    l->cal = LB_CAL_NONE;
}

void lb_lean_set_orientation(lb_lean *l, lb_vec3 screen_normal, lb_vec3 screen_top) {
    l->screen_normal = lb_v3_norm(screen_normal);
    l->screen_top = lb_v3_norm(screen_top);
}

void lb_lean_set_mount(lb_lean *l, const lb_mount *m) {
    if (!m->valid) return;
    l->mount = *m;
    l->mount.up = lb_v3_norm(m->up);
    l->mount.fwd = lb_v3_norm(reject(m->fwd, l->mount.up));
    rebuild_axes(l);
    l->cal = LB_CAL_DONE;
}

void lb_lean_request_calibration(lb_lean *l) {
    l->cal = LB_CAL_WAITING;
}

static void auto_trim(lb_lean *l, lb_vec3 alp, float dt, int straight) {
    if (!straight) return;
    l->trim_ay += alp.y * dt;
    l->trim_az += alp.z * dt;
    l->trim_w += dt;
    if (l->trim_w < TRIM_WINDOW_S) return;
    float err = lb_atan2f(l->trim_ay, l->trim_az); /* apparent lean while really upright */
    l->trim_ay = l->trim_az = l->trim_w = 0.0f;
    if (lb_absf(err) > TRIM_MAX_RAD || lb_absf(err) < 0.2f * LB_DEG) return;
    float step = err * TRIM_GAIN;
    /* true up has a component along "left" when the bike looks leaned right */
    l->mount.up = lb_v3_norm(lb_v3_add(l->mount.up, lb_v3_scale(l->ry, lb_tanf(step))));
    l->mount.fwd = lb_v3_norm(reject(l->mount.fwd, l->mount.up));
    rebuild_axes(l);
    l->roll -= step; /* the frame moved under the estimate */
    l->mount_changed = 1;
}

/* The screen-based guess of "forward" is off if the buddy is mounted turned a
 * little.  Accelerating and braking in a straight line push along the bike's
 * forward axis, so their direction (sign-corrected) refines it. */
static void auto_forward(lb_lean *l, float dt, int straight) {
    if (!straight) return;
    lb_vec3 h = reject(l->acc_lp, l->mount.up);
    float hl = lb_v3_len(h);
    if (hl < FWD_MIN_G) return;
    float sgn = lb_v3_dot(h, l->mount.fwd) < 0.0f ? -1.0f : 1.0f;
    l->fwd_sum = lb_v3_add(l->fwd_sum, lb_v3_scale(h, sgn * dt));
    l->fwd_w += hl * dt;
    if (l->fwd_w < FWD_WINDOW_GS) return;
    lb_vec3 f = lb_v3_norm(reject(l->fwd_sum, l->mount.up));
    l->fwd_sum = lb_v3(0, 0, 0);
    l->fwd_w = 0.0f;
    float ang = lb_acosf(lb_v3_dot(f, l->mount.fwd));
    if (ang > FWD_MAX_RAD || ang < 0.3f * LB_DEG) return;
    lb_vec3 nf = lb_v3_add(l->mount.fwd, lb_v3_scale(lb_v3_sub(f, l->mount.fwd), FWD_GAIN));
    l->mount.fwd = lb_v3_norm(reject(nf, l->mount.up));
    rebuild_axes(l);
    l->mount_changed = 1;
}

void lb_lean_update(lb_lean *l, const lb_imu_sample *s) {
    float dt = l->have_t ? s->t - l->last_t : 0.005f;
    l->last_t = s->t;
    if (!l->have_t) { /* start the filters from the first reading, not from zero */
        l->acc_lp = l->acc_slow = s->acc_g;
        l->gyro_lp = s->gyro_dps;
    }
    l->have_t = 1;
    if (dt <= 0.0f) return;
    if (dt > 0.1f) dt = 0.1f; /* a stall in the sample stream: do not integrate a jump */

    /* ---- signal statistics, sensor frame ---- */
    lb_vec3 acc = s->acc_g;
    lb_vec3 raw_gyro = s->gyro_dps;
    l->acc_lp = lp3(l->acc_lp, acc, lb_lp_alpha(dt, 0.08f));
    l->acc_slow = lp3(l->acc_slow, acc, lb_lp_alpha(dt, 0.5f));
    lb_vec3 adev = lb_v3_sub(acc, l->acc_slow);
    l->acc_dev2 += (lb_v3_dot(adev, adev) - l->acc_dev2) * lb_lp_alpha(dt, 0.5f);
    l->gyro_lp = lp3(l->gyro_lp, raw_gyro, lb_lp_alpha(dt, 0.5f));
    lb_vec3 gdev = lb_v3_sub(raw_gyro, l->gyro_lp);
    l->gyro_dev2 += (lb_v3_dot(gdev, gdev) - l->gyro_dev2) * lb_lp_alpha(dt, 0.5f);
    /* vibration: what is left after the 0.08 s low-pass (an engine, a rough road),
     * unlike the slower wobble above, which any movement causes */
    lb_vec3 vdev = lb_v3_sub(acc, l->acc_lp);
    l->vib2 += (lb_v3_dot(vdev, vdev) - l->vib2) * lb_lp_alpha(dt, 0.5f);
    l->vib_g = lb_sqrtf(l->vib2);

    l->still = lb_sqrtf(l->gyro_dev2) < STILL_GYRO_DEV_DPS && lb_sqrtf(l->acc_dev2) < STILL_ACC_DEV_G &&
               lb_absf(lb_v3_len(l->acc_slow) - 1.0f) < STILL_ACC_NORM_G &&
               lb_v3_len(lb_v3_sub(l->gyro_lp, l->bias)) < STILL_GYRO_MAG_DPS;
    l->still_s = l->still ? l->still_s + dt : 0.0f;
    /* "quiet": the engine may be running, but the bike is not moving */
    int quiet = lb_sqrtf(l->gyro_dev2) < QUIET_GYRO_DEV_DPS &&
                lb_absf(lb_v3_len(l->acc_slow) - 1.0f) < STILL_ACC_NORM_G &&
                lb_v3_len(lb_v3_sub(l->gyro_lp, l->bias)) < STILL_GYRO_MAG_DPS;
    l->quiet_s = quiet ? l->quiet_s + dt : 0.0f;
    if (l->still_s > BIAS_STILL_S) l->bias = lp3(l->bias, l->gyro_lp, lb_lp_alpha(dt, BIAS_TAU_S));

    /* ---- calibration: "up" is wherever gravity points when held still ---- */
    if ((l->cal != LB_CAL_DONE || !l->mount.valid) &&
        (l->still_s > CAL_STILL_S || l->quiet_s > CAL_QUIET_S)) {
        l->mount = mount_from(l->acc_slow, l->screen_normal, l->screen_top);
        rebuild_axes(l);
        l->roll = l->pitch = 0.0f;
        l->cal = LB_CAL_DONE;
        l->mount_changed = 1;
    }
    if (!l->mount.valid) return;

    /* ---- bike frame ---- */
    lb_vec3 w = lb_v3_scale(to_bike(l, lb_v3_sub(raw_gyro, l->bias)), LB_DEG); /* rad/s */
    lb_vec3 a = to_bike(l, acc);
    lb_vec3 alp = to_bike(l, l->acc_lp);

    float S = lb_sinf(l->roll), C = lb_cosf(l->roll);
    float sp = lb_sinf(l->pitch), cp = lb_cosf(l->pitch);
    if (cp < 0.2f) cp = 0.2f;

    /* Euler rates from body rates (yaw about z, then pitch, then roll) */
    float yawc = w.y * S + w.z * C; /* yaw rate x cos(pitch) */
    float roll_dot = w.x - yawc * sp / cp;
    float pitch_dot = w.z * S - w.y * C;
    l->roll += roll_dot * dt;
    l->pitch += pitch_dot * dt;

    /* ---- drift references ---- */
    /* cornering: the rotation axis is the world vertical; in bike axes it leans by
     * the lean angle, so pitch-rate / yaw-rate = tan(lean).  Filtered rates. */
    float yaw_dps = lb_absf(yawc) * LB_RAD;
    float w_turn = lb_smoothstep(TURN_MIN_DPS, TURN_FULL_DPS, yaw_dps) *
                   (1.0f - lb_smoothstep(6.0f, 25.0f, lb_absf(roll_dot) * LB_RAD)) * /* steady, not flicking */
                   (1.0f - lb_smoothstep(10.0f, 30.0f, lb_absf(pitch_dot) * LB_RAD)); /* not a bump/wheelie */
    /* the ratio uses filtered rates, so vibration on q and r averages out */
    l->turn_q += (w.y - l->turn_q) * lb_lp_alpha(dt, 0.08f);
    l->turn_r += (w.z - l->turn_r) * lb_lp_alpha(dt, 0.08f);
    float phi_turn = 0.0f;
    if (lb_absf(l->turn_r) > 1e-4f) phi_turn = lb_atanf(l->turn_q / l->turn_r);
    if (lb_absf(phi_turn) > MAX_TURN_REF_RAD) w_turn = 0.0f;
    /* trust builds up over a moment but is dropped at once when the bike rolls */
    if (w_turn < l->turn_conf) l->turn_conf = w_turn;
    else l->turn_conf += (w_turn - l->turn_conf) * lb_lp_alpha(dt, 0.2f);
    w_turn = l->turn_conf;

    /* straight or still: the accelerometer's tilt */
    float anorm_err = lb_absf(lb_v3_len(alp) - 1.0f);
    float w_acc = (1.0f - lb_smoothstep(1.0f, 3.0f, yaw_dps)) *
                  (1.0f - lb_smoothstep(0.04f, 0.15f, anorm_err)) *
                  (1.0f - lb_smoothstep(10.0f, 40.0f, lb_absf(roll_dot) * LB_RAD)); /* gyro owns fast rolls */
    float phi_acc = lb_atan2f(alp.y, alp.z);
    float acc_tau = ACC_TAU_S;
    if (l->still) { w_acc = 1.0f; acc_tau = STILL_TAU_S; }

    float k = w_turn / TURN_TAU_S + w_acc / acc_tau;
    if (k > 0.0f) {
        float target = (w_turn / TURN_TAU_S * phi_turn + w_acc / acc_tau * phi_acc) / k;
        l->roll += (target - l->roll) * lb_minf(k * dt, 1.0f);
    }
    if (w_turn + w_acc < 0.05f) l->roll -= l->roll * dt / LEAK_TAU_S;
    l->roll = lb_clampf(l->roll, -1.4f, 1.4f);

    /* pitch: exact from the accelerometer when still; while riding, forward
     * acceleration masquerades as pitch, so it is only re-zeroed slowly (road-relative) */
    if (l->still) {
        float th = lb_atan2f(alp.x, lb_sqrtf(alp.y * alp.y + alp.z * alp.z));
        l->pitch += (th - l->pitch) * lb_minf(dt / STILL_TAU_S, 1.0f);
    } else {
        l->pitch -= l->pitch * dt / PITCH_LEAK_TAU_S;
    }
    l->pitch = lb_clampf(l->pitch, -1.2f, 1.2f);

    /* ---- accelerations in the road plane ---- */
    S = lb_sinf(l->roll); C = lb_cosf(l->roll);
    sp = lb_sinf(l->pitch); cp = lb_cosf(l->pitch);
    lb_vec3 v1 = lb_v3(a.x, C * a.y - S * a.z, S * a.y + C * a.z);      /* undo roll */
    lb_vec3 lev = lb_v3(cp * v1.x - sp * v1.z, v1.y, sp * v1.x + cp * v1.z); /* undo pitch */
    lb_vec3 lin = lb_v3(lev.x, -lev.y, lev.z); /* lon, lat (+ right), vert */
    l->lin_lp = lp3(l->lin_lp, lin, lb_lp_alpha(dt, 0.12f));
    l->vert_fast_lp += (lev.z - l->vert_fast_lp) * lb_lp_alpha(dt, 0.015f);

    l->lean_deg = l->roll * LB_RAD;
    l->pitch_deg = l->pitch * LB_RAD;
    l->lean_rate_dps = roll_dot * LB_RAD;
    l->yaw_rate_dps = yawc / cp * LB_RAD;
    l->lon_g = l->lin_lp.x;
    l->lat_g = l->lin_lp.y;
    l->vert_g = l->lin_lp.z;
    l->vert_g_fast = l->vert_fast_lp;

    /* ---- riding or not ---- */
    if (yaw_dps > 3.0f || lb_absf(l->lon_g) > 0.08f || lb_absf(l->lean_rate_dps) > 5.0f)
        l->since_activity_s = 0.0f;
    else
        l->since_activity_s += dt;
    /* There is no speedometer: a steady cruise looks much like idling at a light.
     * So "moving" holds for a while after the last sign of riding. */
    l->moving = !l->still && (l->since_activity_s < ACTIVITY_HOLD_S || !quiet);

    if (l->auto_trim) {
        /* riding, engine running, going straight: on average the bike is upright */
        int straight = l->moving && l->vib_g > TRIM_MIN_VIB_G && w_acc > 0.9f && yaw_dps < 1.5f &&
                       lb_absf(w.x) * LB_RAD < 3.0f;
        auto_trim(l, alp, dt, straight);
        /* rates only: no yaw means upright on a moving bike, and the lean estimate
         * itself is what a wrong "forward" spoils */
        int straight_line = !l->still && yaw_dps < 1.5f && lb_absf(w.x) * LB_RAD < 3.0f &&
                            lb_absf(pitch_dot) * LB_RAD < 4.0f;
        auto_forward(l, dt, straight_line);
    }
}
