#include "ridesim.h"

#define D(x) ((x) * LB_DEG)

/* A loop of a twisty road: pull away, sweepers, two knee-down corners, a hard
 * stop, a wheelie, a crest, then a traffic light and a park long enough to fall
 * asleep.  ~112 s. */
static const lb_ride_seg CANYON[] = {
    {4.0f, 0, 0.0f, 0.0f, LB_SEG_ENGINE_OFF},  /* held upright, still: calibrates */
    {2.5f, 0, 0.0f, 0.0f, 0},                  /* engine on */
    {5.0f, 0, 0.75f, 0.0f, 0},                 /* pull away hard */
    {2.0f, 0, 0.30f, 0.0f, 0},
    {1.2f, 0, 0.0f, 0.45f, 0},                 /* brake for the first corner */
    {4.0f, -26, 0.25f, 0.0f, 0},               /* left */
    {2.5f, 0, 0.45f, 0.0f, 0},
    {4.0f, 34, 0.30f, 0.0f, 0},                /* right */
    {1.0f, 0, 0.40f, 0.0f, LB_SEG_BUMP},       /* pothole */
    {3.0f, 0, 0.50f, 0.0f, 0},
    {1.3f, 0, 0.0f, 0.85f, 0},                 /* big stop */
    {4.0f, -44, 0.30f, 0.0f, 0},               /* knee down, left */
    {2.8f, 0, 0.45f, 0.0f, 0},
    {4.0f, 47, 0.30f, 0.0f, 0},                /* knee down, right */
    {3.0f, 0, 0.25f, 0.0f, 0},
    {1.2f, 0, 0.0f, 0.70f, 0},                 /* slow down for the wheelie */
    {1.5f, 0, 0.30f, 0.0f, 0},
    {2.2f, 0, 0.90f, 0.0f, LB_SEG_WHEELIE},    /* wheelie */
    {3.5f, 0, 0.25f, 0.0f, 0},
    {2.5f, 0, 0.25f, 0.0f, LB_SEG_JUMP},       /* crest: airtime */
    {3.5f, -18, 0.20f, 0.0f, 0},               /* sweepers */
    {3.5f, 20, 0.20f, 0.0f, 0},
    {2.0f, 0, 0.20f, 0.0f, 0},
    {4.5f, 0, 0.0f, 0.75f, 0},                 /* stop at the lights */
    {5.0f, 0, 0.0f, 0.0f, 0},                  /* traffic light, engine idling */
    {3.0f, 0, 0.45f, 0.0f, 0},
    {3.0f, 0, 0.0f, 0.40f, 0},
    {35.0f, 0, 0.0f, 0.0f, LB_SEG_ENGINE_OFF}, /* parked: falls asleep */
};

/* Boots while parked on the side stand, so the first calibration is 12 deg off;
 * auto-trim has to fix it while riding straight. */
static const lb_ride_seg SIDESTAND[] = {
    {3.0f, -12, 0.0f, 0.0f, LB_SEG_ENGINE_OFF | LB_SEG_STAND},
    {2.0f, 0, 0.0f, 0.0f, LB_SEG_ENGINE_OFF},  /* rider stands it up */
    {2.0f, 0, 0.0f, 0.0f, 0},
    {4.0f, 0, 0.60f, 0.0f, 0},
    {40.0f, 0, 0.25f, 0.0f, 0},                /* long straight with a little throttle */
    {4.0f, 30, 0.25f, 0.0f, 0},
    {3.0f, 0, 0.25f, 0.0f, 0},
};

#define NSEG(a) ((int)(sizeof(a) / sizeof((a)[0])))

void lb_ridesim_set_mount(lb_ridesim *r, float tilt_deg, float yaw_deg) {
    /* sensor axes in bike coordinates: x = screen right, y = picture top,
     * z = out of the screen towards the rider */
    float a = D(tilt_deg), b = D(yaw_deg);
    lb_vec3 sx = lb_v3(0, -1, 0);
    lb_vec3 sy = lb_v3(lb_sinf(a), 0, lb_cosf(a));
    lb_vec3 sz = lb_v3(-lb_cosf(a), 0, lb_sinf(a));
    float cb = lb_cosf(b), sb = lb_sinf(b);
    lb_vec3 *ax[3] = {&sx, &sy, &sz};
    for (int i = 0; i < 3; i++) {
        lb_vec3 v = *ax[i];
        *ax[i] = lb_v3(v.x * cb - v.y * sb, v.x * sb + v.y * cb, v.z);
    }
    r->fwd_s = lb_v3(sx.x, sy.x, sz.x);
    r->left_s = lb_v3(sx.y, sy.y, sz.y);
    r->up_s = lb_v3(sx.z, sy.z, sz.z);
}

void lb_ridesim_init(lb_ridesim *r, lb_script script, uint32_t seed) {
    lb_ridesim z = {0};
    *r = z;
    r->rng.s = seed ? seed : 1;
    r->bump_t = r->jump_t = -1.0f;
    r->script = script;
    lb_ridesim_set_mount(r, 25.0f, 3.0f);
    /* a cheap MEMS gyro: up to a couple of deg/s of bias per axis */
    r->gyro_bias_dps = lb_v3(lb_rng_range(&r->rng, -2, 2), lb_rng_range(&r->rng, -2, 2),
                             lb_rng_range(&r->rng, -2, 2));
    r->acc_noise_g = 0.008f;
    r->gyro_noise_dps = 0.12f;
    if (script == LB_SCRIPT_CANYON) { r->segs = CANYON; r->nsegs = NSEG(CANYON); }
    else if (script == LB_SCRIPT_SIDESTAND_BOOT) { r->segs = SIDESTAND; r->nsegs = NSEG(SIDESTAND); }
    if (r->segs && (r->segs[0].flags & LB_SEG_STAND)) r->lean = D(r->segs[0].lean_deg);
    r->engine_on = script == LB_SCRIPT_MANUAL;
}

float lb_ridesim_script_length(const lb_ridesim *r) {
    float t = 0.0f;
    for (int i = 0; i < r->nsegs; i++) t += r->segs[i].dur_s;
    return t;
}

void lb_ridesim_bump(lb_ridesim *r) { r->bump_t = 0.0f; }
void lb_ridesim_jump(lb_ridesim *r) { r->jump_t = 0.0f; }

static void apply_script(lb_ridesim *r, float dt, int *stand) {
    *stand = 0;
    if (!r->segs) return;
    r->seg_t += dt;
    while (r->seg_t >= r->segs[r->seg].dur_s) {
        r->seg_t -= r->segs[r->seg].dur_s;
        if (++r->seg >= r->nsegs) { r->seg = 0; r->loops++; }
        int f = r->segs[r->seg].flags;
        if (f & LB_SEG_BUMP) lb_ridesim_bump(r);
        if (f & LB_SEG_JUMP) lb_ridesim_jump(r);
    }
    const lb_ride_seg *s = &r->segs[r->seg];
    r->target_lean_deg = s->lean_deg;
    r->throttle = s->throttle;
    r->brake = s->brake;
    r->wheelie = (s->flags & LB_SEG_WHEELIE) != 0;
    r->engine_on = !(s->flags & LB_SEG_ENGINE_OFF);
    *stand = (s->flags & LB_SEG_STAND) != 0;
}

/* critically-damped second order towards a target, with a rate limit */
static void follow(float *x, float *rate, float target, float omega, float max_rate, float dt) {
    float acc = omega * omega * (target - *x) - 2.0f * omega * *rate;
    *rate = lb_clampf(*rate + acc * dt, -max_rate, max_rate);
    *x += *rate * dt;
}

void lb_ridesim_step(lb_ridesim *r, float dt, lb_imu_sample *out) {
    int stand;
    apply_script(r, dt, &stand);
    r->t += dt;

    /* ---- longitudinal ---- */
    float thr = r->engine_on ? r->throttle : 0.0f;
    float a = thr * 5.5f * (1.0f - r->v / 65.0f) - r->brake * 9.0f - 0.0005f * r->v * r->v;
    if (r->v > 0.0f) a -= 0.15f;
    float v0 = r->v;
    r->v = lb_maxf(0.0f, r->v + a * dt);
    r->lon_acc = (r->v - v0) / dt;

    /* ---- lean: only possible with speed (or on the stand) ---- */
    float tl = r->target_lean_deg;
    if (!stand && r->v < 4.0f) tl *= lb_clampf((r->v - 1.0f) / 3.0f, 0.0f, 1.0f);
    follow(&r->lean, &r->lean_rate, D(tl), 3.5f, D(70.0f), dt);
    r->yaw_rate = r->v > 0.3f ? -LB_G * lb_tanf(r->lean) / lb_maxf(r->v, 3.0f) : 0.0f;

    /* ---- pitch: wheelies, dive under braking, squat under power ---- */
    float tp = r->wheelie && r->v > 3.0f ? D(16.0f) : D(-2.5f * r->brake + 0.6f * thr);
    follow(&r->pitch, &r->pitch_rate, tp, r->wheelie ? 3.0f : 8.0f, D(90.0f), dt);

    /* ---- vertical events ---- */
    r->vert_extra = 0.0f;
    if (r->bump_t >= 0.0f) {
        float t = r->bump_t;
        if (t < 0.07f) r->vert_extra = 16.0f * lb_sinf(LB_PI * t / 0.07f);
        else if (t < 0.2f) r->vert_extra = -6.0f * lb_sinf(LB_PI * (t - 0.07f) / 0.13f);
        if (t < 0.07f) r->pitch_rate += D(300.0f) * dt; /* front wheel kicked up */
        r->bump_t = t + dt < 0.3f ? t + dt : -1.0f;
    }
    if (r->jump_t >= 0.0f) {
        float t = r->jump_t;
        if (t < 0.10f) r->vert_extra = 5.0f * lb_sinf(LB_PI * t / 0.10f);      /* compression */
        else if (t < 0.55f) r->vert_extra = -LB_G;                              /* in the air */
        else if (t < 0.70f) r->vert_extra = 18.0f * lb_sinf(LB_PI * (t - 0.55f) / 0.15f); /* landing */
        r->jump_t = t + dt < 0.8f ? t + dt : -1.0f;
    }

    /* ---- what the sensor feels, bike axes ---- */
    float S = lb_sinf(r->lean), C = lb_cosf(r->lean);
    float sp = lb_sinf(r->pitch), cp = lb_cosf(r->pitch);
    /* specific force in the road frame: forward accel, centripetal, gravity */
    lb_vec3 f = lb_v3(r->lon_acc, r->v * r->yaw_rate, LB_G + r->vert_extra);
    lb_vec3 u = lb_v3(cp * f.x + sp * f.z, f.y, -sp * f.x + cp * f.z);       /* into pitch */
    lb_vec3 fb = lb_v3(u.x, C * u.y + S * u.z, -S * u.y + C * u.z);         /* into roll */
    fb = lb_v3_scale(fb, 1.0f / LB_G);
    /* body rates for yaw -> pitch -> roll */
    lb_vec3 wb = lb_v3(r->lean_rate + r->yaw_rate * sp,
                       -r->pitch_rate * C + r->yaw_rate * S * cp,
                       r->pitch_rate * S + r->yaw_rate * C * cp);
    wb = lb_v3_scale(wb, LB_RAD);

    /* ---- vibration and noise (after the IMU's own low-pass) ---- */
    float amp = r->engine_on ? 0.06f + 0.004f * r->v : 0.0f;
    float freq = 22.0f + 1.3f * r->v;
    r->vib_phase = lb_fractf(r->vib_phase + freq * dt);
    float vib = amp * lb_sinf(LB_TAU * r->vib_phase);
    float road = r->v > 0.5f ? 0.015f * lb_minf(r->v / 20.0f, 1.5f) : 0.0f;
    fb.z += vib + road * lb_rng_gauss(&r->rng);
    fb.x += 0.4f * vib;
    float gv = amp * 12.0f; /* deg/s of vibration on the gyro */
    wb.x += gv * lb_sinf(LB_TAU * r->vib_phase + 1.0f);
    wb.y += gv * lb_sinf(LB_TAU * r->vib_phase + 2.0f);
    wb.z += 0.5f * gv * lb_sinf(LB_TAU * r->vib_phase + 3.0f);

    /* ---- into sensor axes ---- */
    lb_vec3 as = lb_v3_add(lb_v3_add(lb_v3_scale(r->fwd_s, fb.x), lb_v3_scale(r->left_s, fb.y)),
                           lb_v3_scale(r->up_s, fb.z));
    lb_vec3 gs = lb_v3_add(lb_v3_add(lb_v3_scale(r->fwd_s, wb.x), lb_v3_scale(r->left_s, wb.y)),
                           lb_v3_scale(r->up_s, wb.z));
    as = lb_v3_add(as, lb_v3(r->acc_noise_g * lb_rng_gauss(&r->rng), r->acc_noise_g * lb_rng_gauss(&r->rng),
                             r->acc_noise_g * lb_rng_gauss(&r->rng)));
    gs = lb_v3_add(gs, r->gyro_bias_dps);
    gs = lb_v3_add(gs, lb_v3(r->gyro_noise_dps * lb_rng_gauss(&r->rng), r->gyro_noise_dps * lb_rng_gauss(&r->rng),
                             r->gyro_noise_dps * lb_rng_gauss(&r->rng)));
    out->t = r->t;
    out->acc_g = as;
    out->gyro_dps = gs;
}
