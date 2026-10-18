/* Lean estimation against the ride simulator's ground truth. */
#include "lean.h"
#include "lbtest.h"
#include "ridesim.h"

typedef struct {
    double rms, max;
    double lat_err_max, lon_err_max;
    lb_lean l;
} run_result;

/* runs a script; errors are measured while riding (engine on, after `from_s`) */
static run_result run(lb_script script, uint32_t seed, float tilt, float yaw, float from_s, float until_s) {
    lb_ridesim r;
    lb_ridesim_init(&r, script, seed);
    lb_ridesim_set_mount(&r, tilt, yaw);
    run_result res = {0};
    lb_lean_init(&res.l);
    double se = 0, lon_ref = 0;
    long n = 0;
    const float dt = 1.0f / 200.0f;
    if (until_s <= 0) until_s = lb_ridesim_script_length(&r);
    while (r.t < until_s) {
        lb_imu_sample s;
        lb_ridesim_step(&r, dt, &s);
        lb_lean_update(&res.l, &s);
        lon_ref += (r.lon_acc / LB_G - lon_ref) * dt / (0.12 + dt); /* same smoothing as the estimate */
        if (r.t < from_s || !r.engine_on || r.v < 3.0f) continue;
        double e = res.l.lean_deg - r.lean * LB_RAD;
        se += e * e;
        n++;
        if (fabs(e) > res.max) res.max = fabs(e);
        /* steady corners: cornering G is tan(lean); straight-line: forward accel */
        if (fabs(r.lean_rate) < 0.02f && fabs(r.lean) > 0.3f)
            res.lat_err_max = fmax(res.lat_err_max, fabs(res.l.lat_g - tan(r.lean)));
        if (fabs(r.lean) < 0.02f && r.bump_t < 0 && r.jump_t < 0 && fabs(r.pitch_rate) < 0.05f)
            res.lon_err_max = fmax(res.lon_err_max, fabs(res.l.lon_g - lon_ref));
    }
    res.rms = n ? sqrt(se / n) : 0;
    return res;
}

static void test_canyon_accuracy(void) {
    run_result r = run(LB_SCRIPT_CANYON, 7, 25, 3, 5, 0);
    printf("  canyon: lean RMS %.2f deg, max %.2f deg; lat G err %.3f; lon G err %.3f\n", r.rms, r.max,
           r.lat_err_max, r.lon_err_max);
    CHECK(r.rms < 1.5, "RMS %.2f", r.rms);
    CHECK(r.max < 5.0, "max %.2f", r.max);
    CHECK(r.lat_err_max < 0.12, "cornering G error %.3f", r.lat_err_max);
    CHECK(r.lon_err_max < 0.12, "forward G error %.3f", r.lon_err_max);
}

static void test_many_seeds_and_mounts(void) {
    const float mounts[][2] = {{0, 0}, {25, 3}, {45, -6}, {60, 8}, {80, 0}, {10, 15}};
    for (unsigned m = 0; m < sizeof mounts / sizeof mounts[0]; m++)
        for (uint32_t seed = 1; seed <= 4; seed++) {
            run_result r = run(LB_SCRIPT_CANYON, seed * 7919u, mounts[m][0], mounts[m][1], 5, 0);
            CHECK(r.rms < 2.5 && r.max < 7.0, "tilt %g yaw %g seed %u: RMS %.2f max %.2f", mounts[m][0],
                  mounts[m][1], seed, r.rms, r.max);
        }
}

/* Why the gyro is needed: mid-corner the accelerometer says "upright" */
static void test_accelerometer_alone_reads_upright_in_a_corner(void) {
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    lb_lean l;
    lb_lean_init(&l);
    float worst_naive = 0, truth = 0, est = 0;
    while (r.t < 32.5f) { /* deep into the 44 deg left-hander */
        lb_imu_sample s;
        lb_ridesim_step(&r, 1.0f / 200.0f, &s);
        lb_lean_update(&l, &s);
        if (r.t > 31.5f) {
            /* naive tilt from the accelerometer, in bike axes (perfect mounting) */
            float ay = lb_v3_dot(s.acc_g, r.left_s), az = lb_v3_dot(s.acc_g, r.up_s);
            float naive = lb_atan2f(ay, az) * LB_RAD;
            if (fabsf(naive) > worst_naive) worst_naive = fabsf(naive);
            truth = r.lean * LB_RAD;
            est = l.lean_deg;
        }
    }
    printf("  mid-corner: true lean %.1f, estimate %.1f, accelerometer-only tilt <= %.1f deg\n", truth, est, worst_naive);
    CHECK(truth < -40.0f, "script should be mid-corner, lean %.1f", truth);
    CHECK(worst_naive < 8.0f, "naive %.1f", worst_naive);
    CHECK_NEAR(est, truth, 3.0);
}

static lb_imu_sample static_sample(const lb_ridesim *r, float t, float roll_deg, float rate_dps) {
    float a = roll_deg * LB_DEG;
    lb_imu_sample s;
    s.t = t;
    s.acc_g = lb_v3_add(lb_v3_scale(r->left_s, lb_sinf(a)), lb_v3_scale(r->up_s, lb_cosf(a)));
    s.gyro_dps = lb_v3_add(r->gyro_bias_dps, lb_v3_scale(r->fwd_s, rate_dps)); /* roll is about "forward" */
    return s;
}

/* hold a roll angle */
static void hold(lb_lean *l, const lb_ridesim *r, float *t, float roll, float secs) {
    for (float end = *t + secs; *t < end; *t += 0.005f) {
        lb_imu_sample s = static_sample(r, *t, roll, 0);
        lb_lean_update(l, &s);
    }
}

/* tilt smoothly from one angle to another, with the gyro seeing the motion */
static void tilt(lb_lean *l, const lb_ridesim *r, float *t, float from, float to, float secs) {
    float t0 = *t;
    for (float end = *t + secs; *t < end; *t += 0.005f) {
        float u = (*t - t0) / secs;
        float roll = from + (to - from) * (u - lb_sinf(LB_TAU * u) / LB_TAU);
        float rate = (to - from) / secs * (1.0f - lb_cosf(LB_TAU * u));
        lb_imu_sample s = static_sample(r, *t, roll, rate);
        lb_lean_update(l, &s);
    }
}

static void test_static_tilt_and_bias(void) {
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_MANUAL, 3);
    lb_lean l;
    lb_lean_init(&l);
    float t = 0;
    hold(&l, &r, &t, 0, 5);
    CHECK(l.cal == LB_CAL_DONE, "calibrated after holding still");
    CHECK(l.still, "still");
    CHECK(lb_v3_len(lb_v3_sub(l.bias, r.gyro_bias_dps)) < 0.1f, "gyro bias learnt");
    /* onto the side stand: right while it moves (gyro), and after */
    tilt(&l, &r, &t, 0, -12, 0.8f);
    CHECK_NEAR(l.lean_deg, -12.0, 1.0);
    hold(&l, &r, &t, -12, 2);
    CHECK_NEAR(l.lean_deg, -12.0, 0.3);
    /* knocked over */
    tilt(&l, &r, &t, -12, 75, 1.0f);
    CHECK_NEAR(l.lean_deg, 75.0, 2.0);
    hold(&l, &r, &t, 75, 3);
    CHECK_NEAR(l.lean_deg, 75.0, 0.5);
}

static void test_recalibration(void) {
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_MANUAL, 5);
    lb_lean l;
    lb_lean_init(&l);
    float t = 0;
    hold(&l, &r, &t, 8, 3);
    CHECK_NEAR(l.lean_deg, 0.0, 0.5); /* calibrated while (unknowingly) tilted 8 deg */
    l.mount_changed = 0;
    tilt(&l, &r, &t, 8, 0, 0.5f);
    lb_lean_request_calibration(&l);
    CHECK(l.cal == LB_CAL_WAITING, "waiting");
    hold(&l, &r, &t, 0, 4);
    CHECK(l.cal == LB_CAL_DONE, "done");
    CHECK(l.mount_changed, "mount_changed tells the firmware to save");
    tilt(&l, &r, &t, 0, 20, 0.6f);
    hold(&l, &r, &t, 20, 2);
    CHECK_NEAR(l.lean_deg, 20.0, 0.5);
}

static void test_side_stand_boot_auto_trim(void) {
    /* boots on the side stand: 12 deg off, fixed by riding straight */
    run_result early = run(LB_SCRIPT_SIDESTAND_BOOT, 11, 25, 3, 6, 12);
    run_result late = run(LB_SCRIPT_SIDESTAND_BOOT, 11, 25, 3, 35, 0);
    printf("  side-stand boot: max error %.1f early, %.1f after 30 s of riding (RMS %.2f)\n", early.max, late.max, late.rms);
    CHECK(early.max > 4.0, "the bad first calibration should show early (%.1f)", early.max);
    CHECK(late.max < 4.0, "auto-trim should have fixed it, max %.2f", late.max);
    CHECK(late.rms < 1.5, "RMS %.2f", late.rms);
}

static void test_still_and_moving(void) {
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    lb_lean l;
    lb_lean_init(&l);
    int moving_while_riding = 0, riding = 0, still_parked = 0, parked = 0;
    float T = lb_ridesim_script_length(&r);
    while (r.t < T) {
        lb_imu_sample s;
        lb_ridesim_step(&r, 1.0f / 200.0f, &s);
        lb_lean_update(&l, &s);
        if (r.v > 5.0f) { riding++; moving_while_riding += l.moving; }
        if (!r.engine_on && r.t > T - 30.0f) { parked++; still_parked += l.still; }
    }
    CHECK(moving_while_riding > riding * 0.97, "moving %d of %d", moving_while_riding, riding);
    CHECK(still_parked > parked * 0.9, "still %d of %d", still_parked, parked);
}

int main(void) {
    RUN(test_canyon_accuracy);
    RUN(test_many_seeds_and_mounts);
    RUN(test_accelerometer_alone_reads_upright_in_a_corner);
    RUN(test_static_tilt_and_bias);
    RUN(test_recalibration);
    RUN(test_side_stand_boot_auto_trim);
    RUN(test_still_and_moving);
    DONE();
}
