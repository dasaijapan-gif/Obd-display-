/* WebAssembly entry points for the browser simulator.
 *
 * The page drives exactly the code the firmware runs: the ride simulator (or the
 * phone's own motion sensors) feeds the lean estimator, the brain and the face,
 * and the face renders into the same RGB565 frame buffer, converted to RGBA here
 * for a <canvas>.
 */
#include "brain.h"
#include "face.h"
#include "ridesim.h"

#define EXPORT(name) __attribute__((export_name(#name)))

#define IMU_HZ 200.0f

static lb_ridesim sim;
static lb_lean lean;
static lb_brain brain;
static lb_face face;
static lb_gfx gfx;
static lb_color fb[LB_W * LB_H];
static uint32_t rgba[LB_W * LB_H];
static float imu_acc;      /* fractional IMU steps owed */
static int external;       /* 1: samples come from sim_feed_imu (phone), not the ride simulator */
static lb_imu_sample last;
static lb_records best;

EXPORT(sim_init) void sim_init(int script, uint32_t seed) {
    lb_ridesim_init(&sim, (lb_script)script, seed);
    lb_lean_init(&lean);
    lb_brain_init(&brain, &best);
    int page = face.page;
    lb_face_init(&face, seed * 2654435761u + 1u);
    face.page = (lb_page)page;
    lb_gfx_init(&gfx, fb);
    imu_acc = 0.0f;
    external = 0;
}

EXPORT(sim_reset_records) void sim_reset_records(void) {
    lb_records z = {0};
    best = z;
    brain.best = z;
    brain.ride = z;
}

/* advance the ride simulator by dt seconds, then the face */
EXPORT(sim_step) void sim_step(float dt) {
    if (dt > 0.1f) dt = 0.1f;
    if (!external) {
        imu_acc += dt * IMU_HZ;
        while (imu_acc >= 1.0f) {
            imu_acc -= 1.0f;
            lb_ridesim_step(&sim, 1.0f / IMU_HZ, &last);
            lb_lean_update(&lean, &last);
            lb_brain_update(&brain, &lean, 1.0f / IMU_HZ);
        }
    }
    lb_face_update(&face, &brain, &lean, dt);
    if (brain.best_dirty) { best = brain.best; brain.best_dirty = 0; }
}

/* phone mode: one real IMU sample (sensor axes: x right, y up, z out of the screen) */
EXPORT(sim_feed_imu) void sim_feed_imu(float t, float ax, float ay, float az, float gx, float gy, float gz) {
    static float last_t = -1.0f;
    external = 1;
    last.t = t;
    last.acc_g = lb_v3(ax, ay, az);
    last.gyro_dps = lb_v3(gx, gy, gz);
    float dt = last_t < 0.0f ? 0.0f : t - last_t;
    last_t = t;
    lb_lean_update(&lean, &last);
    if (dt > 0.0f && dt < 0.2f) lb_brain_update(&brain, &lean, dt);
}

EXPORT(sim_render) uint32_t *sim_render(void) {
    lb_face_draw(&face, &brain, &lean, &gfx);
    for (int i = 0; i < LB_W * LB_H; i++) {
        lb_color c = fb[i];
        uint32_t r = (uint32_t)((c >> 11) & 31) * 255u / 31u;
        uint32_t g = (uint32_t)((c >> 5) & 63) * 255u / 63u;
        uint32_t b = (uint32_t)(c & 31) * 255u / 31u;
        rgba[i] = 0xFF000000u | (b << 16) | (g << 8) | r; /* little-endian RGBA bytes */
    }
    return rgba;
}

/* ---- manual riding ---- */
EXPORT(sim_controls) void sim_controls(float target_lean_deg, float throttle, float brake, int wheelie) {
    sim.target_lean_deg = target_lean_deg;
    sim.throttle = throttle;
    sim.brake = brake;
    sim.wheelie = wheelie;
}
EXPORT(sim_engine) void sim_engine(int on) { sim.engine_on = on; }
EXPORT(sim_bump) void sim_bump(void) { lb_ridesim_bump(&sim); }
EXPORT(sim_jump) void sim_jump(void) { lb_ridesim_jump(&sim); }
EXPORT(sim_set_mount) void sim_set_mount(float tilt, float yaw) { lb_ridesim_set_mount(&sim, tilt, yaw); }

/* ---- buttons ---- */
EXPORT(sim_tap) void sim_tap(void) { lb_face_next_page(&face); }
EXPORT(sim_set_page) void sim_set_page(int p) { face.page = (lb_page)(p % LB_PAGE_COUNT); }
EXPORT(sim_calibrate) void sim_calibrate(void) { lb_lean_request_calibration(&lean); }

/* ---- readouts ---- */
EXPORT(sim_time) float sim_time(void) { return sim.t; }
EXPORT(sim_script_length) float sim_script_length(void) { return lb_ridesim_script_length(&sim); }
EXPORT(sim_true_lean) float sim_true_lean(void) { return sim.lean * LB_RAD; }
EXPORT(sim_true_pitch) float sim_true_pitch(void) { return sim.pitch * LB_RAD; }
EXPORT(sim_speed_kmh) float sim_speed_kmh(void) { return sim.v * 3.6f; }
EXPORT(sim_lean) float sim_lean(void) { return lean.lean_deg; }
EXPORT(sim_pitch) float sim_pitch(void) { return lean.pitch_deg; }
EXPORT(sim_lon_g) float sim_lon_g(void) { return lean.lon_g; }
EXPORT(sim_lat_g) float sim_lat_g(void) { return lean.lat_g; }
EXPORT(sim_vert_g) float sim_vert_g(void) { return lean.vert_g_fast; }
EXPORT(sim_accel_only_lean) float sim_accel_only_lean(void) {
    /* what a naive accelerometer tilt would say, in bike axes */
    float ay = lb_v3_dot(last.acc_g, sim.left_s), az = lb_v3_dot(last.acc_g, sim.up_s);
    return lb_atan2f(ay, az) * LB_RAD;
}
EXPORT(sim_event) int sim_event(void) { return (int)brain.ev; }
EXPORT(sim_activity) int sim_activity(void) { return (int)brain.act; }
EXPORT(sim_tier) int sim_tier(void) { return (int)brain.tier; }
EXPORT(sim_cal) int sim_cal(void) { return (int)lean.cal; }
EXPORT(sim_brightness) float sim_brightness(void) { return face.brightness; }
EXPORT(sim_page) int sim_page(void) { return (int)face.page; }
EXPORT(sim_excitement) float sim_excitement(void) { return brain.excitement; }
EXPORT(sim_turn_conf) float sim_turn_conf(void) { return lean.turn_conf; }
EXPORT(sim_engine_on) int sim_engine_on(void) { return sim.engine_on; }
EXPORT(sim_segment) int sim_segment(void) { return sim.seg; }
EXPORT(sim_ride_left) float sim_ride_left(void) { return brain.ride.lean_left_deg; }
EXPORT(sim_ride_right) float sim_ride_right(void) { return brain.ride.lean_right_deg; }
EXPORT(sim_best_left) float sim_best_left(void) { return best.lean_left_deg; }
EXPORT(sim_best_right) float sim_best_right(void) { return best.lean_right_deg; }
EXPORT(sim_corners) int sim_corners(void) { return brain.ride.corners; }
