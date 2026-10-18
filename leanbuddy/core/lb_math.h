/* Small, dependency-free maths for LeanBuddy.
 *
 * The core is built three ways: for the ESP32-S3 firmware, natively for the tests,
 * and as freestanding WebAssembly for the browser simulator (no libc, no libm).
 * So it carries its own trig.  The approximations are accurate to ~1e-6 rad, far
 * below anything a lean angle or an animation can show, and they make all three
 * builds produce the same numbers.
 */
#ifndef LB_MATH_H
#define LB_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LB_PI 3.14159265358979f
#define LB_TAU 6.28318530717959f
#define LB_DEG (LB_PI / 180.0f)  /* degrees -> radians: 45 * LB_DEG */
#define LB_RAD (180.0f / LB_PI)  /* radians -> degrees */
#define LB_G 9.80665f            /* m/s^2 per g */

typedef struct { float x, y, z; } lb_vec3;

static inline float lb_sqrtf(float x) { return x > 0.0f ? __builtin_sqrtf(x) : 0.0f; }
static inline float lb_absf(float x) { return x < 0.0f ? -x : x; }
static inline float lb_minf(float a, float b) { return a < b ? a : b; }
static inline float lb_maxf(float a, float b) { return a > b ? a : b; }
static inline float lb_clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float lb_signf(float x) { return x < 0.0f ? -1.0f : 1.0f; }
static inline float lb_lerpf(float a, float b, float t) { return a + (b - a) * t; }
/* 0 below e0, 1 above e1, smooth in between */
static inline float lb_smoothstep(float e0, float e1, float x) {
    float t = lb_clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
static inline float lb_floorf(float x) {
    float t = (float)(int32_t)x;
    return t > x ? t - 1.0f : t;
}
static inline float lb_fractf(float x) { return x - lb_floorf(x); }

float lb_sinf(float x);
float lb_cosf(float x);
float lb_tanf(float x);
float lb_atanf(float x);
float lb_atan2f(float y, float x);
float lb_acosf(float x);
float lb_expf(float x);
/* wraps an angle to [-pi, pi) */
float lb_wrap_pi(float a);

/* first-order low-pass coefficient for time constant tau at step dt */
static inline float lb_lp_alpha(float dt, float tau) { return tau <= 0.0f ? 1.0f : dt / (tau + dt); }

static inline lb_vec3 lb_v3(float x, float y, float z) { lb_vec3 v = {x, y, z}; return v; }
static inline lb_vec3 lb_v3_add(lb_vec3 a, lb_vec3 b) { return lb_v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline lb_vec3 lb_v3_sub(lb_vec3 a, lb_vec3 b) { return lb_v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline lb_vec3 lb_v3_scale(lb_vec3 a, float s) { return lb_v3(a.x * s, a.y * s, a.z * s); }
static inline float lb_v3_dot(lb_vec3 a, lb_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline lb_vec3 lb_v3_cross(lb_vec3 a, lb_vec3 b) {
    return lb_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline float lb_v3_len(lb_vec3 a) { return lb_sqrtf(lb_v3_dot(a, a)); }
static inline lb_vec3 lb_v3_norm(lb_vec3 a) {
    float l = lb_v3_len(a);
    return l > 1e-9f ? lb_v3_scale(a, 1.0f / l) : lb_v3(0.0f, 0.0f, 1.0f);
}

/* Small deterministic PRNG (xorshift32) so animations and the ride simulator are
 * reproducible on every build. */
typedef struct { uint32_t s; } lb_rng;
static inline uint32_t lb_rng_u32(lb_rng *r) {
    uint32_t x = r->s ? r->s : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    r->s = x;
    return x;
}
/* uniform in [0, 1) */
static inline float lb_rng_f(lb_rng *r) { return (float)(lb_rng_u32(r) >> 8) * (1.0f / 16777216.0f); }
/* uniform in [lo, hi) */
static inline float lb_rng_range(lb_rng *r, float lo, float hi) { return lo + (hi - lo) * lb_rng_f(r); }
/* roughly normal, mean 0, std 1 (sum of uniforms) */
static inline float lb_rng_gauss(lb_rng *r) {
    return (lb_rng_f(r) + lb_rng_f(r) + lb_rng_f(r) + lb_rng_f(r) - 2.0f) * 1.7320508f;
}

#ifdef __cplusplus
}
#endif

#endif
