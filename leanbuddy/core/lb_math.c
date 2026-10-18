#include "lb_math.h"

float lb_wrap_pi(float a) {
    return a - LB_TAU * lb_floorf((a + LB_PI) / LB_TAU);
}

float lb_sinf(float x) {
    x = lb_wrap_pi(x);
    /* fold into [-pi/2, pi/2] where the series converges fast */
    if (x > 0.5f * LB_PI) x = LB_PI - x;
    else if (x < -0.5f * LB_PI) x = -LB_PI - x;
    float x2 = x * x;
    /* Taylor to x^11: error < 6e-8 on [-pi/2, pi/2] */
    return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f *
           (1.0f - x2 / 72.0f * (1.0f - x2 / 110.0f)))));
}

float lb_cosf(float x) { return lb_sinf(x + 0.5f * LB_PI); }

float lb_tanf(float x) {
    float c = lb_cosf(x);
    if (lb_absf(c) < 1e-7f) c = c < 0.0f ? -1e-7f : 1e-7f;
    return lb_sinf(x) / c;
}

float lb_atanf(float x) {
    float sign = 1.0f;
    if (x < 0.0f) { x = -x; sign = -1.0f; }
    float offset = 0.0f;
    if (x > 1.0f) { x = 1.0f / x; offset = 0.5f * LB_PI; sign = -sign; }
    /* half-angle: atan(x) = 2 atan(x / (1 + sqrt(1 + x^2))), brings x below 0.415 */
    x = x / (1.0f + lb_sqrtf(1.0f + x * x));
    float x2 = x * x, term = x, sum = 0.0f;
    for (int n = 1; n <= 15; n += 2) {
        sum += term / (float)n;
        term *= -x2;
    }
    float r = 2.0f * sum;
    /* for |x| > 1 the result is pi/2 - atan(1/x); sign was flipped above to subtract */
    return offset > 0.0f ? -sign * (offset - r) : sign * r;
}

float lb_atan2f(float y, float x) {
    if (x > 0.0f) return lb_atanf(y / x);
    if (x < 0.0f) return lb_atanf(y / x) + (y >= 0.0f ? LB_PI : -LB_PI);
    if (y > 0.0f) return 0.5f * LB_PI;
    if (y < 0.0f) return -0.5f * LB_PI;
    return 0.0f;
}

float lb_acosf(float x) {
    x = lb_clampf(x, -1.0f, 1.0f);
    return lb_atan2f(lb_sqrtf(1.0f - x * x), x);
}

float lb_expf(float x) {
    x = lb_clampf(x, -80.0f, 80.0f);
    /* x = k ln2 + r, |r| <= ln2/2 */
    float kf = lb_floorf(x * 1.44269504f + 0.5f);
    float r = x - kf * 0.69314718f;
    float p = 1.0f + r * (1.0f + r / 2.0f * (1.0f + r / 3.0f * (1.0f + r / 4.0f *
              (1.0f + r / 5.0f * (1.0f + r / 6.0f * (1.0f + r / 7.0f))))));
    union { float f; int32_t i; } u;
    u.i = ((int32_t)kf + 127) << 23; /* 2^k */
    return p * u.f;
}

#if defined(__wasm__)
/* Freestanding WebAssembly has no libc; the compiler still emits calls to these
 * for struct copies and zeroing. */
void *memset(void *d, int c, unsigned long n) {
    unsigned char *p = (unsigned char *)d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}
void *memcpy(void *d, const void *s, unsigned long n) {
    unsigned char *p = (unsigned char *)d;
    const unsigned char *q = (const unsigned char *)s;
    while (n--) *p++ = *q++;
    return d;
}
void *memmove(void *d, const void *s, unsigned long n) {
    unsigned char *p = (unsigned char *)d;
    const unsigned char *q = (const unsigned char *)s;
    if (p < q) { while (n--) *p++ = *q++; }
    else { p += n; q += n; while (n--) *--p = *--q; }
    return d;
}
#endif
