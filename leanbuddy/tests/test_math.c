#include "lb_math.h"
#include "lbtest.h"

static void test_trig_matches_libm(void) {
    double worst = 0;
    for (float x = -20.0f; x <= 20.0f; x += 0.0137f) {
        worst = fmax(worst, fabs(lb_sinf(x) - sin(x)));
        worst = fmax(worst, fabs(lb_cosf(x) - cos(x)));
    }
    CHECK(worst < 2e-6, "sin/cos worst error %g", worst);
}

static void test_atan2_all_quadrants(void) {
    double worst = 0;
    for (float a = -3.1f; a <= 3.1f; a += 0.01f)
        for (float r = 0.01f; r < 100.0f; r *= 3.7f) {
            float y = r * (float)sin(a), x = r * (float)cos(a);
            worst = fmax(worst, fabs(lb_atan2f(y, x) - atan2(y, x)));
        }
    CHECK(worst < 2e-6, "atan2 worst error %g", worst);
    CHECK_NEAR(lb_atan2f(1, 0), M_PI / 2, 1e-6);
    CHECK_NEAR(lb_atan2f(-1, 0), -M_PI / 2, 1e-6);
    CHECK_NEAR(lb_atan2f(0, -1), M_PI, 1e-6);
}

static void test_atan_tan_acos_exp(void) {
    for (float x = -50.0f; x < 50.0f; x += 0.37f) CHECK_NEAR(lb_atanf(x), atan(x), 2e-6);
    for (float x = -1.4f; x < 1.4f; x += 0.05f) CHECK_NEAR(lb_tanf(x), tan(x), 2e-5 * (1 + fabs(tan(x))));
    for (float x = -1.0f; x <= 1.0f; x += 0.05f) CHECK_NEAR(lb_acosf(x), acos(x), 1e-5);
    for (float x = -20.0f; x < 20.0f; x += 0.3f) CHECK(fabs(lb_expf(x) / exp(x) - 1) < 1e-5, "exp(%g)", x);
}

static void test_wrap_and_rng(void) {
    CHECK_NEAR(lb_wrap_pi(3 * M_PI / 2), -M_PI / 2, 1e-5);
    CHECK_NEAR(lb_wrap_pi(-7.0f), -7.0 + 2 * M_PI, 1e-5);
    lb_rng a = {42}, b = {42};
    double sum = 0;
    for (int i = 0; i < 10000; i++) {
        float x = lb_rng_f(&a);
        CHECK(x >= 0 && x < 1, "rng out of range %g", x);
        CHECK(x == lb_rng_f(&b), "rng not deterministic");
        sum += x;
    }
    CHECK_NEAR(sum / 10000, 0.5, 0.02);
}

int main(void) {
    RUN(test_trig_matches_libm);
    RUN(test_atan2_all_quadrants);
    RUN(test_atan_tan_acos_exp);
    RUN(test_wrap_and_rng);
    DONE();
}
