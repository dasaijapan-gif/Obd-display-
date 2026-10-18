/* Rendering: stays inside the frame buffer, is deterministic, and is fast enough. */
#include <string.h>
#include <time.h>
#include "face.h"
#include "lbtest.h"
#include "ridesim.h"

#define GUARD 4096
static lb_color mem[GUARD + LB_W * LB_H + GUARD];

static uint32_t hash_frame(const lb_color *fb) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < LB_W * LB_H; i++) h = (h ^ fb[i]) * 16777619u;
    return h;
}

/* runs the canyon ride, drawing every frame; returns a hash of all frames */
static uint32_t ride_and_draw(int page, double *ms_per_frame, int *frames, int *guard_ok) {
    for (int i = 0; i < GUARD; i++) mem[i] = mem[GUARD + LB_W * LB_H + i] = 0xBEEF;
    lb_color *fb = mem + GUARD;
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    lb_lean l;
    lb_lean_init(&l);
    lb_brain b;
    lb_brain_init(&b, 0);
    lb_face f;
    lb_face_init(&f, 1);
    f.page = (lb_page)page;
    lb_gfx g;
    lb_gfx_init(&g, fb);
    uint32_t h = 0;
    int n = 0, sub = 0;
    double draw_s = 0;
    const float dt = 1.0f / 200.0f;
    float T = lb_ridesim_script_length(&r);
    while (r.t < T) {
        lb_imu_sample s;
        lb_ridesim_step(&r, dt, &s);
        lb_lean_update(&l, &s);
        lb_brain_update(&b, &l, dt);
        if (++sub % 6) continue; /* ~33 fps */
        lb_face_update(&f, &b, &l, dt * 6);
        clock_t c0 = clock();
        lb_face_draw(&f, &b, &l, &g);
        draw_s += (double)(clock() - c0) / CLOCKS_PER_SEC;
        h = h * 31u + hash_frame(fb);
        n++;
    }
    *guard_ok = 1;
    for (int i = 0; i < GUARD; i++)
        if (mem[i] != 0xBEEF || mem[GUARD + LB_W * LB_H + i] != 0xBEEF) *guard_ok = 0;
    *ms_per_frame = 1000.0 * draw_s / n;
    *frames = n;
    return h;
}

static void test_pages_draw_inside_buffer_and_deterministic(void) {
    for (int page = 0; page < LB_PAGE_COUNT; page++) {
        double ms;
        int n, ok;
        uint32_t h1 = ride_and_draw(page, &ms, &n, &ok);
        CHECK(ok, "page %d wrote outside the frame buffer", page);
        uint32_t h2 = ride_and_draw(page, &ms, &n, &ok);
        CHECK(h1 == h2, "page %d not deterministic", page);
        printf("  page %d: %d frames, %.2f ms/frame on this machine\n", page, n, ms);
    }
}

static void test_face_is_not_blank_and_glyphs_exist(void) {
    lb_color *fb = mem + GUARD;
    lb_gfx g;
    lb_gfx_init(&g, fb);
    lb_gfx_clear(&g, 0);
    const char *all = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.!-:/<>?%'+*";
    char one[2] = {0, 0};
    for (const char *p = all; *p; p++) {
        lb_gfx_clear(&g, 0);
        one[0] = *p;
        lb_draw_text_c(&g, one, 120, 120, 40, 5, 0xFFFF, 1);
        int lit = 0;
        for (int i = 0; i < LB_W * LB_H; i++) lit += fb[i] != 0;
        CHECK(lit > 15, "glyph '%c' draws nothing (%d px)", *p, lit);
    }
    CHECK(lb_text_width("NEW BEST!", 17) < 200, "fits the round screen");
}

static void test_color_mix(void) {
    lb_color a = lb_rgb(0, 0, 0), b = lb_rgb(248, 252, 248);
    CHECK(lb_color_mix(a, b, 0) == a, "t=0");
    CHECK(lb_color_mix(a, b, 1) == b, "t=1");
    lb_color m = lb_color_mix(a, b, 0.5f);
    CHECK(((m >> 11) & 31) >= 15 && ((m >> 11) & 31) <= 16, "half red %d", (m >> 11) & 31);
}

int main(void) {
    RUN(test_pages_draw_inside_buffer_and_deterministic);
    RUN(test_face_is_not_blank_and_glyphs_exist);
    RUN(test_color_mix);
    DONE();
}
