/* Renders every expression side by side, with the brain state forced:
 *   gallery OUTDIR   ->  OUTDIR/<name>.ppm for each case */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "brain.h"
#include "face.h"

static lb_color fb[LB_W * LB_H];

static void write_ppm(const char *path, float brightness) {
    FILE *fp = fopen(path, "wb");
    if (!fp) { perror(path); exit(1); }
    fprintf(fp, "P6\n%d %d\n255\n", LB_W, LB_H);
    for (int i = 0; i < LB_W * LB_H; i++) {
        int x = i % LB_W, y = i / LB_W;
        float dx = x + 0.5f - 120.0f, dy = y + 0.5f - 120.0f;
        unsigned char px[3] = {24, 24, 24};
        if (dx * dx + dy * dy <= 120.0f * 120.0f) {
            lb_color c = fb[i];
            px[0] = (unsigned char)(((c >> 11) & 31) * 255 / 31 * brightness);
            px[1] = (unsigned char)(((c >> 5) & 63) * 255 / 63 * brightness);
            px[2] = (unsigned char)((c & 31) * 255 / 31 * brightness);
        }
        fwrite(px, 1, 3, fp);
    }
    fclose(fp);
}

typedef struct {
    const char *name;
    lb_activity act;
    float lean;
    lb_event ev;
    float value;
    int side, level;
    float hold; /* seconds to animate before the snapshot */
    int page;
} scene;

static const scene SCENES[] = {
    {"stopped", LB_ACT_STOPPED, 0, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"cruise", LB_ACT_RIDING, 3, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"lean_18", LB_ACT_RIDING, 18, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"spicy_32", LB_ACT_RIDING, -32, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"knee_45", LB_ACT_RIDING, 45, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"elbow_55", LB_ACT_RIDING, -55, LB_EV_NONE, 0, 0, 0, 1.5f, 0},
    {"launch", LB_ACT_RIDING, 0, LB_EV_LAUNCH, 0.4f, 0, 1, 0.8f, 0},
    {"brake", LB_ACT_RIDING, 0, LB_EV_HARD_BRAKE, 0.6f, 0, 1, 0.5f, 0},
    {"brake_big", LB_ACT_RIDING, 0, LB_EV_HARD_BRAKE, 0.9f, 0, 2, 0.6f, 0},
    {"bump", LB_ACT_RIDING, 0, LB_EV_BUMP, 1.3f, 0, 2, 0.15f, 0},
    {"airtime", LB_ACT_RIDING, 0, LB_EV_AIRTIME, 0.3f, 0, 1, 0.4f, 0},
    {"landing", LB_ACT_RIDING, 0, LB_EV_LANDING, 0.45f, 0, 1, 0.5f, 0},
    {"wheelie", LB_ACT_RIDING, 0, LB_EV_WHEELIE, 1.0f, 0, 1, 0.8f, 0},
    {"wheelie_done", LB_ACT_RIDING, 0, LB_EV_WHEELIE_DONE, 1.8f, 0, 2, 0.9f, 0},
    {"corner", LB_ACT_RIDING, 0, LB_EV_CORNER, 41.0f, 1, 3, 0.9f, 0},
    {"record", LB_ACT_RIDING, 0, LB_EV_RECORD, 47.0f, -1, 1, 0.9f, 0},
    {"tipover", LB_ACT_STOPPED, 70, LB_EV_TIPOVER, 0, 1, 1, 1.0f, 0},
    {"asleep", LB_ACT_ASLEEP, 0, LB_EV_NONE, 0, 0, 0, 3.0f, 0},
    {"wake", LB_ACT_STOPPED, 0, LB_EV_WAKE, 0, 0, 1, 0.25f, 0},
    {"calibrating", LB_ACT_STOPPED, 0, LB_EV_CALIBRATING, 0, 0, 1, 1.0f, 0},
    {"gauge_page", LB_ACT_RIDING, 38, LB_EV_NONE, 0, 0, 0, 1.5f, 1},
    {"stats_page", LB_ACT_RIDING, 0, LB_EV_NONE, 0, 0, 0, 1.0f, 2},
};

int main(int argc, char **argv) {
    const char *out = argc > 1 ? argv[1] : ".";
    for (size_t i = 0; i < sizeof SCENES / sizeof SCENES[0]; i++) {
        const scene *sc = &SCENES[i];
        lb_lean l; lb_lean_init(&l);
        l.cal = LB_CAL_DONE;
        l.lean_deg = sc->lean;
        l.lat_g = 0.6f; l.lon_g = -0.3f; l.vert_g_fast = 1.0f;
        lb_brain b; lb_brain_init(&b, 0);
        b.act = sc->act; b.lean_deg = sc->lean; b.lean_abs = sc->lean < 0 ? -sc->lean : sc->lean;
        b.tier = lb_tier_of(b.lean_abs);
        b.excitement = b.lean_abs / 50.0f;
        b.ride.lean_left_deg = 44; b.ride.lean_right_deg = 47; b.ride.brake_g = 0.81f;
        b.ride.wheelie_s = 1.8f; b.ride.corners = 11;
        b.best.lean_left_deg = 46; b.best.lean_right_deg = 47;
        lb_face f; lb_face_init(&f, 3);
        f.page = (lb_page)sc->page;
        lb_gfx g; lb_gfx_init(&g, fb);
        float dt = 1.0f / 30.0f;
        /* settle on the plain mood first, then start the event */
        for (int k = 0; k < 30; k++) lb_face_update(&f, &b, &l, dt);
        b.ev = sc->ev; b.ev_value = sc->value; b.ev_side = sc->side; b.ev_level = sc->level;
        for (float t = 0; t < sc->hold; t += dt) {
            b.ev_t = t;
            if (sc->act == LB_ACT_ASLEEP) b.act = LB_ACT_ASLEEP;
            lb_face_update(&f, &b, &l, dt);
        }
        lb_face_draw(&f, &b, &l, &g);
        char path[512];
        snprintf(path, sizeof path, "%s/%02d_%s.ppm", out, (int)i, sc->name);
        write_ppm(path, f.brightness);
    }
    return 0;
}
