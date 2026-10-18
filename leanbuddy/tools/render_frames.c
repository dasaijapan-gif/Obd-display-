/* Renders frames of the scripted canyon ride to PPM images, for screenshots and
 * eyeballing animations without hardware:  render_frames OUTDIR t1 t2 ...
 * Each frame is written as OUTDIR/frame_<t>.ppm (t in tenths of a second). */
#include <stdio.h>
#include <stdlib.h>
#include "brain.h"
#include "face.h"
#include "ridesim.h"

static lb_color fb[LB_W * LB_H];

static void write_ppm(const char *path, float brightness) {
    FILE *fp = fopen(path, "wb");
    if (!fp) { perror(path); exit(1); }
    fprintf(fp, "P6\n%d %d\n255\n", LB_W, LB_H);
    for (int i = 0; i < LB_W * LB_H; i++) {
        int x = i % LB_W, y = i / LB_W;
        float dx = x + 0.5f - 120.0f, dy = y + 0.5f - 120.0f;
        unsigned char px[3] = {0, 0, 0};
        if (dx * dx + dy * dy <= 120.0f * 120.0f) { /* the round panel */
            lb_color c = fb[i];
            px[0] = (unsigned char)(((c >> 11) & 31) * 255 / 31 * brightness);
            px[1] = (unsigned char)(((c >> 5) & 63) * 255 / 63 * brightness);
            px[2] = (unsigned char)((c & 31) * 255 / 31 * brightness);
        } else {
            px[0] = px[1] = px[2] = 24;
        }
        fwrite(px, 1, 3, fp);
    }
    fclose(fp);
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s OUTDIR t1 [t2 ...] [-p page]\n", argv[0]); return 2; }
    const char *out = argv[1];
    int page = 0, first = 2;
    if (argc > 3 && argv[2][0] == '-' && argv[2][1] == 'p') { page = atoi(argv[3]); first = 4; }
    lb_ridesim r; lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    lb_lean l; lb_lean_init(&l);
    lb_brain b; lb_brain_init(&b, 0);
    lb_face f; lb_face_init(&f, 1);
    f.page = (lb_page)page;
    lb_gfx g; lb_gfx_init(&g, fb);
    const float imu_dt = 1.0f / 200.0f;
    int sub = 0;
    for (int k = first; k < argc; k++) {
        float target = (float)atof(argv[k]);
        while (r.t < target) {
            lb_imu_sample s;
            lb_ridesim_step(&r, imu_dt, &s);
            lb_lean_update(&l, &s);
            lb_brain_update(&b, &l, imu_dt);
            if (++sub % 6 == 0) lb_face_update(&f, &b, &l, imu_dt * 6); /* ~33 fps */
        }
        lb_face_draw(&f, &b, &l, &g);
        char path[512];
        snprintf(path, sizeof path, "%s/frame_%05d.ppm", out, (int)(target * 10.0f + 0.5f));
        write_ppm(path, f.brightness);
        printf("%s t=%.1f lean=%.1f ev=%s act=%d\n", path, r.t, l.lean_deg, lb_event_name(b.ev), b.act);
    }
    return 0;
}
