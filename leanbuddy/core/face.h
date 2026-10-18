/* The face: how the buddy looks.  Expressions, the lean gauge round the edge of
 * the round screen, particles, and the corner/record report screens.
 *
 * Everything animates through springs, so expressions blend and overshoot a
 * little instead of snapping.  The face is drawn "horizon-locked": rotated against
 * the lean so it stays level with the world while the bike (and screen) leans,
 * the way a rider keeps their head level through a corner.
 */
#ifndef LB_FACE_H
#define LB_FACE_H

#include "brain.h"
#include "gfx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { LB_PAGE_FACE = 0, LB_PAGE_GAUGE, LB_PAGE_STATS, LB_PAGE_COUNT } lb_page;

typedef enum {
    LB_EYES_NORMAL = 0,
    LB_EYES_EEK,     /* > < */
    LB_EYES_DIZZY,   /* spirals */
    LB_EYES_HEART,
    LB_EYES_SLEEP,   /* closed, curved */
    LB_EYES_ROUND,   /* wide circles: airtime */
} lb_eye_kind;

typedef enum {
    LB_MOUTH_NONE = 0,
    LB_MOUTH_SMILE,
    LB_MOUTH_GRIN,   /* open half-moon */
    LB_MOUTH_O,
    LB_MOUTH_FLAT,
    LB_MOUTH_WAVY,
} lb_mouth_kind;

typedef struct { float x, v; } lb_spring;

typedef struct {
    uint8_t kind, alive;
    float x, y, vx, vy, life, max_life, size, spin;
    lb_color c;
} lb_particle;

#define LB_MAX_PARTICLES 64

typedef struct {
    float t;
    lb_rng rng;
    lb_page page;
    float page_t;            /* time on this page (page-change animation) */

    /* animated state */
    lb_spring rot, scale, off_x, off_y;
    lb_spring eye_w, eye_h, eye_dx, eye_dy, lid, tilt, cheek, squash, mouth_s, eye_gap;
    lb_eye_kind eyes, eyes_target;
    lb_mouth_kind mouth, mouth_target;
    float blink_t, next_blink;   /* blink_t < 0: not blinking */
    int swap_pending;            /* eye kind swaps at the middle of a blink */
    float look_x, look_y, next_look, look_hold;
    float r, g, b;               /* eye colour, 0..255 floats, eased */
    float gauge;                 /* displayed lean for the ring, eased */
    float brightness;            /* 0..1: firmware maps it to the backlight */
    float shake;
    float spawn[8];              /* per-effect spawn accumulators */
    lb_event last_ev;
    lb_activity last_act;
    float hop;                   /* little jump impulses */

    lb_particle p[LB_MAX_PARTICLES];
} lb_face;

void lb_face_init(lb_face *f, uint32_t seed);
void lb_face_update(lb_face *f, const lb_brain *b, const lb_lean *l, float dt);
void lb_face_draw(const lb_face *f, const lb_brain *b, const lb_lean *l, lb_gfx *g);
void lb_face_next_page(lb_face *f);

#ifdef __cplusplus
}
#endif

#endif
