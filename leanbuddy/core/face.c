#include "face.h"

#define CX 120.0f
#define CY 120.0f

/* particles */
enum { P_SPARK = 1, P_SPEED, P_SWEAT, P_Z, P_CONFETTI, P_TWINKLE };
/* spawn accumulators */
enum { S_SPARK = 0, S_SPEED, S_SWEAT, S_Z, S_CONFETTI };

typedef struct { uint8_t r, g, b; } rgb;
static const rgb TIER_RGB[] = {
    {70, 215, 255},  /* upright: cyan */
    {80, 255, 170},  /* lean: mint */
    {255, 215, 60},  /* spirited: yellow */
    {255, 130, 40},  /* knee: orange */
    {255, 60, 140},  /* elbow: hot pink */
};
static const rgb GOLD = {255, 200, 60};
static const rgb SLEEPY = {40, 100, 150};
static const rgb CONFETTI_RGB[] = {{255, 80, 120}, {255, 210, 60}, {80, 255, 170}, {70, 200, 255}, {190, 120, 255}};

static lb_color c565(rgb c) { return lb_rgb(c.r, c.g, c.b); }

/* ------------------------------------------------------------------ springs */

static void spring(lb_spring *s, float target, float k, float zeta, float dt) {
    /* semi-implicit Euler in small steps: stable for any frame rate */
    float c = 2.0f * zeta * lb_sqrtf(k);
    int n = (int)(dt / 0.004f) + 1;
    float h = dt / (float)n;
    for (int i = 0; i < n; i++) {
        s->v += (k * (target - s->x) - c * s->v) * h;
        s->x += s->v * h;
    }
}

static void spring_set(lb_spring *s, float x) { s->x = x; s->v = 0.0f; }

/* --------------------------------------------------------------- expression */

typedef struct {
    float w, h, dx, dy, gap, lid, tilt, cheek;
    float scale, ox, oy;
    float mouth_s;
    lb_eye_kind eyes;
    lb_mouth_kind mouth;
    rgb col;
    float look;   /* how much idle glancing applies */
    float shake;
} expr;

static void neutral(expr *e) {
    e->w = 21.0f; e->h = 29.0f; e->dx = 0.0f; e->dy = 0.0f; e->gap = 42.0f;
    e->lid = 0.0f; e->tilt = 0.0f; e->cheek = 0.0f;
    e->scale = 1.0f; e->ox = 0.0f; e->oy = 0.0f;
    e->mouth_s = 0.8f; e->eyes = LB_EYES_NORMAL; e->mouth = LB_MOUTH_SMILE;
    e->col = TIER_RGB[0]; e->look = 1.0f; e->shake = 0.0f;
}

static void report_eyes(expr *e) {
    /* small happy eyes up top, leaving the middle for the number */
    e->w = 11.0f; e->h = 13.0f; e->gap = 22.0f; e->dy = -60.0f; e->cheek = 0.75f;
    e->mouth = LB_MOUTH_NONE; e->mouth_s = 0.0f; e->look = 0.0f; e->dx = 0.0f;
}

static void choose(const lb_face *f, const lb_brain *b, expr *e) {
    neutral(e);
    float lean = b->lean_deg;
    if (b->act == LB_ACT_ASLEEP) {
        e->eyes = LB_EYES_SLEEP; e->mouth = LB_MOUTH_SMILE; e->mouth_s = 0.45f;
        e->col = SLEEPY; e->look = 0.0f; e->oy = 6.0f;
    } else if (b->act == LB_ACT_RIDING) {
        /* riders look through the corner: so does the buddy */
        e->dx = lb_clampf(lean * 0.45f, -16.0f, 16.0f);
        e->look = b->tier == LB_TIER_UPRIGHT ? 0.6f : 0.0f;
        e->col = TIER_RGB[b->tier];
        switch (b->tier) {
        case LB_TIER_UPRIGHT: e->cheek = 0.15f; break;
        case LB_TIER_LEAN: e->cheek = 0.35f; e->mouth_s = 1.0f; break;
        case LB_TIER_SPIRITED:
            e->lid = 0.25f; e->tilt = 0.30f; e->cheek = 0.1f; e->mouth = LB_MOUTH_FLAT; e->mouth_s = 0.8f;
            break;
        case LB_TIER_KNEE:
            e->w = 24.0f; e->h = 33.0f; e->lid = 0.12f; e->tilt = 0.22f; e->mouth = LB_MOUTH_GRIN;
            e->mouth_s = 0.9f;
            break;
        case LB_TIER_ELBOW:
            e->w = 26.0f; e->h = 36.0f; e->mouth = LB_MOUTH_O; e->mouth_s = 1.0f; e->shake = 1.6f;
            break;
        }
    } else {
        /* stopped: look around; yawn now and then when bored */
        if (b->bored_s > 12.0f) {
            float ph = lb_fractf((b->bored_s - 12.0f) / 9.0f) * 9.0f;
            if (ph < 1.8f) { e->mouth = LB_MOUTH_O; e->mouth_s = 1.25f; e->lid = 0.55f; e->look = 0.0f; e->dy = -3.0f; }
        }
    }

    float t = b->ev_t;
    switch (b->ev) {
    case LB_EV_CALIBRATING:
        e->dx = 14.0f * lb_sinf(f->t * 2.2f); e->lid = 0.2f; e->mouth = LB_MOUTH_FLAT; e->mouth_s = 0.6f;
        e->col = (rgb){200, 220, 255}; e->look = 0.0f; e->dy = -10.0f;
        break;
    case LB_EV_CALIBRATED:
        e->cheek = 0.8f; e->mouth = LB_MOUTH_GRIN; e->mouth_s = 0.9f; e->col = TIER_RGB[1]; e->look = 0.0f;
        e->dy = -8.0f;
        break;
    case LB_EV_WAKE:
        e->w = 23.0f; e->h = 33.0f; e->look = 0.0f;
        if (t < 0.6f) { e->mouth = LB_MOUTH_O; e->mouth_s = 0.7f; }
        break;
    case LB_EV_LAUNCH:
        e->cheek = 0.75f; e->mouth = LB_MOUTH_GRIN; e->mouth_s = 1.0f; e->scale = 0.93f; e->oy = -4.0f;
        e->col = TIER_RGB[1]; e->look = 0.0f;
        break;
    case LB_EV_HARD_BRAKE:
        e->look = 0.0f; e->mouth = LB_MOUTH_WAVY; e->oy = 7.0f; e->col = (rgb){120, 200, 255};
        if (b->ev_level >= 2) { e->eyes = LB_EYES_EEK; e->scale = 1.12f; e->mouth_s = 1.1f; e->shake = 2.2f; }
        else { e->w = 23.0f; e->h = 33.0f; e->tilt = -0.35f; e->lid = 0.05f; e->scale = 1.07f; e->mouth_s = 0.9f; }
        break;
    case LB_EV_STOPPIE:
        e->eyes = LB_EYES_EEK; e->mouth = LB_MOUTH_O; e->mouth_s = 0.9f; e->oy = 16.0f; e->look = 0.0f;
        break;
    case LB_EV_BUMP:
        e->look = 0.0f; e->mouth = LB_MOUTH_O; e->mouth_s = 0.6f;
        if (b->ev_level >= 2 && t < 0.35f) e->eyes = LB_EYES_EEK;
        break;
    case LB_EV_WHEELIE:
        e->oy = -18.0f; e->dy = -5.0f; e->cheek = 0.7f; e->mouth = LB_MOUTH_GRIN; e->mouth_s = 1.1f;
        e->col = TIER_RGB[1]; e->look = 0.0f;
        break;
    case LB_EV_AIRTIME:
        e->eyes = LB_EYES_ROUND; e->w = 24.0f; e->h = 24.0f; e->oy = -12.0f; e->mouth = LB_MOUTH_O; e->cheek = 0.0f;
        e->mouth_s = 1.0f; e->col = (rgb){150, 230, 255}; e->look = 0.0f;
        break;
    case LB_EV_LANDING:
        e->cheek = 0.6f; e->mouth = LB_MOUTH_GRIN; e->mouth_s = 0.9f; e->look = 0.0f; e->dy = -6.0f;
        break;
    case LB_EV_TIPOVER:
        e->eyes = LB_EYES_DIZZY; e->mouth = LB_MOUTH_WAVY; e->mouth_s = 0.8f; e->col = (rgb){190, 120, 255};
        e->look = 0.0f;
        break;
    case LB_EV_CORNER:
        report_eyes(e);
        e->col = TIER_RGB[b->ev_level > 0 && b->ev_level < 5 ? b->ev_level : 1];
        break;
    case LB_EV_RECORD:
        report_eyes(e);
        e->eyes = LB_EYES_HEART; e->col = GOLD;
        break;
    case LB_EV_WHEELIE_DONE:
        report_eyes(e);
        e->col = b->ev_level >= 2 ? GOLD : TIER_RGB[1];
        break;
    default: break;
    }
}

/* ---------------------------------------------------------------- particles */

static lb_particle *spawn(lb_face *f, int kind) {
    for (int i = 0; i < LB_MAX_PARTICLES; i++) {
        if (!f->p[i].alive) {
            lb_particle *p = &f->p[i];
            lb_particle z = {0};
            *p = z;
            p->alive = 1;
            p->kind = (uint8_t)kind;
            return p;
        }
    }
    return 0;
}

/* face space -> screen space with the face's current transform */
static void face_to_screen(const lb_face *f, float x, float y, float *sx, float *sy) {
    float cs = lb_cosf(f->rot.x), sn = lb_sinf(f->rot.x), k = f->scale.x;
    float dx = (x - CX) * k, dy = (y - CY) * k;
    *sx = cs * dx - sn * dy + CX + f->off_x.x;
    *sy = sn * dx + cs * dy + CY + f->off_y.x;
}

static void confetti_burst(lb_face *f, int n) {
    for (int i = 0; i < n; i++) {
        lb_particle *p = spawn(f, P_CONFETTI);
        if (!p) return;
        float a = lb_rng_range(&f->rng, 0.0f, LB_TAU);
        float sp = lb_rng_range(&f->rng, 60.0f, 190.0f);
        p->x = CX; p->y = CY - 10.0f;
        p->vx = sp * lb_sinf(a); p->vy = -sp * lb_absf(lb_cosf(a)) - 40.0f;
        p->max_life = p->life = lb_rng_range(&f->rng, 1.6f, 2.6f);
        p->size = lb_rng_range(&f->rng, 2.5f, 4.5f);
        p->spin = lb_rng_range(&f->rng, -10.0f, 10.0f);
        p->c = c565(CONFETTI_RGB[lb_rng_u32(&f->rng) % 5]);
    }
}

static void twinkles(lb_face *f, int n) {
    for (int i = 0; i < n; i++) {
        lb_particle *p = spawn(f, P_TWINKLE);
        if (!p) return;
        float a = lb_rng_range(&f->rng, 0.0f, LB_TAU), r = lb_rng_range(&f->rng, 55.0f, 80.0f);
        p->x = CX + r * lb_sinf(a); p->y = CY + 8.0f - 0.7f * r * lb_cosf(a);
        p->max_life = p->life = lb_rng_range(&f->rng, 0.5f, 0.9f);
        p->size = lb_rng_range(&f->rng, 4.0f, 8.0f);
        p->spin = lb_rng_range(&f->rng, 0.0f, 0.6f); /* start delay */
        p->c = lb_rgb(255, 250, 220);
    }
}

static void update_particles(lb_face *f, const lb_brain *b, const lb_lean *l, float dt) {
    float lean = b->lean_deg * LB_DEG;
    /* world "down" on the screen (the screen leans with the bike) */
    float gx = lb_sinf(lean), gy = lb_cosf(lean);
    for (int i = 0; i < LB_MAX_PARTICLES; i++) {
        lb_particle *p = &f->p[i];
        if (!p->alive) continue;
        p->life -= dt;
        if (p->life <= 0.0f) { p->alive = 0; continue; }
        float grav = 0.0f, drag = 0.0f;
        switch (p->kind) {
        case P_SPARK: grav = 420.0f; drag = 1.5f; break;
        case P_SWEAT: grav = 240.0f; break;
        case P_CONFETTI: grav = 150.0f; drag = 2.2f; break;
        case P_Z: drag = 0.0f; break;
        case P_TWINKLE: if (p->spin > 0.0f) { p->spin -= dt; p->life += dt; } break;
        default: break;
        }
        p->vx += gx * grav * dt; p->vy += gy * grav * dt;
        p->vx -= p->vx * drag * dt; p->vy -= p->vy * drag * dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
    }

    /* sparks from where the peg / knee slider meets the road */
    float rate = 0.0f;
    if (b->act == LB_ACT_RIDING && b->ev == LB_EV_NONE) {
        if (b->tier == LB_TIER_KNEE) rate = 55.0f;
        else if (b->tier >= LB_TIER_ELBOW) rate = 120.0f;
    }
    f->spawn[S_SPARK] += rate * dt;
    while (f->spawn[S_SPARK] >= 1.0f) {
        f->spawn[S_SPARK] -= 1.0f;
        lb_particle *p = spawn(f, P_SPARK);
        if (!p) break;
        float a = LB_PI - lean + lb_rng_range(&f->rng, -0.12f, 0.12f); /* 0 = up, clockwise */
        p->x = CX + 114.0f * lb_sinf(a); p->y = CY - 114.0f * lb_cosf(a);
        /* thrown inwards and backwards (towards the rider = up the screen) */
        float in = lb_rng_range(&f->rng, 70.0f, 170.0f), side = lb_rng_range(&f->rng, -90.0f, 90.0f);
        p->vx = -in * lb_sinf(a) + side * lb_cosf(a);
        p->vy = in * lb_cosf(a) + side * lb_sinf(a) - lb_rng_range(&f->rng, 60.0f, 160.0f);
        p->max_life = p->life = lb_rng_range(&f->rng, 0.3f, 0.65f);
        p->size = lb_rng_range(&f->rng, 1.3f, 2.4f);
        p->c = lb_rng_f(&f->rng) < 0.5f ? lb_rgb(255, 245, 170) : lb_rgb(255, 160, 50);
    }
    (void)l;

    /* speed lines: launches and fast riding */
    rate = 0.0f;
    if (b->ev == LB_EV_LAUNCH || b->ev == LB_EV_WHEELIE) rate = 34.0f;
    else if (b->act == LB_ACT_RIDING && b->excitement > 0.55f && b->ev == LB_EV_NONE) rate = 14.0f;
    f->spawn[S_SPEED] += rate * dt;
    while (f->spawn[S_SPEED] >= 1.0f) {
        f->spawn[S_SPEED] -= 1.0f;
        lb_particle *p = spawn(f, P_SPEED);
        if (!p) break;
        float a = lb_rng_range(&f->rng, 0.0f, LB_TAU), r0 = lb_rng_range(&f->rng, 60.0f, 90.0f);
        float sp = lb_rng_range(&f->rng, 320.0f, 560.0f);
        p->x = CX + r0 * lb_sinf(a); p->y = CY - r0 * lb_cosf(a);
        p->vx = sp * lb_sinf(a); p->vy = -sp * lb_cosf(a);
        p->max_life = p->life = 0.22f;
        p->size = lb_rng_range(&f->rng, 10.0f, 22.0f);
        p->c = lb_rgb(170, 190, 220);
    }

    /* sweat drops when braking hard */
    f->spawn[S_SWEAT] += (b->ev == LB_EV_HARD_BRAKE || b->ev == LB_EV_STOPPIE ? 2.6f : 0.0f) * dt;
    while (f->spawn[S_SWEAT] >= 1.0f) {
        f->spawn[S_SWEAT] -= 1.0f;
        lb_particle *p = spawn(f, P_SWEAT);
        if (!p) break;
        float side = lb_rng_f(&f->rng) < 0.6f ? 1.0f : -1.0f;
        face_to_screen(f, CX + side * 74.0f, CY - 50.0f, &p->x, &p->y);
        p->vx = side * 20.0f; p->vy = -30.0f;
        p->max_life = p->life = 0.9f;
        p->size = lb_rng_range(&f->rng, 5.0f, 7.0f);
        p->c = lb_rgb(120, 200, 255);
    }

    /* Zzz */
    f->spawn[S_Z] += (b->act == LB_ACT_ASLEEP ? 0.6f : 0.0f) * dt;
    while (f->spawn[S_Z] >= 1.0f) {
        f->spawn[S_Z] -= 1.0f;
        lb_particle *p = spawn(f, P_Z);
        if (!p) break;
        p->x = CX + 50.0f; p->y = CY - 40.0f;
        p->vx = 10.0f; p->vy = -18.0f;
        p->max_life = p->life = 3.0f;
        p->size = 12.0f;
        p->c = c565(SLEEPY);
    }

    /* a record keeps raining confetti for a moment */
    f->spawn[S_CONFETTI] += (b->ev == LB_EV_RECORD && b->ev_t < 1.6f ? 14.0f : 0.0f) * dt;
    if (f->spawn[S_CONFETTI] >= 1.0f) {
        int n = (int)f->spawn[S_CONFETTI];
        f->spawn[S_CONFETTI] -= (float)n;
        for (int i = 0; i < n; i++) {
            lb_particle *p = spawn(f, P_CONFETTI);
            if (!p) break;
            p->x = lb_rng_range(&f->rng, 40.0f, 200.0f); p->y = lb_rng_range(&f->rng, 10.0f, 40.0f);
            p->vx = lb_rng_range(&f->rng, -30.0f, 30.0f); p->vy = lb_rng_range(&f->rng, 10.0f, 50.0f);
            p->max_life = p->life = 2.2f;
            p->size = lb_rng_range(&f->rng, 2.5f, 4.0f);
            p->spin = lb_rng_range(&f->rng, -8.0f, 8.0f);
            p->c = c565(CONFETTI_RGB[lb_rng_u32(&f->rng) % 5]);
        }
    }
}

/* ------------------------------------------------------------------- update */

void lb_face_init(lb_face *f, uint32_t seed) {
    lb_face z = {0};
    *f = z;
    f->rng.s = seed ? seed : 12345u;
    spring_set(&f->scale, 1.0f);
    spring_set(&f->eye_w, 21.0f);
    spring_set(&f->eye_h, 29.0f);
    spring_set(&f->eye_gap, 42.0f);
    spring_set(&f->squash, 1.0f);
    spring_set(&f->mouth_s, 0.8f);
    f->eyes = f->eyes_target = LB_EYES_NORMAL;
    f->mouth = f->mouth_target = LB_MOUTH_SMILE;
    f->blink_t = -1.0f;
    f->next_blink = 1.5f;
    f->next_look = 2.0f;
    f->r = 70; f->g = 215; f->b = 255;
    f->brightness = 1.0f;
    f->last_ev = LB_EV_NONE;
    f->last_act = LB_ACT_STOPPED;
}

void lb_face_next_page(lb_face *f) {
    f->page = (lb_page)((f->page + 1) % LB_PAGE_COUNT);
    f->page_t = 0.0f;
}

void lb_face_update(lb_face *f, const lb_brain *b, const lb_lean *l, float dt) {
    if (dt > 0.05f) dt = 0.05f;
    if (dt <= 0.0f) return;
    f->t += dt;
    f->page_t += dt;

    expr e;
    choose(f, b, &e);

    /* ---- one-off kicks when an event starts ---- */
    if (b->ev != f->last_ev) {
        switch (b->ev) {
        case LB_EV_BUMP: f->squash.v -= b->ev_level >= 2 ? 14.0f : 9.0f; break;
        case LB_EV_LANDING: f->squash.v -= b->ev_level >= 2 ? 16.0f : 11.0f; break;
        case LB_EV_WAKE: f->off_y.v -= 260.0f; f->squash.v += 6.0f; break;
        case LB_EV_CALIBRATED: f->off_y.v -= 220.0f; break;
        case LB_EV_RECORD: confetti_burst(f, 36); break;
        case LB_EV_WHEELIE_DONE: if (b->ev_level >= 2) confetti_burst(f, 24); break;
        case LB_EV_CORNER: if (b->ev_level >= LB_TIER_KNEE) twinkles(f, 6); break;
        default: break;
        }
        f->last_ev = b->ev;
    }
    if (f->last_act == LB_ACT_ASLEEP && b->act != LB_ACT_ASLEEP) f->blink_t = -1.0f;
    f->last_act = b->act;

    /* ---- eye shape changes happen behind a blink ---- */
    if (e.eyes != f->eyes_target) {
        f->eyes_target = e.eyes;
        if (f->eyes == LB_EYES_SLEEP || e.eyes == LB_EYES_SLEEP) { f->eyes = e.eyes; }
        else { f->blink_t = 0.0f; f->swap_pending = 1; }
    }
    if (f->blink_t < 0.0f) {
        f->next_blink -= dt;
        if (f->next_blink <= 0.0f && (f->eyes == LB_EYES_NORMAL || f->eyes == LB_EYES_ROUND)) {
            f->blink_t = 0.0f;
            f->next_blink = lb_rng_range(&f->rng, 2.2f, 5.5f) * (1.0f + b->excitement);
            /* sometimes a double blink */
            if (lb_rng_f(&f->rng) < 0.15f) f->next_blink = 0.25f;
        }
    } else {
        f->blink_t += dt;
        if (f->swap_pending && f->blink_t >= 0.07f) { f->eyes = f->eyes_target; f->swap_pending = 0; }
        if (f->blink_t > 0.15f) f->blink_t = -1.0f;
    }

    /* ---- mouth changes shrink, swap, grow ---- */
    float mouth_target = e.mouth_s;
    if (e.mouth != f->mouth) {
        mouth_target = 0.0f;
        if (f->mouth_s.x < 0.12f || f->mouth == LB_MOUTH_NONE) f->mouth = e.mouth;
    }
    f->mouth_target = e.mouth;

    /* ---- idle glances ---- */
    if (f->look_hold > 0.0f) {
        f->look_hold -= dt;
        if (f->look_hold <= 0.0f) { f->look_x = f->look_y = 0.0f; f->next_look = lb_rng_range(&f->rng, 1.5f, 4.5f); }
    } else {
        f->next_look -= dt;
        if (f->next_look <= 0.0f) {
            f->look_x = lb_rng_range(&f->rng, -13.0f, 13.0f);
            f->look_y = lb_rng_range(&f->rng, -7.0f, 5.0f);
            f->look_hold = lb_rng_range(&f->rng, 0.5f, 1.5f);
        }
    }

    /* ---- springs ---- */
    float lean_rad = b->lean_deg * LB_DEG;
    spring(&f->rot, -lean_rad, 70.0f, 0.75f, dt);          /* horizon lock, with a little wobble */
    spring(&f->scale, e.scale, 220.0f, 0.5f, dt);
    spring(&f->off_x, e.ox, 160.0f, 0.7f, dt);
    spring(&f->off_y, e.oy, 160.0f, 0.55f, dt);
    spring(&f->eye_w, e.w, 260.0f, 0.55f, dt);
    spring(&f->eye_h, e.h, 260.0f, 0.55f, dt);
    spring(&f->eye_gap, e.gap, 220.0f, 0.6f, dt);
    spring(&f->eye_dx, e.dx + e.look * f->look_x, 240.0f, 0.65f, dt);
    spring(&f->eye_dy, e.dy + e.look * f->look_y, 240.0f, 0.65f, dt);
    spring(&f->lid, e.lid, 400.0f, 0.8f, dt);
    spring(&f->tilt, e.tilt, 300.0f, 0.8f, dt);
    spring(&f->cheek, e.cheek, 300.0f, 0.75f, dt);
    spring(&f->squash, 1.0f, 320.0f, 0.3f, dt);
    spring(&f->mouth_s, mouth_target, 300.0f, 0.6f, dt);

    float k = lb_minf(1.0f, dt * 6.0f);
    f->r += (e.col.r - f->r) * k;
    f->g += (e.col.g - f->g) * k;
    f->b += (e.col.b - f->b) * k;
    f->gauge += (b->lean_deg - f->gauge) * lb_minf(1.0f, dt * 14.0f);
    float bt = b->act == LB_ACT_ASLEEP ? 0.15f : 1.0f;
    f->brightness += (bt - f->brightness) * lb_minf(1.0f, dt * (bt > f->brightness ? 6.0f : 0.8f));
    f->shake += (e.shake - f->shake) * lb_minf(1.0f, dt * 8.0f);

    update_particles(f, b, l, dt);
}

/* --------------------------------------------------------------------- draw */

typedef struct {
    float cx, cy, hw, hh, r;
    float lid_y, lid_slope, lid_norm;
    float cheek_cy, cheek_r;
} eye_ctx;

static float sdf_eye(const void *c, float x, float y) {
    const eye_ctx *k = (const eye_ctx *)c;
    float d = lb_sdf_rrect(x - k->cx, y - k->cy, k->hw, k->hh, k->r);
    /* top lid: everything above the line is cut away */
    float line = k->lid_y + k->lid_slope * (x - k->cx);
    d = lb_maxf(d, (line - y) * k->lid_norm);
    /* cheek: a big circle pushing up from below makes a happy squint */
    float dx = x - k->cx, dy = y - k->cheek_cy;
    d = lb_maxf(d, k->cheek_r - lb_sqrtf(dx * dx + dy * dy));
    return d;
}

static float blink_amount(const lb_face *f) {
    if (f->blink_t < 0.0f) return 0.0f;
    return lb_clampf(1.0f - lb_absf(f->blink_t - 0.07f) / 0.07f, 0.0f, 1.0f);
}

static void draw_eye(const lb_face *f, lb_gfx *g, float cx, float cy, float side, lb_color col) {
    float hw = f->eye_w.x * (2.0f - f->squash.x);
    float hh = f->eye_h.x * f->squash.x;
    if (hw < 1.0f) hw = 1.0f;
    if (hh < 1.0f) hh = 1.0f;
    float t = f->t;
    switch (f->eyes) {
    case LB_EYES_EEK: {
        /* ">" on the left, "<" on the right: squeezed shut, pointing inwards */
        float s = hh * 0.55f, w = hw * 0.65f, r = 4.5f;
        float tip = cx - side * w, back = cx + side * w;
        lb_fill_capsule(g, back, cy - s, tip, cy, r, col, 1.0f);
        lb_fill_capsule(g, tip, cy, back, cy + s, r, col, 1.0f);
        return;
    }
    case LB_EYES_DIZZY: {
        for (int i = 0; i < 3; i++) {
            float r0 = 5.0f + 7.0f * i, a = t * (5.0f + i) * side + i * 1.3f;
            lb_fill_arc(g, cx, cy, r0, r0 + 3.5f, a, a + 4.4f, col, 1.0f);
        }
        return;
    }
    case LB_EYES_HEART: {
        float pulse = 1.0f + 0.12f * lb_sinf(t * 9.0f);
        lb_fill_heart(g, cx, cy, hh * 2.4f * pulse, col, 1.0f);
        return;
    }
    case LB_EYES_SLEEP: {
        float r = hw * 0.85f;
        lb_fill_arc(g, cx, cy - r * 0.55f, r - 2.0f, r + 2.0f, 0.62f * LB_PI, 1.38f * LB_PI, col, 1.0f);
        return;
    }
    default: break;
    }

    float lid = lb_maxf(f->lid.x, blink_amount(f));
    if (lid > 0.92f) { /* closed: a soft line */
        lb_fill_capsule(g, cx - hw * 0.85f, cy + hh * 0.35f, cx + hw * 0.85f, cy + hh * 0.35f, 3.0f, col, 1.0f);
        return;
    }
    eye_ctx k;
    k.cx = cx; k.cy = cy; k.hw = hw; k.hh = hh;
    k.r = f->eyes == LB_EYES_ROUND ? lb_minf(hw, hh) : lb_minf(hw, hh) * 0.62f;
    k.lid_slope = f->tilt.x * side * -1.0f; /* + tilt: inner corners lower (focused) */
    k.lid_y = cy - hh + lid * 2.0f * hh;
    if (f->tilt.x > 0.0f) k.lid_y += f->tilt.x * hw * 0.5f; /* a focused lid is always a little down */
    k.lid_norm = 1.0f / lb_sqrtf(1.0f + k.lid_slope * k.lid_slope);
    float ch = lb_clampf(f->cheek.x, 0.0f, 1.0f);
    k.cheek_r = hw * 1.7f;
    float top_edge = cy + hh - ch * 1.25f * hh; /* where the cheek circle reaches up to */
    k.cheek_cy = top_edge + k.cheek_r;
    lb_fill_sdf(g, cx - hw, cy - hh, cx + hw, cy + hh, sdf_eye, &k, col, 1.0f);
    /* a small highlight makes them look wet and alive */
    if (f->eyes == LB_EYES_NORMAL && lid < 0.5f && ch < 0.5f && hh > 14.0f)
        lb_fill_circle(g, cx - hw * 0.38f, cy - hh * 0.45f + lid * hh, hw * 0.16f, lb_rgb(255, 255, 255), 0.55f);
}

static void draw_mouth(const lb_face *f, lb_gfx *g, float y, lb_color col) {
    float s = f->mouth_s.x;
    if (s < 0.05f) return;
    float x = CX + f->eye_dx.x * 0.5f;
    switch (f->mouth) {
    case LB_MOUTH_SMILE: {
        float r = 13.0f * s;
        lb_fill_arc(g, x, y - r * 0.55f, r - 2.0f, r + 2.0f, 0.68f * LB_PI, 1.32f * LB_PI, col, 1.0f);
        break;
    }
    case LB_MOUTH_GRIN:
        lb_fill_arc(g, x, y - 4.0f * s, 0.0f, 15.0f * s, 0.5f * LB_PI, 1.5f * LB_PI, col, 1.0f);
        break;
    case LB_MOUTH_O:
        lb_fill_ellipse(g, x, y, 8.0f * s, 10.0f * s, col, 1.0f);
        lb_fill_ellipse(g, x, y + 1.0f, 4.5f * s, 6.0f * s, lb_rgb(0, 0, 0), 1.0f);
        break;
    case LB_MOUTH_FLAT:
        lb_fill_capsule(g, x - 12.0f * s, y, x + 12.0f * s, y, 2.5f, col, 1.0f);
        break;
    case LB_MOUTH_WAVY: {
        float w = 26.0f * s, px = x - w * 0.5f, py = y;
        for (int i = 1; i <= 6; i++) {
            float nx = x - w * 0.5f + w * i / 6.0f, ny = y + ((i & 1) ? -3.5f : 3.5f) * s;
            lb_fill_capsule(g, px, py, nx, ny, 2.2f, col, 1.0f);
            px = nx; py = ny;
        }
        break;
    }
    default: break;
    }
}

static lb_color tier_color(float lean_abs) { return c565(TIER_RGB[lb_tier_of(lean_abs)]); }

/* The lean gauge around the rim.  Lean right fills clockwise from the top. */
static void draw_ring(const lb_face *f, const lb_brain *b, lb_gfx *g, float r0, float r1, float dim) {
    lb_color track = lb_color_scale(lb_rgb(70, 74, 90), dim);
    float lim = 70.0f * LB_DEG;
    lb_fill_arc(g, CX, CY, r0 + 1.5f, r1 - 1.5f, -lim, lim, track, 0.6f);
    for (int a = 15; a <= 60; a += 15) {
        lb_color c = lb_color_scale(a >= 45 ? lb_rgb(200, 200, 210) : lb_rgb(120, 124, 140), dim);
        for (int s = -1; s <= 1; s += 2) {
            float th = s * a * LB_DEG;
            lb_fill_capsule(g, CX + (r0 - 5.0f) * lb_sinf(th), CY - (r0 - 5.0f) * lb_cosf(th),
                            CX + (r0 - 2.0f) * lb_sinf(th), CY - (r0 - 2.0f) * lb_cosf(th), 1.1f, c, 1.0f);
        }
    }
    float v = lb_clampf(f->gauge, -70.0f, 70.0f) * LB_DEG;
    lb_color c = lb_color_scale(tier_color(lb_absf(f->gauge)), dim);
    if (lb_absf(v) > 0.01f) {
        lb_fill_arc(g, CX, CY, r0, r1, 0.0f, v, c, 1.0f);
        float rm = (r0 + r1) * 0.5f;
        lb_fill_circle(g, CX + rm * lb_sinf(v), CY - rm * lb_cosf(v), (r1 - r0) * 0.5f + 1.5f, c, 1.0f);
    }
    /* this ride's deepest lean each way, and the all-time best in gold */
    float pl = -b->ride.lean_left_deg * LB_DEG, pr = b->ride.lean_right_deg * LB_DEG;
    lb_color pk = lb_color_scale(lb_rgb(255, 255, 255), dim);
    if (b->ride.lean_left_deg > 3.0f)
        lb_fill_capsule(g, CX + (r0 - 2) * lb_sinf(pl), CY - (r0 - 2) * lb_cosf(pl), CX + (r1 + 2) * lb_sinf(pl),
                        CY - (r1 + 2) * lb_cosf(pl), 1.3f, pk, 0.9f);
    if (b->ride.lean_right_deg > 3.0f)
        lb_fill_capsule(g, CX + (r0 - 2) * lb_sinf(pr), CY - (r0 - 2) * lb_cosf(pr), CX + (r1 + 2) * lb_sinf(pr),
                        CY - (r1 + 2) * lb_cosf(pr), 1.3f, pk, 0.9f);
    lb_color gold = lb_color_scale(c565(GOLD), dim);
    float bl = -b->best.lean_left_deg * LB_DEG, br = b->best.lean_right_deg * LB_DEG;
    if (b->best.lean_left_deg > 3.0f) lb_fill_circle(g, CX + (r1 + 3.5f) * lb_sinf(bl), CY - (r1 + 3.5f) * lb_cosf(bl), 2.0f, gold, 1.0f);
    if (b->best.lean_right_deg > 3.0f) lb_fill_circle(g, CX + (r1 + 3.5f) * lb_sinf(br), CY - (r1 + 3.5f) * lb_cosf(br), 2.0f, gold, 1.0f);
}

static int fmt_deg(char *s, float v) {
    int n = lb_fmt_int(s, (int)(v + 0.5f));
    s[n++] = '*';
    s[n] = 0;
    return n;
}

static int fmt_1dp(char *s, float v) {
    int t = (int)(v * 10.0f + 0.5f);
    int n = lb_fmt_int(s, t / 10);
    s[n++] = '.';
    s[n++] = (char)('0' + t % 10);
    s[n] = 0;
    return n;
}

static float pop(float t) {
    /* 0 -> 1 with a springy overshoot */
    if (t <= 0.0f) return 0.0f;
    if (t > 0.6f) return 1.0f;
    return 1.0f - lb_expf(-t * 11.0f) * lb_cosf(t * 18.0f);
}

static void draw_report(const lb_face *f, const lb_brain *b, lb_gfx *g, lb_color col) {
    char s[24];
    float k = pop(b->ev_t);
    lb_color white = lb_rgb(235, 238, 245);
    if (b->ev == LB_EV_CORNER || b->ev == LB_EV_RECORD) {
        fmt_deg(s, b->ev_value);
        float size = 58.0f * k;
        lb_draw_text_c(g, s, CX + 6.0f, CY + 6.0f, size, 8.5f * k, col, 1.0f);
        /* which way */
        float w = lb_text_width(s, size);
        float ax = CX + 6.0f + b->ev_side * (w * 0.5f + 26.0f);
        lb_draw_text_c(g, b->ev_side < 0 ? "<" : ">", ax, CY + 6.0f, 20.0f * k, 4.5f, col, 0.9f);
        if (b->ev == LB_EV_RECORD) {
            lb_draw_text_c(g, "NEW BEST!", CX, CY + 58.0f, 17.0f * k, 3.4f, white, 1.0f);
        } else {
            lb_draw_text_c(g, lb_tier_word(lb_tier_of(b->ev_value)), CX, CY + 58.0f, 15.0f * k, 3.0f, white, 1.0f);
        }
    } else if (b->ev == LB_EV_WHEELIE_DONE) {
        int n = fmt_1dp(s, b->ev_value);
        s[n++] = 'S'; s[n] = 0;
        lb_draw_text_c(g, s, CX, CY + 6.0f, 50.0f * k, 7.5f * k, col, 1.0f);
        lb_draw_text_c(g, b->ev_level >= 2 ? "BEST WHEELIE!" : "WHEELIE", CX, CY + 58.0f, 15.0f * k, 3.0f, white, 1.0f);
    }
    (void)f;
}

static void draw_stars(const lb_face *f, lb_gfx *g, int n, float rx, float ry, float y) {
    for (int i = 0; i < n; i++) {
        float a = f->t * 3.0f + i * LB_TAU / n;
        float x = CX + rx * lb_sinf(a), yy = y + ry * lb_cosf(a);
        float depth = 0.6f + 0.4f * lb_cosf(a);
        lb_fill_star(g, x, yy, 7.0f * depth + 3.0f, f->t * 4.0f + i, lb_rgb(255, 225, 90), 0.5f + 0.5f * depth);
    }
}

static void draw_particles(const lb_face *f, lb_gfx *g, int front) {
    for (int i = 0; i < LB_MAX_PARTICLES; i++) {
        const lb_particle *p = &f->p[i];
        if (!p->alive) continue;
        float a = p->max_life > 0.0f ? p->life / p->max_life : 1.0f;
        if (front != (p->kind != P_SPEED)) continue;
        switch (p->kind) {
        case P_SPARK: {
            const float tail = 0.025f; /* seconds of motion blur */
            lb_fill_capsule(g, p->x, p->y, p->x - p->vx * tail, p->y - p->vy * tail, p->size * a + 0.4f, p->c, 0.4f + 0.6f * a);
            break;
        }
        case P_SPEED: {
            float vl = lb_sqrtf(p->vx * p->vx + p->vy * p->vy) + 1e-3f;
            float ux = p->vx / vl, uy = p->vy / vl;
            lb_fill_capsule(g, p->x, p->y, p->x - ux * p->size, p->y - uy * p->size, 1.2f, p->c, 0.55f * a);
            break;
        }
        case P_SWEAT:
            lb_fill_circle(g, p->x, p->y, p->size * 0.62f, p->c, a);
            lb_fill_capsule(g, p->x, p->y - p->size * 1.1f, p->x, p->y - p->size * 0.2f, p->size * 0.22f, p->c, a);
            break;
        case P_Z: {
            float age = 1.0f - a;
            lb_draw_text_c(g, "Z", p->x, p->y, p->size * (0.6f + age), 2.2f, p->c, lb_minf(1.0f, a * 2.0f));
            break;
        }
        case P_CONFETTI: {
            float w = p->size * (0.4f + 0.6f * lb_absf(lb_sinf(p->spin * (p->max_life - p->life))));
            lb_fill_rrect(g, p->x, p->y, w, p->size * 0.6f, 0.8f, p->c, lb_minf(1.0f, a * 3.0f));
            break;
        }
        case P_TWINKLE: {
            if (p->spin > 0.0f) break;
            float s = p->size * lb_sinf(LB_PI * a);
            lb_fill_capsule(g, p->x - s, p->y, p->x + s, p->y, 1.2f, p->c, 1.0f);
            lb_fill_capsule(g, p->x, p->y - s, p->x, p->y + s, 1.2f, p->c, 1.0f);
            break;
        }
        default: break;
        }
    }
}

static void draw_face_page(const lb_face *f, const lb_brain *b, lb_gfx *g, lb_color col) {
    float jx = 0.0f, jy = 0.0f;
    if (f->shake > 0.05f) {
        /* deterministic jitter from the clock */
        jx = f->shake * lb_sinf(f->t * 91.0f);
        jy = f->shake * lb_sinf(f->t * 73.0f + 1.0f);
    }
    float bob = b->act == LB_ACT_ASLEEP ? 3.0f * lb_sinf(f->t * 1.6f) : 1.2f * lb_sinf(f->t * 2.1f);
    lb_gfx_transform(g, f->rot.x, f->scale.x, CX, CY, f->off_x.x + jx, f->off_y.x + jy + bob);

    float ey = CY - 12.0f + f->eye_dy.x, ex = f->eye_dx.x, gap = f->eye_gap.x;
    draw_eye(f, g, CX - gap + ex, ey, -1.0f, col);
    draw_eye(f, g, CX + gap + ex, ey, 1.0f, col);
    draw_mouth(f, g, CY + 40.0f + f->eye_dy.x * 0.4f, col);

    if (lb_event_is_report(b->ev)) draw_report(f, b, g, col);
    if (b->ev == LB_EV_TIPOVER) draw_stars(f, g, 5, 70.0f, 18.0f, CY - 62.0f);
    if (b->ev == LB_EV_BUMP && b->ev_level >= 2) draw_stars(f, g, 3, 60.0f, 14.0f, CY - 56.0f);

    lb_color white = lb_rgb(235, 238, 245);
    char s[24];
    switch (b->ev) {
    case LB_EV_CALIBRATING:
        lb_draw_text_c(g, "KEEP BIKE", CX, CY + 62.0f, 13.0f, 2.6f, white, 1.0f);
        lb_draw_text_c(g, "UPRIGHT", CX, CY + 82.0f, 13.0f, 2.6f, white, 1.0f);
        break;
    case LB_EV_CALIBRATED:
        lb_draw_text_c(g, "READY!", CX, CY + 72.0f, 16.0f * pop(b->ev_t), 3.2f, white, 1.0f);
        break;
    case LB_EV_WHEELIE:
        lb_draw_text_c(g, "WHEELIE!", CX, CY + 66.0f, 16.0f * pop(b->ev_t), 3.4f, col, 1.0f);
        break;
    case LB_EV_TIPOVER:
        lb_draw_text_c(g, "UH OH", CX, CY + 72.0f, 15.0f, 3.0f, white, 1.0f);
        break;
    case LB_EV_LANDING:
        if (b->ev_value >= 0.25f) {
            int n = fmt_1dp(s, b->ev_value);
            s[n++] = 'S'; s[n++] = ' '; s[n++] = 'A'; s[n++] = 'I'; s[n++] = 'R'; s[n] = 0;
            lb_draw_text_c(g, s, CX, CY + 74.0f, 13.0f * pop(b->ev_t), 2.8f, white, 1.0f);
        }
        break;
    default: break;
    }
    lb_gfx_identity(g);
}

static void draw_gauge_page(const lb_face *f, const lb_brain *b, const lb_lean *l, lb_gfx *g) {
    char s[24];
    float a = lb_absf(f->gauge);
    lb_color col = tier_color(a);
    lb_color grey = lb_rgb(140, 144, 160);
    /* screen-aligned: a gauge is read at a glance, and the rider leans with the bike */
    lb_draw_text_c(g, "LEAN", CX, CY - 56.0f, 12.0f, 2.4f, grey, 1.0f);
    fmt_deg(s, a);
    lb_draw_text_c(g, s, CX + 6.0f, CY - 12.0f, 50.0f, 7.5f, col, 1.0f);
    if (a >= 1.0f) {
        float w = lb_text_width(s, 50.0f);
        lb_draw_text_c(g, f->gauge < 0.0f ? "<" : ">", CX + 6.0f + (f->gauge < 0.0f ? -1 : 1) * (w * 0.5f + 22.0f),
                       CY - 12.0f, 18.0f, 4.0f, col, 1.0f);
    }
    /* G-ball: the grip circle, braking up, accelerating down */
    float gr = 22.0f, gy = CY + 52.0f;
    lb_fill_arc(g, CX, gy, gr - 1.0f, gr + 1.0f, 0.0f, LB_TAU, grey, 0.8f);
    lb_fill_capsule(g, CX - gr, gy, CX + gr, gy, 0.6f, grey, 0.5f);
    lb_fill_capsule(g, CX, gy - gr, CX, gy + gr, 0.6f, grey, 0.5f);
    float bx = lb_clampf(l->lat_g / 1.2f, -1.0f, 1.0f) * gr, by = lb_clampf(-l->lon_g / 1.2f, -1.0f, 1.0f) * gr;
    lb_fill_circle(g, CX + bx, gy + by, 5.0f, lb_rgb(255, 255, 255), 1.0f);
    float tg = lb_sqrtf(l->lat_g * l->lat_g + l->lon_g * l->lon_g);
    int n = fmt_1dp(s, tg);
    s[n++] = 'G'; s[n] = 0;
    lb_draw_text_c(g, s, CX, gy + gr + 14.0f, 11.0f, 2.2f, grey, 1.0f);
    /* this ride's maxima */
    fmt_deg(s, b->ride.lean_left_deg);
    lb_draw_text_c(g, s, CX - 62.0f, CY + 52.0f, 13.0f, 2.6f, grey, 1.0f);
    fmt_deg(s, b->ride.lean_right_deg);
    lb_draw_text_c(g, s, CX + 66.0f, CY + 52.0f, 13.0f, 2.6f, grey, 1.0f);
    (void)f;
}

static void draw_stats_page(const lb_face *f, const lb_brain *b, lb_gfx *g) {
    char s[32], v[16];
    lb_color grey = lb_rgb(140, 144, 160), white = lb_rgb(235, 238, 245), gold = c565(GOLD);
    (void)f;
    lb_draw_text_c(g, "THIS RIDE", CX, 40.0f, 12.0f, 2.4f, grey, 1.0f);
    s[0] = 'L'; s[1] = ' '; fmt_deg(s + 2, b->ride.lean_left_deg);
    lb_draw_text_c(g, s, CX - 50.0f, 76.0f, 19.0f, 3.6f, tier_color(b->ride.lean_left_deg), 1.0f);
    s[0] = 'R'; fmt_deg(s + 2, b->ride.lean_right_deg);
    lb_draw_text_c(g, s, CX + 50.0f, 76.0f, 19.0f, 3.6f, tier_color(b->ride.lean_right_deg), 1.0f);

    const char *labels[3] = {"BRAKE ", "WHEELIE ", "CORNERS "};
    for (int i = 0; i < 3; i++) {
        int n = 0;
        for (const char *p = labels[i]; *p; p++) s[n++] = *p;
        if (i == 0) { int m = fmt_1dp(v, b->ride.brake_g); v[m++] = 'G'; v[m] = 0; }
        else if (i == 1) { int m = fmt_1dp(v, b->ride.wheelie_s); v[m++] = 'S'; v[m] = 0; }
        else lb_fmt_int(v, b->ride.corners);
        for (const char *p = v; *p; p++) s[n++] = *p;
        s[n] = 0;
        lb_draw_text_c(g, s, CX, 116.0f + i * 26.0f, 14.0f, 2.8f, white, 1.0f);
    }
    int n = 0;
    const char *best = "BEST L";
    for (const char *p = best; *p; p++) s[n++] = *p;
    n += lb_fmt_int(s + n, (int)(b->best.lean_left_deg + 0.5f));
    s[n++] = ' '; s[n++] = 'R';
    n += lb_fmt_int(s + n, (int)(b->best.lean_right_deg + 0.5f));
    s[n] = 0;
    lb_draw_text_c(g, s, CX, 200.0f, 12.0f, 2.4f, gold, 1.0f);
}

void lb_face_draw(const lb_face *f, const lb_brain *b, const lb_lean *l, lb_gfx *g) {
    lb_gfx_identity(g);
    lb_gfx_clear(g, 0);
    lb_color col = lb_rgb((int)f->r, (int)f->g, (int)f->b);
    int asleep = b->act == LB_ACT_ASLEEP;
    switch (f->page) {
    case LB_PAGE_FACE:
        draw_ring(f, b, g, 110.0f, 116.0f, asleep ? 0.3f : 1.0f);
        draw_particles(f, g, 0);
        draw_face_page(f, b, g, col);
        draw_particles(f, g, 1);
        break;
    case LB_PAGE_GAUGE:
        draw_ring(f, b, g, 100.0f, 116.0f, 1.0f);
        draw_gauge_page(f, b, l, g);
        break;
    default:
        draw_stats_page(f, b, g);
        break;
    }
}
