#include "gfx.h"

lb_color lb_color_mix(lb_color a, lb_color b, float t) {
    if (t <= 0.0f) return a;
    if (t >= 1.0f) return b;
    int w = (int)(t * 32.0f + 0.5f);
    int ar = a >> 11, ag = (a >> 5) & 63, ab = a & 31;
    int br = b >> 11, bg = (b >> 5) & 63, bb = b & 31;
    int r = ar + (((br - ar) * w) >> 5);
    int g = ag + (((bg - ag) * w) >> 5);
    int bl = ab + (((bb - ab) * w) >> 5);
    return (lb_color)((r << 11) | (g << 5) | bl);
}

lb_color lb_color_scale(lb_color c, float k) {
    int r = (int)((c >> 11) * k), g = (int)(((c >> 5) & 63) * k), b = (int)((c & 31) * k);
    r = r > 31 ? 31 : (r < 0 ? 0 : r);
    g = g > 63 ? 63 : (g < 0 ? 0 : g);
    b = b > 31 ? 31 : (b < 0 ? 0 : b);
    return (lb_color)((r << 11) | (g << 5) | b);
}

void lb_gfx_identity(lb_gfx *g) {
    g->cs = 1.0f; g->sn = 0.0f; g->scale = 1.0f;
    g->cx = LB_W * 0.5f; g->cy = LB_H * 0.5f; g->ox = g->oy = 0.0f;
}

void lb_gfx_init(lb_gfx *g, lb_color *buffer) {
    g->px = buffer;
    lb_gfx_identity(g);
}

void lb_gfx_clear(lb_gfx *g, lb_color c) {
    uint32_t two = ((uint32_t)c << 16) | c;
    uint32_t *p = (uint32_t *)g->px;
    for (int i = 0; i < LB_W * LB_H / 2; i++) p[i] = two;
}

void lb_gfx_transform(lb_gfx *g, float angle, float scale, float cx, float cy, float ox, float oy) {
    g->cs = lb_cosf(angle); g->sn = lb_sinf(angle);
    g->scale = scale > 0.01f ? scale : 0.01f;
    g->cx = cx; g->cy = cy; g->ox = ox; g->oy = oy;
}

static inline void to_screen(const lb_gfx *g, float x, float y, float *sx, float *sy) {
    float dx = (x - g->cx) * g->scale, dy = (y - g->cy) * g->scale;
    *sx = g->cs * dx - g->sn * dy + g->cx + g->ox;
    *sy = g->sn * dx + g->cs * dy + g->cy + g->oy;
}

static inline void blend(lb_color *p, lb_color c, float a) {
    if (a >= 0.99f) *p = c;
    else if (a > 0.01f) *p = lb_color_mix(*p, c, a);
}

void lb_fill_sdf(lb_gfx *g, float x0, float y0, float x1, float y1, lb_sdf_fn sdf, const void *ctx,
                 lb_color c, float alpha) {
    if (alpha <= 0.01f) return;
    /* screen bounding box of the transformed shape box */
    float xs[4], ys[4];
    to_screen(g, x0, y0, &xs[0], &ys[0]);
    to_screen(g, x1, y0, &xs[1], &ys[1]);
    to_screen(g, x0, y1, &xs[2], &ys[2]);
    to_screen(g, x1, y1, &xs[3], &ys[3]);
    float mnx = xs[0], mxx = xs[0], mny = ys[0], mxy = ys[0];
    for (int i = 1; i < 4; i++) {
        mnx = lb_minf(mnx, xs[i]); mxx = lb_maxf(mxx, xs[i]);
        mny = lb_minf(mny, ys[i]); mxy = lb_maxf(mxy, ys[i]);
    }
    int ix0 = (int)lb_floorf(mnx) - 1, ix1 = (int)mxx + 2;
    int iy0 = (int)lb_floorf(mny) - 1, iy1 = (int)mxy + 2;
    if (ix0 < 0) ix0 = 0;
    if (iy0 < 0) iy0 = 0;
    if (ix1 > LB_W) ix1 = LB_W;
    if (iy1 > LB_H) iy1 = LB_H;
    if (ix0 >= ix1 || iy0 >= iy1) return;

    /* inverse transform, stepped incrementally along each row */
    float inv = 1.0f / g->scale;
    float ux = g->cs * inv, uy = -g->sn * inv; /* shape-space step per +1 screen x */
    for (int y = iy0; y < iy1; y++) {
        float dx = ix0 + 0.5f - g->cx - g->ox, dy = y + 0.5f - g->cy - g->oy;
        float px = (g->cs * dx + g->sn * dy) * inv + g->cx;
        float py = (-g->sn * dx + g->cs * dy) * inv + g->cy;
        lb_color *row = g->px + y * LB_W;
        for (int x = ix0; x < ix1; x++, px += ux, py += uy) {
            float d = sdf(ctx, px, py) * g->scale;
            if (d >= 0.5f) continue;
            float cov = d <= -0.5f ? 1.0f : 0.5f - d;
            blend(&row[x], c, cov * alpha);
        }
    }
}

/* ---------------- SDF building blocks ---------------- */

float lb_sdf_rrect(float px, float py, float hw, float hh, float r) {
    r = lb_minf(r, lb_minf(hw, hh));
    float qx = lb_absf(px) - (hw - r), qy = lb_absf(py) - (hh - r);
    float ox = lb_maxf(qx, 0.0f), oy = lb_maxf(qy, 0.0f);
    return lb_sqrtf(ox * ox + oy * oy) + lb_minf(lb_maxf(qx, qy), 0.0f) - r;
}

float lb_sdf_capsule(float px, float py, float ax, float ay, float bx, float by) {
    float pax = px - ax, pay = py - ay, bax = bx - ax, bay = by - ay;
    float bb = bax * bax + bay * bay;
    float h = bb > 1e-9f ? lb_clampf((pax * bax + pay * bay) / bb, 0.0f, 1.0f) : 0.0f;
    float dx = pax - bax * h, dy = pay - bay * h;
    return lb_sqrtf(dx * dx + dy * dy);
}

/* ---------------- primitives ---------------- */

typedef struct { float x, y, r; } circle_ctx;
static float sdf_circle(const void *c, float x, float y) {
    const circle_ctx *k = (const circle_ctx *)c;
    float dx = x - k->x, dy = y - k->y;
    return lb_sqrtf(dx * dx + dy * dy) - k->r;
}
void lb_fill_circle(lb_gfx *g, float x, float y, float r, lb_color c, float alpha) {
    circle_ctx k = {x, y, r};
    lb_fill_sdf(g, x - r, y - r, x + r, y + r, sdf_circle, &k, c, alpha);
}

typedef struct { float x, y, rx, ry; } ellipse_ctx;
static float sdf_ellipse(const void *c, float x, float y) {
    const ellipse_ctx *k = (const ellipse_ctx *)c;
    float dx = (x - k->x) / k->rx, dy = (y - k->y) / k->ry;
    return (lb_sqrtf(dx * dx + dy * dy) - 1.0f) * lb_minf(k->rx, k->ry);
}
void lb_fill_ellipse(lb_gfx *g, float x, float y, float rx, float ry, lb_color c, float alpha) {
    if (rx < 0.3f || ry < 0.3f) return;
    ellipse_ctx k = {x, y, rx, ry};
    lb_fill_sdf(g, x - rx, y - ry, x + rx, y + ry, sdf_ellipse, &k, c, alpha);
}

typedef struct { float ax, ay, bx, by, r; } capsule_ctx;
static float sdf_capsule(const void *c, float x, float y) {
    const capsule_ctx *k = (const capsule_ctx *)c;
    return lb_sdf_capsule(x, y, k->ax, k->ay, k->bx, k->by) - k->r;
}
void lb_fill_capsule(lb_gfx *g, float x0, float y0, float x1, float y1, float r, lb_color c, float alpha) {
    capsule_ctx k = {x0, y0, x1, y1, r};
    lb_fill_sdf(g, lb_minf(x0, x1) - r, lb_minf(y0, y1) - r, lb_maxf(x0, x1) + r, lb_maxf(y0, y1) + r,
                sdf_capsule, &k, c, alpha);
}

typedef struct { float x, y, hw, hh, r; } rrect_ctx;
static float sdf_rrect(const void *c, float x, float y) {
    const rrect_ctx *k = (const rrect_ctx *)c;
    return lb_sdf_rrect(x - k->x, y - k->y, k->hw, k->hh, k->r);
}
void lb_fill_rrect(lb_gfx *g, float x, float y, float hw, float hh, float radius, lb_color c, float alpha) {
    if (hw < 0.3f || hh < 0.3f) return;
    rrect_ctx k = {x, y, hw, hh, radius};
    lb_fill_sdf(g, x - hw, y - hh, x + hw, y + hh, sdf_rrect, &k, c, alpha);
}

typedef struct { float x, y, r0, r1, mid, half; } arc_ctx;
static float sdf_arc(const void *c, float x, float y) {
    const arc_ctx *k = (const arc_ctx *)c;
    float dx = x - k->x, dy = y - k->y;
    float r = lb_sqrtf(dx * dx + dy * dy);
    float dr = lb_maxf(k->r0 - r, r - k->r1);
    if (dr > 1.0f) return dr; /* cheap reject: no atan off the ring */
    if (k->half >= LB_PI) return dr;
    float th = lb_atan2f(dx, -dy); /* 0 = up, clockwise */
    float da = (lb_absf(lb_wrap_pi(th - k->mid)) - k->half) * lb_maxf(r, 1.0f);
    return lb_maxf(dr, da);
}
void lb_fill_arc(lb_gfx *g, float x, float y, float r0, float r1, float a0, float a1, lb_color c, float alpha) {
    if (a1 < a0) { float t = a0; a0 = a1; a1 = t; }
    if (a1 - a0 < 1e-4f) return;
    arc_ctx k = {x, y, r0, r1, 0.5f * (a0 + a1), 0.5f * (a1 - a0)};
    lb_fill_sdf(g, x - r1, y - r1, x + r1, y + r1, sdf_arc, &k, c, alpha);
}

/* heart: Inigo Quilez's exact heart SDF, y up, tip at the origin, ~1 tall */
typedef struct { float x, y, s; } heart_ctx;
static float sdf_heart(const void *c, float x, float y) {
    const heart_ctx *k = (const heart_ctx *)c;
    float px = lb_absf((x - k->x) / k->s), py = (k->y - y) / k->s + 0.5f;
    float d;
    if (py + px > 1.0f) {
        float ax = px - 0.25f, ay = py - 0.75f;
        d = lb_sqrtf(ax * ax + ay * ay) - 0.35355339f;
    } else {
        float ax = px, ay = py - 1.0f;
        float m = 0.5f * lb_maxf(px + py, 0.0f);
        float bx = px - m, by = py - m;
        float d2 = lb_minf(ax * ax + ay * ay, bx * bx + by * by);
        d = lb_sqrtf(d2) * lb_signf(px - py);
    }
    return d * k->s;
}
void lb_fill_heart(lb_gfx *g, float x, float y, float size, lb_color c, float alpha) {
    heart_ctx k = {x, y, size};
    lb_fill_sdf(g, x - 0.65f * size, y - 0.6f * size, x + 0.65f * size, y + 0.55f * size, sdf_heart, &k, c, alpha);
}

/* five-point star, Inigo Quilez's sdStar5 */
typedef struct { float x, y, r, cs, sn; } star_ctx;
static float sdf_star(const void *c, float x, float y) {
    const star_ctx *k = (const star_ctx *)c;
    float qx = (x - k->x), qy = (k->y - y);
    float px = k->cs * qx - k->sn * qy, py = k->sn * qx + k->cs * qy;
    const float k1x = 0.809016994f, k1y = -0.587785252f;
    const float k2x = -k1x, k2y = k1y;
    px = lb_absf(px);
    float d1 = 2.0f * lb_maxf(k1x * px + k1y * py, 0.0f);
    px -= d1 * k1x; py -= d1 * k1y;
    float d2 = 2.0f * lb_maxf(k2x * px + k2y * py, 0.0f);
    px -= d2 * k2x; py -= d2 * k2y;
    px = lb_absf(px);
    py -= k->r;
    const float rf = 0.45f;
    float bax = rf * -k1y, bay = rf * k1x - 1.0f;
    float h = lb_clampf((px * bax + py * bay) / (bax * bax + bay * bay), 0.0f, k->r);
    float dx = px - bax * h, dy = py - bay * h;
    return lb_sqrtf(dx * dx + dy * dy) * lb_signf(py * bax - px * bay);
}
void lb_fill_star(lb_gfx *g, float x, float y, float r, float spin, lb_color c, float alpha) {
    star_ctx k = {x, y, r, lb_cosf(spin), lb_sinf(spin)};
    lb_fill_sdf(g, x - r, y - r, x + r, y + r, sdf_star, &k, c, alpha);
}

/* ---------------- stroke font ----------------
 * Glyphs on a 4 x 6 grid (y down).  Each glyph is polylines of "xy" digit pairs,
 * separated by spaces; '|' lifts the pen. */
static const char *glyph(char ch) {
    switch (ch) {
    case '0': return "10 30 41 45 36 16 05 01 10";
    case '1': return "11 20 26|16 36";
    case '2': return "01 10 30 41 42 06 46";
    case '3': return "01 10 30 41 42 33 13|33 44 45 36 16 05";
    case '4': return "36 30 03 43";
    case '5': return "40 00 03 33 44 45 36 06";
    case '6': return "41 30 10 01 05 16 36 45 44 33 13 04";
    case '7': return "00 40 42 16";
    case '8': return "13 02 01 10 30 41 42 33 13 04 05 16 36 45 44 33";
    case '9': return "43 13 02 01 10 30 41 45 36 06";
    case 'A': return "06 01 10 30 41 46|03 43";
    case 'B': return "00 06 36 45 44 33 03|33 42 41 30 00";
    case 'C': return "41 30 10 01 05 16 36 45";
    case 'D': return "00 20 42 44 26 06 00";
    case 'E': return "40 00 06 46|03 33";
    case 'F': return "40 00 06|03 33";
    case 'G': return "41 30 10 01 05 16 36 45 43 23";
    case 'H': return "00 06|40 46|03 43";
    case 'I': return "10 30|20 26|16 36";
    case 'J': return "40 45 36 16 05";
    case 'K': return "00 06|40 03|22 46";
    case 'L': return "00 06 46";
    case 'M': return "06 00 23 40 46";
    case 'N': return "06 00 46 40";
    case 'O': return "10 30 41 45 36 16 05 01 10";
    case 'P': return "06 00 30 41 42 33 03";
    case 'Q': return "10 30 41 45 36 16 05 01 10|34 46";
    case 'R': return "06 00 30 41 42 33 03|23 46";
    case 'S': return "41 30 10 01 02 13 33 44 45 36 16 05";
    case 'T': return "00 40|20 26";
    case 'U': return "00 05 16 36 45 40";
    case 'V': return "00 26 40";
    case 'W': return "00 16 23 36 40";
    case 'X': return "00 46|40 06";
    case 'Y': return "00 23 40|23 26";
    case 'Z': return "00 40 06 46";
    case '!': return "20 24|26 26";
    case '.': return "26 26";
    case '-': return "13 33";
    case ':': return "22 22|24 24";
    case '/': return "06 40";
    case '<': return "41 13 45";
    case '>': return "01 33 05";
    case '?': return "01 10 30 41 42 23 24|26 26";
    case '%': return "06 40|01 01|45 45";
    case '\'': return "20 22";
    case '+': return "03 43|21 25";
    default: return 0;
    }
}

static float glyph_advance(char ch) {
    if (ch == ' ') return 4.0f;
    if (ch == '1' || ch == 'I') return 5.0f;
    if (ch == '.' || ch == '!' || ch == ':' || ch == '\'') return 3.0f;
    if (ch == '*') return 4.0f; /* degree sign */
    return 6.0f;
}

static char up(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }

float lb_text_width(const char *s, float size) {
    float u = size / 6.0f, w = 0.0f;
    int n = 0;
    for (; *s; s++, n++) w += glyph_advance(up(*s));
    return n ? (w - 2.0f) * u : 0.0f;
}

float lb_draw_text(lb_gfx *g, const char *s, float x, float y, float size, float weight, lb_color c,
                   float alpha) {
    float u = size / 6.0f, r = weight * 0.5f, x0 = x;
    for (; *s; s++) {
        char ch = up(*s);
        float ox = (ch == '1' || ch == 'I') ? -0.5f * u : ((ch == '.' || ch == '!' || ch == ':' || ch == '\'') ? -1.0f * u : 0.0f);
        if (ch == '*') {
            lb_fill_arc(g, x + 1.2f * u, y + 1.2f * u, 1.2f * u - r, 1.2f * u + r, 0.0f, LB_TAU, c, alpha);
        } else {
            const char *p = glyph(ch);
            float px = 0, py = 0;
            int pen = 0;
            while (p && *p) {
                if (*p == '|') { pen = 0; p++; continue; }
                if (*p == ' ') { p++; continue; }
                float gx = (float)(p[0] - '0'), gy = (float)(p[1] - '0');
                float nx = x + ox + gx * u, ny = y + gy * u;
                /* a repeated point ("26 26") draws a dot */
                if (pen) lb_fill_capsule(g, px, py, nx, ny, r, c, alpha);
                px = nx; py = ny; pen = 1;
                p += 2;
            }
        }
        x += glyph_advance(ch) * u;
    }
    return x - x0;
}

void lb_draw_text_c(lb_gfx *g, const char *s, float x, float y, float size, float weight, lb_color c,
                    float alpha) {
    lb_draw_text(g, s, x - 0.5f * lb_text_width(s, size), y - 0.5f * size, size, weight, c, alpha);
}

int lb_fmt_int(char *out, int v) {
    char tmp[12];
    int n = 0, neg = v < 0;
    unsigned u = neg ? (unsigned)(-v) : (unsigned)v;
    do { tmp[n++] = (char)('0' + u % 10); u /= 10; } while (u && n < 11);
    int k = 0;
    if (neg) out[k++] = '-';
    while (n) out[k++] = tmp[--n];
    out[k] = 0;
    return k;
}
