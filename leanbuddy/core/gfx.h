/* A tiny anti-aliased software renderer for a 240x240 RGB565 frame buffer.
 *
 * Every shape is a signed distance function evaluated per pixel inside its
 * bounding box: edges come out anti-aliased, shapes compose (an eye is a rounded
 * rectangle minus an eyelid minus a cheek), and the whole face can be drawn
 * rotated, which is how it stays level with the horizon while the bike leans.
 * The same code renders on the ESP32 and in the browser simulator.
 */
#ifndef LB_GFX_H
#define LB_GFX_H

#include "lb_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LB_W 240
#define LB_H 240

typedef uint16_t lb_color; /* RGB565 */

static inline lb_color lb_rgb(int r, int g, int b) {
    return (lb_color)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3));
}
lb_color lb_color_mix(lb_color a, lb_color b, float t); /* t=0 -> a, 1 -> b */
lb_color lb_color_scale(lb_color c, float k);         /* dim/brighten */

typedef struct {
    lb_color *px; /* LB_W * LB_H, row-major */
    /* current transform: shape coordinates are rotated by `angle` about (cx, cy),
     * scaled by `scale` about the same point, then offset by (ox, oy) */
    float cs, sn, scale, cx, cy, ox, oy;
} lb_gfx;

void lb_gfx_init(lb_gfx *g, lb_color *buffer);
void lb_gfx_clear(lb_gfx *g, lb_color c);
/* angle in radians, clockwise on screen */
void lb_gfx_transform(lb_gfx *g, float angle, float scale, float cx, float cy, float ox, float oy);
void lb_gfx_identity(lb_gfx *g);

/* Generic SDF fill.  `sdf` returns the signed distance (in shape units) at a
 * point in shape coordinates; negative inside.  (x0,y0)-(x1,y1) bounds it. */
typedef float (*lb_sdf_fn)(const void *ctx, float x, float y);
void lb_fill_sdf(lb_gfx *g, float x0, float y0, float x1, float y1, lb_sdf_fn sdf, const void *ctx,
                 lb_color c, float alpha);

/* primitives, in shape coordinates */
void lb_fill_circle(lb_gfx *g, float x, float y, float r, lb_color c, float alpha);
void lb_fill_ellipse(lb_gfx *g, float x, float y, float rx, float ry, lb_color c, float alpha);
void lb_fill_capsule(lb_gfx *g, float x0, float y0, float x1, float y1, float r, lb_color c, float alpha);
void lb_fill_rrect(lb_gfx *g, float x, float y, float hw, float hh, float radius, lb_color c, float alpha);
/* ring segment: radii r0..r1, angles a0..a1 in radians, 0 = up, clockwise */
void lb_fill_arc(lb_gfx *g, float x, float y, float r0, float r1, float a0, float a1, lb_color c, float alpha);
void lb_fill_heart(lb_gfx *g, float x, float y, float size, lb_color c, float alpha);
void lb_fill_star(lb_gfx *g, float x, float y, float r, float spin, lb_color c, float alpha);

/* SDF building blocks for composite shapes */
float lb_sdf_rrect(float px, float py, float hw, float hh, float r);
float lb_sdf_capsule(float px, float py, float ax, float ay, float bx, float by);

/* Stroke font: digits, A-Z, and . ! - : / < > ? % ' (degree sign: use '*').
 * `size` is the cap height in pixels.  Returns the width drawn. */
float lb_text_width(const char *s, float size);
float lb_draw_text(lb_gfx *g, const char *s, float x, float y, float size, float weight, lb_color c,
                   float alpha);
/* centred on (x, y) */
void lb_draw_text_c(lb_gfx *g, const char *s, float x, float y, float size, float weight, lb_color c,
                    float alpha);

/* small helpers for text */
int lb_fmt_int(char *out, int v);

#ifdef __cplusplus
}
#endif

#endif
