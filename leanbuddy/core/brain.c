#include "brain.h"

#define SLEEP_AFTER_S 20.0f       /* still this long: fall asleep */
#define NEW_RIDE_AFTER_S 300.0f   /* asleep this long: the next wake is a new ride */

#define TIER_HYST_DEG 2.0f
#define CORNER_ENTER_DEG 12.0f
#define CORNER_EXIT_DEG 6.0f
#define CORNER_REPORT_DEG 18.0f   /* corners shallower than this get no report */
#define RECORD_MIN_DEG 20.0f
#define RECORD_MARGIN_DEG 1.0f    /* must beat the best by this much */
#define REPORT_HIDE_DEG LB_REPORT_HIDE_DEG
#define REPORT_UPRIGHT_DEG 8.0f   /* a report starts once the bike is this upright ... */
#define REPORT_UPRIGHT_S 0.4f     /* ... for this long */
#define REPORT_MAX_AGE_S 12.0f    /* not shown by then: stale, dropped */
#define REPORT_SEEN_S 1.0f        /* shown this long counts as seen */

#define BRAKE_G 0.45f
#define BRAKE_BIG_G 0.75f
#define BRAKE_HOLD_S 0.15f
#define LAUNCH_G 0.30f
#define LAUNCH_HOLD_S 0.30f
#define BUMP_G 0.7f
#define BUMP_BIG_G 1.2f
#define AIR_G 0.35f
#define AIR_HOLD_S 0.12f
#define WHEELIE_DEG 8.0f
#define WHEELIE_END_DEG 4.0f
#define WHEELIE_HOLD_S 0.3f
#define WHEELIE_REPORT_S 0.5f
#define STOPPIE_DEG -7.0f
#define TIP_DEG 65.0f
#define TIP_HOLD_S 1.5f

static const float TIER_DEG[] = {10.0f, 25.0f, 40.0f, 50.0f};

lb_tier lb_tier_of(float a) {
    int t = 0;
    while (t < 4 && a >= TIER_DEG[t]) t++;
    return (lb_tier)t;
}

const char *lb_tier_word(lb_tier t) {
    switch (t) {
    case LB_TIER_LEAN: return "NICE";
    case LB_TIER_SPIRITED: return "SPICY";
    case LB_TIER_KNEE: return "KNEE DOWN";
    case LB_TIER_ELBOW: return "ELBOW!";
    default: return "";
    }
}

const char *lb_event_name(lb_event e) {
    static const char *names[LB_EV_COUNT] = {
        "none", "corner", "wheelie_done", "record", "wake", "launch", "bump", "hard_brake",
        "stoppie", "wheelie", "landing", "airtime", "tipover", "calibrated", "calibrating"};
    return (e >= 0 && e < LB_EV_COUNT) ? names[e] : "?";
}

void lb_brain_init(lb_brain *b, const lb_records *best) {
    lb_brain z = {0};
    *b = z;
    if (best) b->best = *best;
    b->act = LB_ACT_STOPPED;
    b->prev_cal = LB_CAL_NONE;
}

static void set_event(lb_event_state *s, lb_event e, float dur, float value, int side, int level) {
    s->ev = e;
    s->t = 0.0f;
    s->dur = dur;
    s->value = value;
    s->side = side;
    s->level = level;
    s->age = 0.0f;
}

/* a reaction never interrupts a higher-priority one */
static void react(lb_brain *b, lb_event e, float dur, float value, int side, int level) {
    if (b->react.ev != LB_EV_NONE && e < b->react.ev) return;
    set_event(&b->react, e, dur, value, side, level);
    b->bored_s = 0.0f;
}

static void drop_report(lb_brain *b, int i) {
    for (int k = i; k < b->nreports - 1; k++) b->reports[k] = b->reports[k + 1];
    b->nreports--;
}

/* plain-corner reports go stale as soon as anything newer happens; `all` also
 * drops one already on screen (a new corner has started) */
static void drop_corner_reports(lb_brain *b, int all) {
    for (int i = b->nreports - 1; i >= 0; i--)
        if (b->reports[i].ev == LB_EV_CORNER && (all || b->reports[i].t <= 0.0f)) drop_report(b, i);
}

static void report(lb_brain *b, lb_event e, float dur, float value, int side, int level) {
    drop_corner_reports(b, 0);
    /* one already seen for a while makes way */
    for (int i = b->nreports - 1; i >= 0; i--)
        if (b->reports[i].t >= REPORT_SEEN_S) drop_report(b, i);
    /* a newer record on the same side supersedes the older one */
    for (int i = b->nreports - 1; i >= 0; i--)
        if (e == LB_EV_RECORD && b->reports[i].ev == LB_EV_RECORD && b->reports[i].side == side) drop_report(b, i);
    if (b->nreports == LB_REPORT_QUEUE) drop_report(b, 0);
    set_event(&b->reports[b->nreports++], e, dur, value, side, level);
}

/* an open-ended reaction (dur < 0) finishes `s` seconds from now */
static void linger(lb_brain *b, lb_event e, float s) {
    if (b->react.ev == e && b->react.dur < 0.0f) b->react.dur = b->react.t + s;
}

static void end_react(lb_brain *b, lb_event e) {
    if (b->react.ev == e) b->react.ev = LB_EV_NONE;
}

static void reset_ride(lb_brain *b) {
    lb_records z = {0};
    b->ride = z;
}

static void update_tier(lb_brain *b) {
    lb_tier raw = lb_tier_of(b->lean_abs);
    if (raw > b->tier) b->tier = raw;
    else if (raw < b->tier && b->lean_abs < TIER_DEG[b->tier - 1] - TIER_HYST_DEG) b->tier = raw;
}

void lb_brain_update(lb_brain *b, const lb_lean *l, float dt) {
    b->t += dt;
    b->new_ride = 0;
    lb_event_state *r = &b->react;
    r->t += dt;
    if (r->ev != LB_EV_NONE && r->dur >= 0.0f && r->t >= r->dur) r->ev = LB_EV_NONE;

    /* ---- calibration ---- */
    if (l->cal != LB_CAL_DONE) {
        react(b, LB_EV_CALIBRATING, -1.0f, 0, 0, 1);
        b->prev_cal = l->cal;
        goto out;
    }
    if (b->prev_cal != LB_CAL_DONE) {
        end_react(b, LB_EV_CALIBRATING);
        react(b, LB_EV_CALIBRATED, 1.6f, 0, 0, 1);
        b->prev_cal = LB_CAL_DONE;
    }

    b->lean_deg = l->lean_deg;
    b->lean_abs = lb_absf(l->lean_deg);
    int side = l->lean_deg < 0.0f ? -1 : 1;
    update_tier(b);

    /* ---- asleep / stopped / riding ---- */
    lb_activity prev = b->act;
    if (l->still_s > SLEEP_AFTER_S) b->act = LB_ACT_ASLEEP;
    else b->act = l->moving ? LB_ACT_RIDING : LB_ACT_STOPPED;
    if (b->act == LB_ACT_ASLEEP) {
        b->asleep_s += dt;
    } else if (prev == LB_ACT_ASLEEP) {
        if (b->asleep_s > NEW_RIDE_AFTER_S) { reset_ride(b); b->new_ride = 1; }
        b->asleep_s = 0.0f;
        react(b, LB_EV_WAKE, 1.4f, 0, 0, 1);
    }
    if (b->act == LB_ACT_RIDING) b->ride.ride_s += dt;

    /* ---- excitement and boredom ---- */
    float e = lb_smoothstep(5.0f, 50.0f, b->lean_abs);
    e = lb_maxf(e, lb_smoothstep(0.2f, 0.9f, lb_absf(l->lon_g)));
    e = lb_maxf(e, 0.7f * lb_smoothstep(0.3f, 1.2f, lb_absf(l->vert_g_fast - 1.0f)));
    float tau = e > b->excitement ? 0.25f : 4.0f;
    b->excitement += (e - b->excitement) * lb_lp_alpha(dt, tau);
    if (r->ev == LB_EV_NONE && b->lean_abs < 8.0f && lb_absf(l->lon_g) < 0.15f && b->act != LB_ACT_ASLEEP)
        b->bored_s += dt;
    else
        b->bored_s = 0.0f;

    /* ---- running peaks for this ride ---- */
    if (b->act == LB_ACT_RIDING) {
        b->ride.brake_g = lb_maxf(b->ride.brake_g, -l->lon_g);
        b->ride.accel_g = lb_maxf(b->ride.accel_g, l->lon_g);
        b->ride.corner_g = lb_maxf(b->ride.corner_g, lb_absf(l->lat_g));
        if (side < 0) b->ride.lean_left_deg = lb_maxf(b->ride.lean_left_deg, b->lean_abs);
        else b->ride.lean_right_deg = lb_maxf(b->ride.lean_right_deg, b->lean_abs);
        if (b->ride.brake_g > b->best.brake_g) { b->best.brake_g = b->ride.brake_g; b->best_dirty = 1; }
        if (b->ride.accel_g > b->best.accel_g) { b->best.accel_g = b->ride.accel_g; b->best_dirty = 1; }
        if (b->ride.corner_g > b->best.corner_g) { b->best.corner_g = b->ride.corner_g; b->best_dirty = 1; }
    }

    /* ---- corners: report once the bike is back upright ---- */
    if (!b->in_corner && b->lean_abs > CORNER_ENTER_DEG && b->lean_abs < TIP_DEG) {
        b->in_corner = 1;
        b->corner_peak = 0.0f;
        drop_corner_reports(b, 1);
    }
    if (b->in_corner) {
        if (b->lean_abs > lb_absf(b->corner_peak)) b->corner_peak = b->lean_deg;
        if (b->lean_abs < CORNER_EXIT_DEG) {
            b->in_corner = 0;
            b->ride.corners++;
            b->best.corners = b->best.corners > b->ride.corners ? b->best.corners : b->ride.corners;
            float pk = lb_absf(b->corner_peak);
            int ps = b->corner_peak < 0.0f ? -1 : 1;
            float *best_side = ps < 0 ? &b->best.lean_left_deg : &b->best.lean_right_deg;
            if (pk >= RECORD_MIN_DEG && pk > *best_side + RECORD_MARGIN_DEG) {
                *best_side = pk;
                b->best_dirty = 1;
                report(b, LB_EV_RECORD, 3.2f, pk, ps, 1);
            } else if (pk >= CORNER_REPORT_DEG) {
                report(b, LB_EV_CORNER, 2.4f, pk, ps, (int)lb_tier_of(pk));
            }
        }
    }

    /* ---- hard braking ---- */
    b->brake_hold = l->lon_g < -BRAKE_G ? b->brake_hold + dt : 0.0f;
    if (b->brake_hold > BRAKE_HOLD_S && r->ev != LB_EV_HARD_BRAKE)
        react(b, LB_EV_HARD_BRAKE, -1.0f, -l->lon_g, 0, 1);
    if (r->ev == LB_EV_HARD_BRAKE) {
        r->value = lb_maxf(r->value, -l->lon_g);
        if (-l->lon_g > BRAKE_BIG_G) r->level = 2;
        if (l->lon_g > -0.3f) linger(b, LB_EV_HARD_BRAKE, 0.6f);
    }

    /* ---- launch ---- */
    b->launch_hold = l->lon_g > LAUNCH_G ? b->launch_hold + dt : 0.0f;
    if (b->launch_hold > LAUNCH_HOLD_S && r->ev != LB_EV_LAUNCH && b->lean_abs < 15.0f)
        react(b, LB_EV_LAUNCH, -1.0f, l->lon_g, 0, 1);
    if (r->ev == LB_EV_LAUNCH) {
        r->value = lb_maxf(r->value, l->lon_g);
        if (l->lon_g < 0.2f) linger(b, LB_EV_LAUNCH, 0.5f);
    }

    /* ---- airtime and landing ---- */
    b->air_hold = l->vert_g_fast < AIR_G ? b->air_hold + dt : 0.0f;
    if (b->air_hold > AIR_HOLD_S && r->ev < LB_EV_AIRTIME) {
        b->air_start = b->t - b->air_hold;
        react(b, LB_EV_AIRTIME, -1.0f, 0, 0, 1);
    }
    if (r->ev == LB_EV_AIRTIME) {
        r->value = b->t - b->air_start;
        if (l->vert_g_fast > 0.6f) {
            float air = b->t - b->air_start;
            end_react(b, LB_EV_AIRTIME);
            b->ride.air_s = lb_maxf(b->ride.air_s, air);
            if (air > b->best.air_s) { b->best.air_s = air; b->best_dirty = 1; }
            react(b, LB_EV_LANDING, 1.3f, air, 0, air > 0.5f ? 2 : 1);
        }
    }

    /* ---- bumps ---- */
    float jolt = lb_absf(l->vert_g_fast - 1.0f);
    if (jolt > BUMP_G && r->ev != LB_EV_BUMP && r->ev != LB_EV_AIRTIME && r->ev != LB_EV_LANDING &&
        b->air_hold == 0.0f)
        react(b, LB_EV_BUMP, 0.9f, jolt, 0, jolt > BUMP_BIG_G ? 2 : 1);

    /* ---- wheelies and stoppies ---- */
    b->wheelie_hold = l->pitch_deg > WHEELIE_DEG ? b->wheelie_hold + dt : 0.0f;
    if (b->wheelie_hold > WHEELIE_HOLD_S && r->ev != LB_EV_WHEELIE && b->act == LB_ACT_RIDING) {
        b->wheelie_start = b->t - b->wheelie_hold;
        react(b, LB_EV_WHEELIE, -1.0f, 0, 0, 1);
    }
    if (r->ev == LB_EV_WHEELIE) {
        r->value = b->t - b->wheelie_start;
        if (l->pitch_deg < WHEELIE_END_DEG) {
            float w = b->t - b->wheelie_start;
            end_react(b, LB_EV_WHEELIE);
            b->ride.wheelie_s = lb_maxf(b->ride.wheelie_s, w);
            int rec = w > b->best.wheelie_s;
            if (rec) { b->best.wheelie_s = w; b->best_dirty = 1; }
            if (w > WHEELIE_REPORT_S) report(b, LB_EV_WHEELIE_DONE, 2.4f, w, 0, rec ? 2 : 1);
        }
    }
    b->stoppie_hold = (l->pitch_deg < STOPPIE_DEG && l->lon_g < -0.5f) ? b->stoppie_hold + dt : 0.0f;
    if (b->stoppie_hold > 0.2f && r->ev != LB_EV_STOPPIE) react(b, LB_EV_STOPPIE, -1.0f, 0, 0, 1);
    if (r->ev == LB_EV_STOPPIE && l->pitch_deg > -3.0f) linger(b, LB_EV_STOPPIE, 0.6f);

    /* ---- lying on its side ---- */
    b->tip_hold = b->lean_abs > TIP_DEG ? b->tip_hold + dt : 0.0f;
    if (b->tip_hold > TIP_HOLD_S && r->ev != LB_EV_TIPOVER) react(b, LB_EV_TIPOVER, -1.0f, 0, side, 1);
    if (r->ev == LB_EV_TIPOVER && b->lean_abs < 40.0f) linger(b, LB_EV_TIPOVER, 1.0f);

out:;
    /* ---- what shows: a reaction, else the oldest waiting report once upright ---- */
    b->upright_s = b->lean_abs < REPORT_UPRIGHT_DEG ? b->upright_s + dt : 0.0f;
    for (int i = b->nreports - 1; i >= 0; i--) {
        b->reports[i].age += dt;
        if (b->reports[i].t <= 0.0f && b->reports[i].age > REPORT_MAX_AGE_S) drop_report(b, i);
    }
    lb_event_state *show = 0;
    if (r->ev != LB_EV_NONE) {
        show = r;
    } else if (b->nreports && b->act != LB_ACT_ASLEEP) {
        lb_event_state *head = &b->reports[0];
        /* once started, a report keeps showing unless the bike leans again; after
         * that it resumes only once the bike is properly upright again */
        int ok = head->t > 0.0f && b->ev == head->ev ? b->lean_abs < REPORT_HIDE_DEG
                                                      : b->upright_s > REPORT_UPRIGHT_S;
        if (ok) {
            head->t += dt;
            if (head->t >= head->dur) drop_report(b, 0);
            else show = head;
        }
    }
    if (show) {
        b->ev = show->ev; b->ev_t = show->t; b->ev_value = show->value;
        b->ev_side = show->side; b->ev_level = show->level;
    } else {
        b->ev = LB_EV_NONE; b->ev_t = 0.0f; b->ev_value = 0.0f; b->ev_side = 0; b->ev_level = 0;
    }
}
