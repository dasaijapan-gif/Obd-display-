/* The ride brain: turns lean and G-forces into moods, events and records.
 *
 * It decides *what* the buddy is reacting to; face.c decides how that looks.
 *
 * Safety by design: while the bike is leaned over the face only changes in ways
 * you can read from the corner of your eye (colour, eyes looking into the turn).
 * Numbers and text (the lean you just hit, a new record) are shown when the
 * corner is finished and the bike is upright again.
 */
#ifndef LB_BRAIN_H
#define LB_BRAIN_H

#include "lean.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LB_ACT_ASLEEP = 0,  /* parked for a while: screen dims */
    LB_ACT_STOPPED,     /* awake, not riding (traffic light, just parked) */
    LB_ACT_RIDING,
} lb_activity;

/* lean tiers, by |lean| */
typedef enum {
    LB_TIER_UPRIGHT = 0, /* < 10 deg */
    LB_TIER_LEAN,        /* 10-25: a normal corner */
    LB_TIER_SPIRITED,    /* 25-40 */
    LB_TIER_KNEE,        /* 40-50: knee-down territory */
    LB_TIER_ELBOW,       /* 50+ */
} lb_tier;

/* What the buddy is reacting to.  Two kinds:
 *  - reactions to what the bike is doing right now (braking, a bump, airtime...),
 *    highest priority last: a lower one never interrupts a higher one;
 *  - reports (corner, record, wheelie time), which carry text and numbers.  They
 *    wait in a short queue and only show once the bike has been upright for a
 *    moment and nothing else is happening, so a reaction is never lost to a
 *    report and text never shows mid-corner. */
typedef enum {
    LB_EV_NONE = 0,
    /* reports */
    LB_EV_CORNER,         /* peak lean of the corner just finished */
    LB_EV_WHEELIE_DONE,   /* how long the wheelie was */
    LB_EV_RECORD,         /* new personal best lean */
    /* reactions */
    LB_EV_WAKE,
    LB_EV_LAUNCH,         /* hard acceleration */
    LB_EV_BUMP,
    LB_EV_HARD_BRAKE,
    LB_EV_STOPPIE,
    LB_EV_WHEELIE,        /* while the front is up */
    LB_EV_LANDING,
    LB_EV_AIRTIME,
    LB_EV_TIPOVER,        /* lying on its side */
    LB_EV_CALIBRATED,
    LB_EV_CALIBRATING,    /* "hold the bike upright" */
    LB_EV_COUNT
} lb_event;

static inline int lb_event_is_report(lb_event e) { return e >= LB_EV_CORNER && e <= LB_EV_RECORD; }

typedef struct {
    lb_event ev;
    float t;        /* seconds since it started (reports: seconds shown so far) */
    float dur;      /* < 0: lasts while its condition holds */
    float value;    /* corner/record: lean deg; wheelie/airtime: seconds; brake: g */
    int side;       /* -1 left, +1 right */
    int level;      /* intensity: 1, 2 (corner reports: the tier) */
    float age;      /* reports: seconds since queued */
} lb_event_state;

#define LB_REPORT_QUEUE 3

typedef struct {
    float lean_left_deg, lean_right_deg; /* deepest lean each way */
    float brake_g, accel_g, corner_g;    /* peaks */
    float wheelie_s, air_s;              /* longest */
    int corners;
    float ride_s;                        /* riding time */
} lb_records;

typedef struct {
    /* --- outputs --- */
    lb_activity act;
    lb_tier tier;
    lb_event ev;          /* what is showing now: reaction, else report, else NONE */
    float ev_t;           /* seconds since it started */
    float ev_value;
    int ev_side;
    int ev_level;
    float excitement;     /* 0..1, rises fast, decays slowly */
    float bored_s;        /* cruising/stopped with nothing happening */
    float lean_deg;       /* = lean estimate */
    float lean_abs;
    lb_records ride;      /* this ride */
    lb_records best;      /* all time (persisted by the firmware) */
    int best_dirty;       /* firmware: save `best` and clear */
    int new_ride;         /* set for one update when a new ride starts */

    /* --- internal --- */
    lb_event_state react;
    lb_event_state reports[LB_REPORT_QUEUE];
    int nreports;
    float upright_s;
    float t;
    float brake_hold, launch_hold, air_hold, wheelie_hold, tip_hold, stoppie_hold;
    float wheelie_start, air_start;
    float corner_peak;
    int in_corner;
    float asleep_s;
    lb_cal_state prev_cal;
} lb_brain;

void lb_brain_init(lb_brain *b, const lb_records *best);
void lb_brain_update(lb_brain *b, const lb_lean *l, float dt);
const char *lb_event_name(lb_event e);
const char *lb_tier_word(lb_tier t);  /* "NICE", "SPICY", "KNEE DOWN", "ELBOW!" */
#define LB_REPORT_HIDE_DEG 15.0f /* face.c hides report text while leaned more than this */
lb_tier lb_tier_of(float lean_abs_deg);

#ifdef __cplusplus
}
#endif

#endif
