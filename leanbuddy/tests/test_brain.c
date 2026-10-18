/* Ride events, reports and records, on the simulated ride and on forced inputs. */
#include <string.h>
#include "brain.h"
#include "lbtest.h"
#include "ridesim.h"

typedef struct {
    int seen[LB_EV_COUNT];
    int max_level[LB_EV_COUNT];
    int asleep, woke;
    int report_while_leaned;   /* text on screen while leaned over: must never happen */
    float report_start_lean;   /* worst lean at the moment a report appeared */
    lb_brain b;
} ride_log;

static ride_log ride(float seconds) {
    ride_log log;
    memset(&log, 0, sizeof log);
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    lb_lean l;
    lb_lean_init(&l);
    lb_brain_init(&log.b, 0);
    lb_event prev = LB_EV_NONE;
    lb_activity pact = LB_ACT_STOPPED;
    const float dt = 1.0f / 200.0f;
    while (r.t < seconds) {
        lb_imu_sample s;
        lb_ridesim_step(&r, dt, &s);
        lb_lean_update(&l, &s);
        lb_brain_update(&log.b, &l, dt);
        lb_brain *b = &log.b;
        log.seen[b->ev]++;
        if (b->ev_level > log.max_level[b->ev]) log.max_level[b->ev] = b->ev_level;
        if (lb_event_is_report(b->ev)) {
            if (b->lean_abs >= LB_REPORT_HIDE_DEG) log.report_while_leaned++;
            if (b->ev != prev && b->lean_abs > log.report_start_lean) log.report_start_lean = b->lean_abs;
        }
        if (b->act == LB_ACT_ASLEEP) log.asleep++;
        if (pact == LB_ACT_ASLEEP && b->act != LB_ACT_ASLEEP) log.woke++;
        prev = b->ev;
        pact = b->act;
    }
    return log;
}

static void test_canyon_events(void) {
    lb_ridesim r;
    lb_ridesim_init(&r, LB_SCRIPT_CANYON, 7);
    ride_log log = ride(lb_ridesim_script_length(&r) + 12.0f); /* one loop and a wake-up */
    const lb_event expected[] = {LB_EV_CALIBRATING, LB_EV_CALIBRATED, LB_EV_LAUNCH, LB_EV_RECORD,
                                 LB_EV_HARD_BRAKE, LB_EV_BUMP, LB_EV_WHEELIE, LB_EV_WHEELIE_DONE,
                                 LB_EV_AIRTIME, LB_EV_LANDING, LB_EV_CORNER, LB_EV_WAKE};
    for (unsigned i = 0; i < sizeof expected / sizeof expected[0]; i++)
        CHECK(log.seen[expected[i]] > 0, "never saw %s", lb_event_name(expected[i]));
    CHECK(log.max_level[LB_EV_HARD_BRAKE] == 2, "the big stop should be a level-2 brake");
    CHECK(log.seen[LB_EV_TIPOVER] == 0, "no tip-over on a normal ride");
    CHECK(log.seen[LB_EV_STOPPIE] == 0, "no stoppie in this script");
    CHECK(log.asleep > 0, "falls asleep when parked");
    CHECK(log.woke == 1, "wakes when the engine starts");
    CHECK(log.report_while_leaned == 0, "%d updates showed text while leaned over", log.report_while_leaned);
    CHECK(log.report_start_lean < 8.0f, "a report appeared at %.1f deg of lean", log.report_start_lean);

    const lb_records *ride = &log.b.ride;
    CHECK_NEAR(ride->lean_left_deg, 44.0, 2.5);
    CHECK_NEAR(ride->lean_right_deg, 47.0, 2.5);
    CHECK(ride->brake_g > 0.75f && ride->brake_g < 0.95f, "brake %.2f", ride->brake_g);
    CHECK(ride->wheelie_s > 1.2f && ride->wheelie_s < 2.6f, "wheelie %.2f", ride->wheelie_s);
    CHECK(ride->air_s > 0.3f && ride->air_s < 0.6f, "air %.2f", ride->air_s);
    CHECK(ride->corners >= 6, "corners %d", ride->corners);
    CHECK(log.b.best.lean_right_deg >= ride->lean_right_deg - 0.01f, "best tracks the ride");
    CHECK(log.b.best_dirty, "records to save");
}

/* drive the brain with a hand-made lean state */
typedef struct {
    lb_lean l;
    lb_brain b;
} rig;

static void rig_init(rig *g) {
    lb_lean_init(&g->l);
    g->l.cal = LB_CAL_DONE;
    g->l.moving = 1;
    g->l.vert_g = g->l.vert_g_fast = 1.0f;
    lb_brain_init(&g->b, 0);
}

static void step(rig *g, float seconds) {
    for (float t = 0; t < seconds; t += 0.01f) lb_brain_update(&g->b, &g->l, 0.01f);
}

static void corner(rig *g, float peak, float secs) {
    for (float t = 0; t < secs; t += 0.01f) {
        g->l.lean_deg = peak * lb_sinf(LB_PI * t / secs);
        lb_brain_update(&g->b, &g->l, 0.01f);
    }
    g->l.lean_deg = 0;
}

static void test_records_and_s_bends(void) {
    rig g;
    rig_init(&g);
    corner(&g, -30, 3);
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_RECORD && g.b.ev_side == -1, "first big left is a record (%s)", lb_event_name(g.b.ev));
    CHECK_NEAR(g.b.ev_value, 30, 0.5);
    step(&g, 4);
    corner(&g, -28, 3);
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_CORNER, "shallower: a plain corner report (%s)", lb_event_name(g.b.ev));
    CHECK(g.b.ev_level == LB_TIER_SPIRITED, "tier %d", g.b.ev_level);
    step(&g, 4);
    /* S-bend: left 20 then straight into right 35; the left report must not show mid-bend */
    corner(&g, -20, 2);
    corner(&g, 35, 3);
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_RECORD && g.b.ev_side == 1, "the right is a record (%s)", lb_event_name(g.b.ev));
    CHECK(g.b.nreports == 1, "the stale left-corner report was dropped (%d queued)", g.b.nreports);
}

static void test_reactions_beat_reports(void) {
    rig g;
    rig_init(&g);
    corner(&g, 40, 3);
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_RECORD, "record showing");
    g.l.lon_g = -0.9f;          /* hard braking interrupts it */
    step(&g, 0.4f);
    CHECK(g.b.ev == LB_EV_HARD_BRAKE && g.b.ev_level == 2, "brake (%s)", lb_event_name(g.b.ev));
    g.l.vert_g_fast = 2.4f;     /* a bump is lower priority than braking */
    step(&g, 0.02f);
    CHECK(g.b.ev == LB_EV_HARD_BRAKE, "a bump does not interrupt braking");
    g.l.vert_g_fast = 1.0f;
    g.l.lon_g = 0;
    step(&g, 1.0f);
    CHECK(g.b.ev == LB_EV_RECORD, "the record resumes afterwards (%s)", lb_event_name(g.b.ev));
    step(&g, 4.0f);
    CHECK(g.b.ev == LB_EV_NONE, "and finishes");
}

static void test_tipover_sleep_and_new_ride(void) {
    rig g;
    rig_init(&g);
    lb_records best = {0};
    best.lean_left_deg = 51;
    lb_brain_init(&g.b, &best);
    g.l.lean_deg = 80;
    step(&g, 2.0f);
    CHECK(g.b.ev == LB_EV_TIPOVER, "lying on its side (%s)", lb_event_name(g.b.ev));
    g.l.lean_deg = 0;
    step(&g, 1.5f);
    CHECK(g.b.ev == LB_EV_NONE, "back up");

    g.b.ride.corners = 9;
    g.l.moving = 0;
    g.l.still = 1;
    for (float t = 0; t < 400; t += 0.1f) { g.l.still_s = t; lb_brain_update(&g.b, &g.l, 0.1f); }
    CHECK(g.b.act == LB_ACT_ASLEEP, "asleep");
    g.l.still = 0;
    g.l.still_s = 0;
    g.l.moving = 1;
    step(&g, 0.1f);
    CHECK(g.b.ev == LB_EV_WAKE, "wake");
    CHECK(g.b.ride.corners == 0, "a long sleep starts a new ride");
    CHECK(g.b.best.lean_left_deg == 51, "all-time best is kept");
}

static void test_calibration_events(void) {
    rig g;
    rig_init(&g);
    g.l.cal = LB_CAL_WAITING;
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_CALIBRATING, "calibrating");
    g.l.lon_g = -0.9f;
    step(&g, 0.5f);
    CHECK(g.b.ev == LB_EV_CALIBRATING, "nothing else while calibrating");
    g.l.lon_g = 0;
    g.l.cal = LB_CAL_DONE;
    step(&g, 0.1f);
    CHECK(g.b.ev == LB_EV_CALIBRATED, "calibrated");
}

int main(void) {
    RUN(test_canyon_events);
    RUN(test_records_and_s_bends);
    RUN(test_reactions_beat_reports);
    RUN(test_tipover_sleep_and_new_ride);
    RUN(test_calibration_events);
    DONE();
}
