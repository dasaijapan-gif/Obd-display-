// Settings and records kept in flash (NVS) across power cycles.
#pragma once
#include "brain.h"
#include "lean.h"

namespace storage {
struct Orientation {
    lb_vec3 screen_normal, screen_top;  // sensor axes; from the wizard
    int valid;
};
void begin();
bool loadMount(lb_mount &m);
void saveMount(const lb_mount &m);
bool loadRecords(lb_records &r);
void saveRecords(const lb_records &r);
bool loadOrientation(Orientation &o);
void saveOrientation(const Orientation &o);
void clearRecords();
}  // namespace storage
