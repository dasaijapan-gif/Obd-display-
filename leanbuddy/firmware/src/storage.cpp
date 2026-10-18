#include "storage.h"

#include <Preferences.h>

namespace {
Preferences prefs;
constexpr uint8_t VERSION = 1;  // bump when a stored struct changes shape

template <typename T>
bool load(const char *key, T &out) {
    uint8_t buf[sizeof(T) + 1];
    if (prefs.getBytesLength(key) != sizeof buf) return false;
    prefs.getBytes(key, buf, sizeof buf);
    if (buf[0] != VERSION) return false;
    memcpy(&out, buf + 1, sizeof(T));
    return true;
}

template <typename T>
void save(const char *key, const T &v) {
    uint8_t buf[sizeof(T) + 1];
    buf[0] = VERSION;
    memcpy(buf + 1, &v, sizeof(T));
    prefs.putBytes(key, buf, sizeof buf);
}
}  // namespace

namespace storage {
void begin() { prefs.begin("leanbuddy", false); }
bool loadMount(lb_mount &m) { return load("mount", m) && m.valid; }
void saveMount(const lb_mount &m) { save("mount", m); }
bool loadRecords(lb_records &r) { return load("best", r); }
void saveRecords(const lb_records &r) { save("best", r); }
bool loadOrientation(Orientation &o) { return load("orient", o) && o.valid; }
void saveOrientation(const Orientation &o) { save("orient", o); }
void clearRecords() { prefs.remove("best"); }
}  // namespace storage
