// LeanBuddy firmware: reads the IMU at 200 Hz on one core, and on the other runs
// the lean estimator, the ride brain and the face, drawing ~30 frames a second.
//
// Serial commands (115200 baud): h help, c recalibrate, p next page,
// o orientation wizard, d toggle telemetry stream, r reset records.
#include <Arduino.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "brain.h"
#include "config.h"
#include "display.h"
#include "face.h"
#include "lean.h"
#include "qmi8658.h"
#include "storage.h"
#include "touch.h"

namespace {

Qmi8658 imu;
Touch touch;
lb_lean lean;
lb_brain brain;
lb_face face;
lb_gfx gfx;
uint16_t *fb = nullptr;

QueueHandle_t samples;          // IMU task -> main loop
SemaphoreHandle_t i2c;          // the IMU and the touch chip share the bus
volatile uint32_t imu_errors = 0;

bool streaming = false;
uint32_t last_frame_ms = 0, last_mount_save_ms = 0, last_records_save_ms = 0, last_stream_ms = 0;
uint32_t boot_down_ms = 0;
bool boot_down = false, boot_long_sent = false;
float last_sample_t = -1.0f;
float backlight = BACKLIGHT_MAX;

// IMU time in seconds, wrapping every 100 s so a float keeps microsecond-ish
// resolution forever (the estimator skips the one backwards step)
float now_s() { return (float)(esp_timer_get_time() % 100000000LL) * 1e-6f; }

void imu_task(void *) {
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_HZ);
    for (;;) {
        float a[3], g[3];
        xSemaphoreTake(i2c, portMAX_DELAY);
        bool ok = imu.read(a, g);
        xSemaphoreGive(i2c);
        if (ok) {
            lb_imu_sample s;
            s.t = now_s();
            s.acc_g = lb_v3(a[0], a[1], a[2]);
            s.gyro_dps = lb_v3(g[0], g[1], g[2]);
            xQueueSend(samples, &s, 0);
        } else {
            imu_errors++;
        }
        vTaskDelayUntil(&wake, period);
    }
}

void message(const char *line1, const char *line2) {
    lb_gfx_identity(&gfx);
    lb_gfx_clear(&gfx, 0);
    lb_color white = lb_rgb(235, 238, 245);
    lb_draw_text_c(&gfx, line1, 120, 104, 16, 3.2f, white, 1);
    if (line2) lb_draw_text_c(&gfx, line2, 120, 136, 13, 2.6f, lb_rgb(140, 144, 160), 1);
    display::push(fb);
}

// ---- orientation wizard: measures how the IMU chip sits behind the screen ----

// averages gravity once the toy has been held still for a moment
bool wait_still(lb_vec3 &mean_out, uint32_t timeout_ms) {
    uint32_t start = millis();
    lb_vec3 mean = lb_v3(0, 0, 0);
    int still = 0;
    while (millis() - start < timeout_ms) {
        lb_imu_sample s;
        if (xQueueReceive(samples, &s, pdMS_TO_TICKS(50)) != pdTRUE) continue;
        float gyro = lb_v3_len(s.gyro_dps);
        float dev = lb_v3_len(lb_v3_sub(s.acc_g, mean));
        if (gyro < 5.0f && (still == 0 || dev < 0.05f)) {
            mean = still == 0 ? s.acc_g : lb_v3_add(mean, lb_v3_scale(lb_v3_sub(s.acc_g, mean), 0.02f));
            if (++still > IMU_HZ * 2) { mean_out = lb_v3_norm(mean); return true; }
        } else {
            still = 0;
        }
    }
    return false;
}

void orientation_wizard() {
    Serial.println("Orientation 1/2: lay the toy flat, screen facing UP, and keep it still.");
    message("SCREEN UP", "HOLD STILL");
    lb_vec3 normal, top;
    if (!wait_still(normal, 60000)) { Serial.println("Timed out."); return; }
    Serial.println("Orientation 2/2: hold it upright, screen facing YOU, and keep it still.");
    message("FACE ME", "HOLD UPRIGHT");
    if (!wait_still(top, 60000)) { Serial.println("Timed out."); return; }
    // gravity points "up" in an accelerometer: flat it is the screen normal,
    // upright it is the top of the picture (made perpendicular to the normal)
    top = lb_v3_norm(lb_v3_sub(top, lb_v3_scale(normal, lb_v3_dot(top, normal))));
    storage::Orientation o = {normal, top, 1};
    storage::saveOrientation(o);
    lb_lean_set_orientation(&lean, normal, top);
    lb_lean_request_calibration(&lean);
    Serial.printf("Saved: screen normal (%.2f %.2f %.2f), top (%.2f %.2f %.2f). Now mount it and hold the bike upright.\n",
                  normal.x, normal.y, normal.z, top.x, top.y, top.z);
}

// ---- inputs ----

void recalibrate() {
    lb_lean_request_calibration(&lean);
    Serial.println("Recalibrating: hold the bike upright and still.");
}

void poll_inputs(uint32_t now) {
    xSemaphoreTake(i2c, portMAX_DELAY);
    Press p = touch.poll(now);
    xSemaphoreGive(i2c);
    if (p == Press::Tap) lb_face_next_page(&face);
    if (p == Press::Long) recalibrate();

    bool down = digitalRead(PIN_BOOT_BUTTON) == LOW;
    if (down && !boot_down) { boot_down = true; boot_long_sent = false; boot_down_ms = now; }
    if (down && boot_down && !boot_long_sent && now - boot_down_ms > 2000) { boot_long_sent = true; recalibrate(); }
    if (!down && boot_down) {
        boot_down = false;
        if (!boot_long_sent && now - boot_down_ms > 30) lb_face_next_page(&face);
    }

    while (Serial.available()) {
        switch (Serial.read()) {
        case 'c': recalibrate(); break;
        case 'p': lb_face_next_page(&face); break;
        case 'o': orientation_wizard(); break;
        case 'd': streaming = !streaming; if (streaming) Serial.println("t,lean,pitch,lon_g,lat_g,vert_g,event,activity,battery_v"); break;
        case 'r': {
            lb_records z = {};
            brain.best = z;
            brain.ride = z;
            storage::clearRecords();
            Serial.println("Records cleared.");
            break;
        }
        case 'h': case '?':
            Serial.println("c recalibrate | p next page | o orientation wizard | d telemetry on/off | r reset records");
            break;
        default: break;
        }
    }
}

void save_if_needed(uint32_t now) {
    if (lean.mount_changed && now - last_mount_save_ms > SAVE_MOUNT_EVERY_MS) {
        storage::saveMount(lean.mount);
        lean.mount_changed = 0;
        last_mount_save_ms = now;
    }
    if (brain.best_dirty && (now - last_records_save_ms > SAVE_RECORDS_EVERY_MS || brain.act == LB_ACT_ASLEEP)) {
        storage::saveRecords(brain.best);
        brain.best_dirty = 0;
        last_records_save_ms = now;
    }
}

float battery_v() { return analogReadMilliVolts(PIN_BAT_ADC) * BAT_DIVIDER / 1000.0f; }

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
    storage::begin();

    fb = (uint16_t *)heap_caps_malloc(LB_W * LB_H * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!fb || !display::begin()) {
        Serial.println("Display init failed.");
        for (;;) delay(1000);
    }
    lb_gfx_init(&gfx, fb);
    display::brightness(BACKLIGHT_MAX);

    lb_lean_init(&lean);
    storage::Orientation o;
    if (storage::loadOrientation(o)) lb_lean_set_orientation(&lean, o.screen_normal, o.screen_top);
    else lb_lean_set_orientation(&lean, lb_v3(IMU_SCREEN_NORMAL_X, IMU_SCREEN_NORMAL_Y, IMU_SCREEN_NORMAL_Z),
                                 lb_v3(IMU_SCREEN_TOP_X, IMU_SCREEN_TOP_Y, IMU_SCREEN_TOP_Z));
    lb_mount m;
    if (storage::loadMount(m)) lb_lean_set_mount(&lean, &m); // else: calibrates the first time the bike is still

    lb_records best;
    bool have_best = storage::loadRecords(best);
    lb_brain_init(&brain, have_best ? &best : nullptr);
    lb_face_init(&face, esp_random());

    if (!imu.begin(Wire)) {
        Serial.println("No QMI8658 found on I2C.");
        message("NO SENSOR", "CHECK BOARD");
        for (;;) delay(1000);
    }
    Serial.printf("QMI8658 at 0x%02X. Send h for commands.\n", imu.address());
    touch.begin(Wire);

    samples = xQueueCreate(64, sizeof(lb_imu_sample));
    i2c = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, nullptr, 5, nullptr, 0);
    last_frame_ms = millis();
}

void loop() {
    uint32_t now = millis();

    lb_imu_sample s;
    while (xQueueReceive(samples, &s, 0) == pdTRUE) {
        float dt = s.t - last_sample_t;
        if (last_sample_t < 0.0f || dt <= 0.0f || dt > 0.1f) dt = 1.0f / IMU_HZ;
        last_sample_t = s.t;
        lb_lean_update(&lean, &s);
        lb_brain_update(&brain, &lean, dt);
    }

    poll_inputs(now);

    if (now - last_frame_ms >= FRAME_MS) {
        float dt = (now - last_frame_ms) / 1000.0f;
        last_frame_ms = now;
        lb_face_update(&face, &brain, &lean, dt);
        lb_face_draw(&face, &brain, &lean, &gfx);
        display::push(fb);
        float target = BACKLIGHT_ASLEEP + (BACKLIGHT_MAX - BACKLIGHT_ASLEEP) * face.brightness;
        backlight += (target - backlight) * 0.2f;
        display::brightness((uint8_t)backlight);
    }

    save_if_needed(now);

    if (streaming && now - last_stream_ms >= 50) {
        last_stream_ms = now;
        Serial.printf("%.2f,%.1f,%.1f,%.2f,%.2f,%.2f,%s,%d,%.2f\n", last_sample_t, lean.lean_deg, lean.pitch_deg, lean.lon_g,
                      lean.lat_g, lean.vert_g_fast, lb_event_name(brain.ev), (int)brain.act, battery_v());
    }
    delay(1);
}
