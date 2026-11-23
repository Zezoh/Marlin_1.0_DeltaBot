// G30 / G33 on a simulated delta: the real probe.cpp, delta_calibration.cpp and Marlin_main.cpp kinematics run against a machine
// whose real geometry differs from the firmware's settings. The FSR fires when the true nozzle touches the true (flat) bed.
#include "kin_src.h"
#include <assert.h>
#include <stdint.h>
#include "delta_calibration.h"
#include "probe.h"

// ---- firmware globals the shims declare ----
float current_position[4] = {0, 0, 0, 0}, add_homing[3] = {0, 0, 0}, endstop_adj[3] = {0, 0, 0};
bool axis_known_position[3] = {false, false, false};
uint8_t active_extruder = 0;
float axis_steps_per_unit[4] = {80, 80, 80, 98};
volatile unsigned int fsr_raw = 500;
int g_verbose = 0, g_stores = 0;

// ---- the simulated machine ----
static DeltaGeometry truth;          // real tower radius, rod, angles (off[] unused)
static float switch_h[3];            // real carriage height at which each endstop switch fires
static float bed_z = 0;              // real height of the bed in the true frame
static float believed[3];            // tower heights the firmware believes (what the stepper counts say)
static float off[3];                 // real carriage height minus believed one, fixed at the last homing
static bool armed = false, hit = false;
static unsigned long now_ms = 0;
static float contact_noise = 0;
static int strokes = 0, homes = 0;
static bool transient_fast = false, transient_slow_once = false, slow_used = false;   // the FSR fires at the start of a move (vibration)
static float feed_now = 0;
static int low_travels = 0, drags = 0;

static float frand() { return 2.0f * rand() / RAND_MAX - 1.0f; }
static float true_z(const float tower[3]) {
  float h[3] = {tower[0] + off[0], tower[1] + off[1], tower[2] + off[2]}, z = -1e9;
  dcal_effector_z(truth, h, z);
  return z;
}

unsigned long millis() { return now_ms; }
void ui_busy(bool) {}
bool ui_cancel_requested() { return false; }
void manage_heater() {}
void manage_inactivity(bool) { now_ms++; }
bool IsStopped() { return false; }
void keep_steppers_on() {}

void home_all_axes() {
  homes++;
  current_position[0] = 0; current_position[1] = 0; current_position[2] = Z_HOME_POS + add_homing[Z_AXIS];
  calculate_delta(current_position);
  for (int a = 0; a < 3; a++) {
    believed[a] = delta[a];
    off[a] = switch_h[a] + (endstop_adj[a] < 0 ? endstop_adj[a] : 0) - believed[a];   // Marlin retraces by a negative endstop_adj only
    axis_known_position[a] = true;
  }
}
void plan_set_position(const float &x, const float &y, const float &z, const float &e) {
  believed[0] = lround(x * 80) / 80.0f; believed[1] = lround(y * 80) / 80.0f; believed[2] = lround(z * 80) / 80.0f;
}
long st_get_position(uint8_t a) { return lround(believed[a] * 80); }
void st_probe_arm(unsigned int) { armed = true; hit = false; }
void st_probe_disarm() { armed = false; }
bool st_probe_hit() { return hit; }
unsigned int st_probe_peak() { return 0; }

static float target[3]; static bool queued = false;
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed, const uint8_t &, float) {
  target[0] = x; target[1] = y; target[2] = z; queued = true; feed_now = feed;
}
void st_synchronize() {
  if (!queued) return;
  queued = false;
  float p[3] = {believed[0], believed[1], believed[2]};
  float len = 0; for (int a = 0; a < 3; a++) len = fmax(len, fabs(target[a] - p[a]));
  long n = (long)ceil(len * 80);
  for (long k = 1; k <= n; k++) {
    float q[3]; for (int a = 0; a < 3; a++) q[a] = lround((p[a] + (target[a] - p[a]) * k / n) * 80) / 80.0f;
    float z = true_z(q);
    bool bump = armed && k <= 10 && ((transient_fast && feed_now > 20) || (transient_slow_once && feed_now < 20 && !slow_used));
    if (bump && feed_now < 20) slow_used = true;
    if (bump || (armed && z <= bed_z + contact_noise * frand())) {
      for (int a = 0; a < 3; a++) believed[a] = q[a];
      hit = true; strokes++;
      return;
    }
    assert(armed || z > bed_z - 0.001 || true);
    if (z < bed_z - 1.0) { printf("CRASH: the nozzle went %.2f mm into the bed\n", bed_z - z); assert(0); }
  }
  for (int a = 0; a < 3; a++) believed[a] = target[a];
}
void move_to(float x, float y, float z, float feed) {
  slow_used = false;   // moves between probe points: no probing, straight in Cartesian space
  float c[3] = {x, y, z}; calculate_delta(c);
  if (hypot(x - current_position[0], y - current_position[1]) > 1.0) {   // a travel move: does the nozzle clear the bed on the way?
    if (z < 15) low_travels++;
    for (int k = 0; k <= 10; k++) {
      float cc[3] = {current_position[0] + (x - current_position[0]) * k / 10, current_position[1] + (y - current_position[1]) * k / 10, z};
      calculate_delta(cc); float t[3] = {delta[0], delta[1], delta[2]};
      if (true_z(t) < bed_z + 1.0) drags++;
    }
    calculate_delta(c);
  }
  for (int a = 0; a < 3; a++) believed[a] = lround(delta[a] * 80) / 80.0f;
  current_position[0] = x; current_position[1] = y; current_position[2] = z;
}

// ---- helpers ----
static void firmware_defaults() {
  delta_radius = 90; delta_diagonal_rod = 210; delta_tower_angle[0] = delta_tower_angle[1] = delta_tower_angle[2] = 0;
  endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = 0; add_homing[2] = 0;
  recalc_delta_settings(delta_radius, delta_diagonal_rod);
  delta_calibrated = false; probe_z_offset = 0; for (int a = 0; a < 3; a++) axis_known_position[a] = false;
}
static void machine(float R, float L, float a0, float a1, float a2, float e0, float e1, float e2) {
  truth.R = R; truth.L = L; truth.ang[0] = a0; truth.ang[1] = a1; truth.ang[2] = a2;
  float H = sqrt(L * L - R * R) + Z_HOME_POS;
  switch_h[0] = H + e0; switch_h[1] = H + e1; switch_h[2] = H + e2;
  bed_z = 0;
}
// worst height error over the bed when the firmware commands z = 0
static double flatness(double radius, double &mean) {
  double worst = 0; mean = 0; int n = 0;
  for (float x = -radius; x <= radius; x += 10) for (float y = -radius; y <= radius; y += 10) {
    if (x * x + y * y > radius * radius) continue;
    float c[3] = {x, y, 0}; calculate_delta(c);
    float t[3] = {delta[0], delta[1], delta[2]};
    double z = true_z(t) - bed_z; mean += z; n++;
    if (fabs(z) > worst) worst = fabs(z);
  }
  mean /= n; return worst;
}

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(int argc, char **argv) {
  g_verbose = argc > 1;
  srand(3);
  // 1. wrong radius, tower angles and endstop heights: G33 finds them
  { firmware_defaults(); machine(91.2, 210, 0.15, -0.10, 0.0, 0.8, -0.5, 0.3); homes = 0; strokes = 0;
    double m, w0 = (home_all_axes(), flatness(70, m));
    probe_g33(6, 10, true);
    double w = flatness(70, m);
    printf("1. before: bed error up to %.3f mm; after G33: %.3f mm (mean %.3f); %d strokes, %d homings; R=%.3f angles %.3f %.3f %.3f adj %.3f %.3f %.3f homeZ %.3f\n", w0, w, m, strokes, homes, delta_radius, delta_tower_angle[0], delta_tower_angle[1], delta_tower_angle[2], endstop_adj[0], endstop_adj[1], endstop_adj[2], add_homing[2]);
    CHECK(delta_calibrated, "calibrated flag");
    CHECK(g_stores == 1, "saved once (%d)", g_stores);
    CHECK(w < 0.1, "bed within 0.1 mm over the printable area (%.3f)", w);
    CHECK(fabs(delta_radius - 91.2) < 0.15, "radius found (%.3f)", delta_radius);
    CHECK(endstop_adj[0] <= 0 && endstop_adj[1] <= 0 && endstop_adj[2] <= 0, "endstop trims are retractions only");
    CHECK(fabs(endstop_adj[0] - endstop_adj[1]) > 0.5, "the tower trims differ (towers were off by different amounts)");
  }
  // 2. only the endstops are off: F3 is enough, and one probing pass plus a verification pass
  { firmware_defaults(); machine(90, 210, 0, 0, 0, 1.5, 0.0, -0.7); g_stores = 0;
    home_all_axes(); probe_g33(3, 7, false);
    double m, w = flatness(70, m);
    printf("2. endstops only, quick: bed error %.3f mm, R %.3f, saved %d times\n", w, delta_radius, g_stores);
    CHECK(delta_calibrated && w < 0.1 && g_stores == 0 && fabs(delta_radius - 90) < 1e-3, "endstops only"); }
  // 3. a machine that is already right: one verification pass, nothing changes
  { firmware_defaults(); machine(90, 210, 0, 0, 0, 0, 0, 0); homes = 0; home_all_axes(); homes = 0;
    probe_g33(6, 10, false);
    printf("3. already calibrated: %d homings, R %.4f\n", homes, delta_radius);
    CHECK(delta_calibrated && homes == 0 && fabs(delta_radius - 90) < 1e-3, "no change for a good machine"); }
  // 4. the probe is noisy (contact threshold jitter)
  { firmware_defaults(); machine(90.8, 210, -0.12, 0.2, 0.0, 0.4, 0.9, -0.2); contact_noise = 0.015; home_all_axes();
    probe_g33(6, 10, false); double m, w = flatness(70, m); contact_noise = 0;
    printf("4. noisy probe (15 um): bed error %.3f mm\n", w);
    CHECK(delta_calibrated && w < 0.12, "calibration with a noisy probe (%.3f)", w); }
  // 5. the bed is out of reach: G33 fails and restores everything
  { firmware_defaults(); machine(90, 210, 0, 0, 0, 0, 0, 0); bed_z = -60; home_all_axes(); homes = 0;
    probe_g33(6, 10, true);
    printf("5. no bed within reach: calibrated %d, R %.3f, re-homed %d times\n", delta_calibrated, delta_radius, homes);
    CHECK(!delta_calibrated && delta_radius == 90 && endstop_adj[0] == 0 && add_homing[2] == 0, "failure restores the settings"); bed_z = 0; }
  // 6. a result outside the allowed range is refused and the old settings are restored
  { firmware_defaults(); machine(112, 210, 0, 0, 0, 0, 0, 0); home_all_axes(); homes = 0; g_stores = 0;
    probe_g33(6, 10, true);
    printf("6. radius 22 mm off: calibrated %d, R %.3f (must be restored to 90), stores %d\n", delta_calibrated, delta_radius, g_stores);
    CHECK(!delta_calibrated && fabs(delta_radius - 90) < 1e-6 && g_stores == 0, "refused"); }
  // 7. G30 reports the height of the bed under the nozzle
  { firmware_defaults(); machine(90, 210, 0, 0, 0, 0, 0, 0); bed_z = 0.37; home_all_axes(); move_to(20, -10, 30, 100); st_synchronize();
    g_verbose = 1; printf("7. G30 with the bed 0.37 mm higher than the firmware thinks: "); probe_g30(); g_verbose = argc > 1;
    CHECK(fabs(current_position[2] - 0.37 - 1.5) < 0.3 || current_position[2] > 0, "left lifted"); }
  // 8. the FSR fires when the fast move starts and when the slow one starts: no wrong heights
  { firmware_defaults(); machine(90.9, 210, 0.1, -0.1, 0.0, 0.5, -0.4, 0.2); transient_fast = transient_slow_once = true; home_all_axes();
    probe_g33(6, 10, false); double m, w = flatness(70, m); transient_fast = transient_slow_once = false;
    printf("8. vibration triggers at the start of moves: bed error %.3f mm\n", w);
    CHECK(delta_calibrated && w < 0.12, "calibration with start-of-move triggers (%.3f)", w); }
  // 9. a badly built machine: the bed is several mm off and tilted in the first pass
  { firmware_defaults(); machine(92.0, 210, 0.3, -0.3, 0.2, 3.0, -2.0, 1.0); home_all_axes(); double m, w0 = flatness(70, m); homes = 0;
    probe_g33(6, 10, false); double w = flatness(70, m);
    printf("9. start %.2f mm off: after G33 %.3f mm, %d homings\n", w0, w, homes);
    CHECK(delta_calibrated && w < 0.12, "calibration of a badly built machine (%.3f)", w); }
  // 10. the bed is 5 mm below where the firmware thinks: pass 2 must not treat the new heights as noise; low travel moves
  { firmware_defaults(); machine(90.5, 210, 0.1, 0.0, 0.0, 5.2, 4.8, 5.0); fsr_sensitivity = 12; home_all_axes(); low_travels = drags = 0;
    probe_g33(6, 10, false); double m, w = flatness(70, m);
    printf("10. bed 5 mm too low at the start: after G33 %.3f mm, FSR sensitivity %u, %d low travel moves, %d moves dragging on the bed\n", w, (unsigned)fsr_sensitivity, low_travels, drags);
    CHECK(delta_calibrated && w < 0.12 && fsr_sensitivity == 12, "second pass after a large correction");
    CHECK(low_travels >= 8 && drags == 0, "travel moves stay low (%d) and clear the bed (%d)", low_travels, drags); }
  // 11. the rod is 2 mm longer than the firmware thinks: the radius absorbs it and the bed is still flat
  { firmware_defaults(); machine(92.0, 212.0, 0.2, -0.1, 0.0, 1.0, 0.0, -1.0); home_all_axes();
    probe_g33(6, 10, false); double m, w = flatness(70, m);
    printf("11. rod 2 mm longer: bed error %.3f mm\n", w);
    CHECK(w < 0.1, "a wrong rod length is absorbed (%.3f)", w); }
  printf(fails ? "probe: %d FAILED\n" : "probe: all ok\n", fails);
  return fails ? 1 : 0;
}
