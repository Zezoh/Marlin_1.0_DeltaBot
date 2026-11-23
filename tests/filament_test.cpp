// M600 and the runout sensor on a simulated machine: the real filament.cpp runs against a heater model, a sensor and a "user"
// who pulls the old filament out, pushes the new one in and clicks the button.
#include "filament.h"
#include <stdio.h>
#include <vector>
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "ui.h"
#include "cardreader.h"

// ---- what the shims declare ----
float current_position[4] = {0, 0, 0, 0}, max_pos[3] = {85, 85, 225};
bool axis_known_position[3] = {true, true, true};
int g_verbose = 0;
CardReader card;
bool cancel_heatup = false;

static unsigned long now_ms = 1000;
static float temp = 20, target = 0;
static int pin_level = 0;           // LOW = filament there (RUNOUT_ACTIVE_HIGH)
static std::vector<float> e_moves;  // every extruder move
static std::vector<std::vector<float>> xyz_moves;
static int cancels = 0, homes = 0, heater_off = 0, cold_extrusions = 0, busy_depth = 0, mode = 0;
static float e_back_total = 0, last_e_set = -999;
static bool cancel_flag = false, short_flag = false;
static float max_chunk = 0;

// the "user"
static bool user_m108 = false;
static bool user_pulls_out = true, user_inserts = true, user_cancels = false, user_clicks_early = false;
static unsigned long insert_after = 3000, click_after = 2000, cancel_after = 2000;
static float target_at_cancel = -1;
static bool pulled = false, inserted = false;
static unsigned long wait_began = 0, inserted_at = 0, first_wait = 0;
static bool early_done = false;

int sim_read(int) { return pin_level; }
extern "C" void sim_write(int, int) {}
unsigned long millis() { return now_ms; }
bool IsStopped() { return false; }
static void advance(unsigned long ms)
{
  float goal = target > 0 ? target : 20;
  temp = goal + (temp - goal) * expf(-(float)ms / 12000.0f);
  now_ms += ms;
}
void manage_heater() {}
void manage_inactivity(bool)
{
  advance(1);
  filament_sensor_poll();
  if (user_pulls_out && !pulled && e_back_total >= 700) {
    pin_level = 1;
    pulled = true;
  }  // the old filament leaves the sensor
  if (mode == 2) {
    unsigned long waited = now_ms - wait_began;
    if (user_cancels && waited > cancel_after) {
      if (!cancel_flag) target_at_cancel = target;
      cancel_flag = true;
    }
    if (user_m108 && waited > 1000) cancel_heatup = true;
    if (!inserted && user_inserts && now_ms - first_wait >= insert_after) {
      pin_level = 0;
      inserted = true;
      inserted_at = now_ms;
    }
    if (user_clicks_early && !early_done && !inserted && waited >= 500) {
      short_flag = true;
      early_done = true;
    }
    bool can_click = inserted || !user_inserts;
    if (can_click && now_ms >= wait_began + click_after && now_ms >= inserted_at + 1500) short_flag = true;
  }
}
bool machine_idle() { return true; }
// verbatim from Marlin_main.cpp (pure logic; filament_hotkey uses it)
bool command_is(const char *line, char letter, int code)
{
  char c;
  while (*line == ' ') line++;
  if (*line == 'N') {  // optional line-number prefix: "N123 M108"
    while ((c = *(++line)) >= '0' && c <= '9') {}
    while (*line == ' ') line++;
  }
  if (*line != letter) return false;
  int v = 0;
  while ((c = *(++line)) >= '0' && c <= '9') v = v * 10 + (c - '0');
  return v == code && (c == 0 || c == ' ' || c == '\r' || c == '\n' || c == '*');
}
bool blocks_queued() { return false; }
void st_synchronize() {}
void plan_set_e_position(const float &e) { last_e_set = e; }
float degHotend(uint8_t) { return temp; }
float degTargetHotend(uint8_t) { return target; }
void setTargetHotend(const float &c, uint8_t) { target = c; }
void disable_heater()
{
  target = 0;
  heater_off++;
}
void home_all_axes()
{
  homes++;
  current_position[0] = current_position[1] = 0;
  current_position[2] = 225;
}
void cancel_print() { cancels++; }
void ui_defer_cancel() { cancels++; }
void move_to(float x, float y, float z, float feed)
{
  float d = sqrtf((x - current_position[0]) * (x - current_position[0]) + (y - current_position[1]) * (y - current_position[1]) + (z - current_position[2]) * (z - current_position[2]));
  xyz_moves.push_back({x, y, z});
  current_position[0] = x;
  current_position[1] = y;
  current_position[2] = z;
  advance((unsigned long)(d / feed * 1000));
}
void move_e(float mm, float feed)
{
  if (temp < 170) cold_extrusions++;
  e_moves.push_back(mm);
  if (fabsf(mm) > max_chunk) max_chunk = fabsf(mm);
  if (mm < 0) e_back_total += -mm;
  current_position[3] += mm;
  advance((unsigned long)(fabsf(mm) / feed * 1000));
}
void ui_busy(bool b)
{
  busy_depth += b ? 1 : -1;
  if (busy_depth == 0) cancel_flag = false;
}
bool ui_cancel_requested() { return cancel_flag; }
void ui_filament_mode(uint8_t m)
{
  if (m == 2 && mode != 2) {
    wait_began = now_ms;
    if (!first_wait) first_wait = now_ms;
  }
  mode = m;
}
bool ui_take_short()
{
  bool s = short_flag;
  short_flag = false;
  return s;
}
void ui_virtual_short() { short_flag = true; }

static int fails = 0;
#define CHECK(c, ...)      \
  do {                     \
    if (!(c)) {            \
      printf("FAIL: ");    \
      printf(__VA_ARGS__); \
      printf("\n");        \
      fails++;             \
    }                      \
  } while (0)

static void reset()
{
  now_ms = 1000;
  temp = 20;
  target = 0;
  pin_level = 0;
  e_moves.clear();
  xyz_moves.clear();
  cancels = homes = heater_off = cold_extrusions = busy_depth = mode = 0;
  cancel_heatup = false;
  e_back_total = 0;
  last_e_set = -999;
  cancel_flag = short_flag = false;
  max_chunk = 0;
  wait_began = 0;
  inserted_at = 0;
  first_wait = 0;
  early_done = false;
  pulled = inserted = false;
  user_pulls_out = user_inserts = true;
  user_cancels = user_clicks_early = user_m108 = false;
  insert_after = 3000;
  click_after = 2000;
  cancel_after = 2000;
  target_at_cancel = -1;
  card.sdprinting = card.paused = false;
  filament_service(true);  // nothing prints: the sensor is not armed any more
  current_position[0] = 10;
  current_position[1] = 20;
  current_position[2] = 50;
  current_position[3] = 123.4f;
  filament_sensor_enable(true);
  filament_sensor_init();
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
}
static float sum(const std::vector<float> &v, bool positive)
{
  float s = 0;
  for (float x : v)
    if ((x > 0) == positive) s += fabsf(x);
  return s;
}

int main(int argc, char **argv)
{
  g_verbose = argc > 1;

  // 1. runout during an SD print at 215 degrees
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  filament_note_extrusion();
  bool ok = filament_change(false);
  float back = sum(e_moves, false), fwd = sum(e_moves, true);
  printf("1. M600 while printing: ok %d, E moves %zu (back %.0f, forward %.0f), biggest piece %.0f mm, %zu XYZ moves, took %.0f s\n", ok, e_moves.size(), back, fwd, max_chunk, xyz_moves.size(), (now_ms - 1000) / 1000.0);
  CHECK(ok, "the change finished");
  CHECK(fabsf(current_position[0] - 10) < 1e-4 && fabsf(current_position[1] - 20) < 1e-4 && fabsf(current_position[2] - 50) < 1e-4, "back at the print position: %.2f %.2f %.2f", current_position[0], current_position[1], current_position[2]);
  CHECK(fabsf(last_e_set - 123.4f) < 1e-3f && fabsf(current_position[3] - 123.4f) < 1e-3f, "E counter restored (%.2f / %.2f)", current_position[3], last_e_set);
  CHECK(target == 215, "heater target restored (%.0f)", target);
  CHECK(card.sdprinting && !card.paused, "the SD print goes on");
  CHECK(cold_extrusions == 0, "no extrusion with a cold nozzle");
  CHECK(max_chunk <= 100.001f, "extruder moves are cut into pieces of at most 100 mm (%.1f)", max_chunk);
  CHECK(fabsf(back - (3 + 745)) < 0.01f, "retract 3 + unload 745 = %.1f", back);
  CHECK(fabsf(fwd - (10 + 740 + 5 + 3)) < 0.01f, "load 10 + 740 + purge 5 + unretract 3 = %.1f", fwd);
  CHECK(xyz_moves.size() == 4 && xyz_moves[0][2] == 55 && xyz_moves[0][0] == 10, "first it lifts 5 mm (%.1f)", xyz_moves[0][2]);
  {
    bool parked = false;
    for (auto &m : xyz_moves)
      if (fabsf(m[0]) < 1e-4 && fabsf(m[1] + 68) < 1e-3) parked = true;
    CHECK(parked, "parked at X0 Y-68 (80%% of the printable radius 85)");
  }
  CHECK(cancels == 0 && busy_depth == 0 && mode == 0, "clean state afterwards (busy %d, mode %d)", busy_depth, mode);

  // 2. the nozzle never lifts less than 5 mm and parks at least 15% of the height up; from high up it keeps 5 mm clearance
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  current_position[2] = 100;
  filament_change(false);
  CHECK(xyz_moves[0][2] == 105, "park height from z=100: %.1f", xyz_moves[0][2]);
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  current_position[2] = 10;
  filament_change(false);
  CHECK(fabsf(xyz_moves[0][2] - 33.75f) < 1e-3f, "park height from z=10: %.1f", xyz_moves[0][2]);

  // 3. cold nozzle, hot enough only after heating: nothing is extruded cold, the target goes to 210 and back to 0
  reset();
  user_pulls_out = true;
  filament_note_extrusion();
  ok = filament_change(true);
  printf("3. button, nothing printing, filament in: ok %d, E back %.0f forward %.0f, homes %d\n", ok, sum(e_moves, false), sum(e_moves, true), homes);
  CHECK(ok && cold_extrusions == 0 && target == 0, "service change finished, heater off again (%.0f)", target);
  CHECK(fabsf(current_position[0]) < 1e-4 && fabsf(current_position[1]) < 1e-4 && fabsf(current_position[2] - 33.75f) < 1e-3, "ends above the centre: %.1f %.1f %.1f", current_position[0], current_position[1], current_position[2]);
  CHECK(fabsf(sum(e_moves, false) - 745) < 0.01f, "unloaded (no retract when it starts cold): %.1f", sum(e_moves, false));

  // 4. no filament in the extruder: load only
  reset();
  pin_level = 1;
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  user_pulls_out = false;
  CHECK(filament_absent(), "the sensor settled on 'absent'");
  ok = filament_change(true);
  printf("4. button with no filament: ok %d, E back %.0f forward %.0f\n", ok, sum(e_moves, false), sum(e_moves, true));
  CHECK(ok && sum(e_moves, false) == 0 && fabsf(sum(e_moves, true) - 755 + 3 * 0) < 0.01f, "only loads (755 mm): back %.1f forward %.1f", sum(e_moves, false), sum(e_moves, true));

  // 5. cancel while waiting
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  user_cancels = true;
  ok = filament_change(false);
  printf("5. long press while waiting: ok %d, cancels %d, heater off %d\n", ok, cancels, heater_off);
  CHECK(!ok && cancels == 1 && target == 0 && !card.sdprinting && busy_depth == 0, "cancel: print cancelled, heater off");

  // 5b. M108 from the host cancels as well
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  user_m108 = true;
  user_inserts = false;
  ok = filament_change(false);
  CHECK(!ok && cancels == 1 && target == 0, "M108 cancels the change");

  // 6. a click before the sensor sees the new filament does nothing
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  user_clicks_early = true;
  insert_after = 6000;
  ok = filament_change(false);
  printf("6. a click before the new filament is in: ok %d, took %.1f s\n", ok, (now_ms - 1000) / 1000.0);
  CHECK(ok && (now_ms - 1000) > 6000, "waited for the sensor");

  // 7. nobody comes: the heater goes off after 2 minutes, the wait goes on
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  user_inserts = false;
  click_after = 1000000000UL;
  user_cancels = true;
  cancel_after = 150000;
  ok = filament_change(false);
  printf("7. nobody comes: heater target when it was cancelled after 150 s: %.0f\n", target_at_cancel);
  CHECK(!ok && target_at_cancel == 0, "the heater was switched off by itself (%.0f)", target_at_cancel);

  // 7b. the sensor starts the change by itself, but only while something prints and after an extrusion
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  user_pulls_out = false;
  pin_level = 1;
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  filament_service(true);
  CHECK(e_moves.empty(), "not armed: nothing happens");
  filament_note_extrusion();
  filament_service(true);
  printf("7b. runout while printing: %zu E moves, SD printing again %d\n", e_moves.size(), card.sdprinting);
  CHECK(!e_moves.empty() && card.sdprinting, "the runout started a change that finished");
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = false;
  pin_level = 1;
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  filament_note_extrusion();
  filament_service(true);
  CHECK(e_moves.empty(), "nothing printing: no change");
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  filament_note_extrusion();
  filament_service(true);
  CHECK(e_moves.empty(), "filament there: no change");
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  pin_level = 1;
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  filament_note_extrusion();
  filament_sensor_enable(false);
  filament_service(true);
  CHECK(e_moves.empty(), "sensor off (M412 S0): no change");

  // 7c. a sensor that never saw filament (nothing plugged in) does not stop the print
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  pin_level = 1;
  filament_sensor_init();
  for (int i = 0; i < 3000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(filament_absent(), "the open input reads absent");
  filament_note_extrusion();
  filament_service(true);
  CHECK(e_moves.empty(), "never present: no automatic change");

  // 8. M701 / M24 / M108 are only answered while a change waits
  reset();
  CHECK(!filament_hotkey("M701") && !filament_hotkey("M24"), "no hotkey outside a change");

  // 9. the sensor: a flicker shorter than the debounce is ignored, a steady loss is seen, and so is the insertion
  reset();
  pin_level = 1;
  for (int i = 0; i < 100; i++) {
    advance(1);
    filament_sensor_poll();
  }
  pin_level = 0;
  for (int i = 0; i < 2000; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(filament_present(), "a 100 ms flicker is ignored");
  pin_level = 1;
  for (int i = 0; i < 400; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(!filament_absent(), "400 ms is not enough");
  for (int i = 0; i < 200; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(filament_absent(), "500 ms is a runout");
  pin_level = 0;
  for (int i = 0; i < 700; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(filament_absent(), "an insertion needs 750 ms");
  for (int i = 0; i < 100; i++) {
    advance(1);
    filament_sensor_poll();
  }
  CHECK(filament_present(), "filament in");
  filament_sensor_enable(false);
  CHECK(!filament_present() && !filament_absent(), "a disabled sensor says nothing");

  // 8. button pause while printing: the nozzle parks, a click brings it back, the extruder is where it was
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  current_position[2] = 20;
  card.pauseSDPrint();
  filament_pause_request();
  filament_service(false);
  CHECK(xyz_moves.empty(), "not parked while commands are still queued");
  filament_service(true);
  CHECK(xyz_moves.size() == 2 && fabsf(xyz_moves[1][0]) < 1e-4 && fabsf(xyz_moves[1][1] + 68) < 1e-3 && fabsf(xyz_moves[1][2] - 33.75f) < 1e-3, "pause: lifted, then parked at X0 Y-68 (%zu moves)", xyz_moves.size());
  CHECK(e_moves.size() == 1 && e_moves[0] < 0, "pause: retracted");
  xyz_moves.clear();
  e_moves.clear();
  ok = filament_resume();
  CHECK(ok && xyz_moves.size() == 2 && fabsf(xyz_moves[1][0] - 10) < 1e-4 && fabsf(xyz_moves[1][1] - 20) < 1e-4 && fabsf(xyz_moves[1][2] - 20) < 1e-4, "resume: back at the print position");
  CHECK(e_moves.size() == 1 && e_moves[0] > 0 && fabsf(current_position[3] - 123.4f) < 1e-3f, "resume: pushed forward again, E counter restored");
  CHECK(busy_depth == 0, "pause: clean state");

  printf(fails ? "filament: %d FAILED\n" : "filament: all ok\n", fails);
  return fails ? 1 : 0;
}
