// Filament runout sensor and filament change (M600).
//
// The change runs inside one G-code command (like M109): the machine does nothing else while it waits for you.
// The button decides: a click goes on (loads the new filament, confirms), a long press cancels.
// M701 and M24 go on, M108 cancels, they are answered at once while a change waits (see filament_hotkey()).

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "cardreader.h"
#include "language.h"
#include "ui.h"
#include "filament.h"

// ---- the sensor ------------------------------------------------------------------------------------------------------

static bool enabled = RUNOUT_ENABLED;
static bool raw_absent = false, stable_absent = false, valid = false;
static unsigned long changed_ms = 0;
static bool armed = false;            // an extrusion happened: runout is worth acting on
static bool seen_absent = false;
static bool seen_present = false;     // the sensor has shown filament since power-on: it is connected and works      // the old filament was out of the sensor at some time since the change began

static float park_y() { return -DELTA_PRINTABLE_RADIUS * FILAMENT_PARK_RADIUS_PERCENT / 100.0; }
static float min_park_z() { return max_pos[Z_AXIS] * FILAMENT_PARK_MIN_Z_PERCENT / 100.0; }

static bool read_absent() { return READ(RUNOUT_PIN) == (RUNOUT_ACTIVE_HIGH ? HIGH : LOW); }

void filament_sensor_init()
{
  SET_INPUT(RUNOUT_PIN);
  #if RUNOUT_PULLUP
    WRITE(RUNOUT_PIN, HIGH);
  #endif
  raw_absent = read_absent();
  stable_absent = raw_absent;
  valid = false;
  seen_present = false;
  changed_ms = millis();
}

void filament_sensor_poll()
{
  bool raw = read_absent();
  unsigned long now = millis();
  if (raw != raw_absent) { raw_absent = raw; changed_ms = now; }
  // running out must be noticed soon, an insertion has to be steady
  if (now - changed_ms >= (raw ? RUNOUT_DEBOUNCE_MS : FILAMENT_INSERT_DEBOUNCE_MS)) { stable_absent = raw; valid = true; }
  if (stable_absent && valid) seen_absent = true;
  if (!stable_absent && valid) seen_present = true;
}

bool filament_sensor_enabled() { return enabled; }
void filament_sensor_enable(bool on) { enabled = on; }
bool filament_present() { return enabled && valid && !stable_absent; }
bool filament_absent() { return enabled && valid && stable_absent; }
void filament_note_extrusion() { armed = true; }

void filament_report()
{
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("Runout sensor: ", (unsigned long)enabled);
  SERIAL_ECHOPAIR(" filament present: ", (unsigned long)filament_present());
  SERIAL_ECHOPAIR(" absent: ", (unsigned long)filament_absent());
  SERIAL_ECHOPAIR(" pin: ", (unsigned long)raw_absent);
  SERIAL_ECHOLN("");
}

// ---- the change ------------------------------------------------------------------------------------------------------

static bool changing = false;       // a change is running
static bool waiting = false;        // ... and waits for the user
static bool failed = false;
static unsigned long last_report = 0;

bool filament_waiting() { return changing && waiting; }

bool filament_hotkey(const char *line)
{
  if (!filament_waiting()) return false;
  const char *m = strchr(line, 'M');
  if (m == NULL) return false;
  int code = atoi(m + 1);
  if (code == 24 || code == 701) ui_virtual_short();
  else return false;
  SERIAL_PROTOCOLLNPGM(MSG_OK);
  return true;
}

static bool stop_requested() { return cancel_heatup || ui_cancel_requested() || IsStopped(); }

static void tick(const char *what_P)
{
  manage_heater();
  manage_inactivity();
  unsigned long now = millis();
  if (now - last_report >= 2000) {   // keeps a host from thinking the printer died
    last_report = now;
    SERIAL_PROTOCOLPGM("T:"); SERIAL_PROTOCOL_F(degHotend(0), 1);
    SERIAL_PROTOCOLPGM(" /"); SERIAL_PROTOCOL_F(degTargetHotend(0), 1);
    SERIAL_PROTOCOLPGM(" echo:busy: "); serialprintPGM(what_P);
    SERIAL_PROTOCOLLN("");
  }
}

static void fail(const char *why)
{
  if (failed) return;
  failed = true;
  SERIAL_ERROR_START;
  SERIAL_ERRORPGM("FILAMENT ");
  SERIAL_ERRORLN(why);
}

static bool hot_enough() { return degHotend(0) >= FILAMENT_MIN_EXTRUDE_TEMP_C; }

// Heat to target and hold it. False if stopped, or it took too long.
static bool heat_to(float target)
{
  setTargetHotend(target, 0);
  unsigned long start = millis(), stable = 0;
  for (;;) {
    tick(PSTR("heating"));
    if (stop_requested()) return false;
    if (millis() - start > FILAMENT_HEAT_TIMEOUT_MS) { fail("HEAT_TIMEOUT"); return false; }
    float t = degHotend(0);
    if (t >= FILAMENT_MIN_EXTRUDE_TEMP_C && fabs(t - target) <= FILAMENT_TEMP_TOLERANCE_C) {
      if (!stable) stable = millis();
      if (millis() - stable >= FILAMENT_TEMP_HOLD_MS) return true;
    } else {
      stable = 0;
    }
  }
}

// An extruder move of mm (negative: back), in pieces the planner accepts.
static bool e_move(float mm, float feed)
{
  while (fabs(mm) > 0.0001) {
    float step = mm > 0 ? min(mm, (float)FILAMENT_E_CHUNK_MM) : max(mm, -(float)FILAMENT_E_CHUNK_MM);
    if (!hot_enough()) { fail("TOO_COLD"); return false; }
    move_e(step, feed);
    st_synchronize();
    if (stop_requested()) return false;
    mm -= step;
  }
  return true;
}

// Wait for a click, a M701 or a M24. The heater goes off if it takes too long.
static bool wait_for_user(const char *what_P)
{
  waiting = true;
  ui_filament_mode(2);
  unsigned long start = millis();
  bool off = false;
  ui_take_short();
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("FILAMENT waiting: ");
  serialprintPGM(what_P);
  SERIAL_ECHOLN("");
  for (;;) {
    tick(what_P);
    if (stop_requested()) break;
    if (!off && millis() - start >= FILAMENT_IDLE_TIMEOUT_MS) {
      off = true;
      setTargetHotend(0, 0);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("FILAMENT heater off, it is heated again before the filament moves");
    }
    if (ui_take_short()) { waiting = false; ui_filament_mode(1); return true; }
  }
  waiting = false;
  ui_filament_mode(1);
  return false;
}

// ---- pause: park the nozzle away from the part, come back on a click ---------------------------------------------------

static bool pause_pending = false;   // the print was paused, the nozzle parks when the queued commands have run
static bool parked = false;
static float pk_x, pk_y, pk_z, pk_e, pk_park_z, pk_target;
static bool pk_retracted;

static bool homed() { return axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS]; }

void filament_pause_request() { pause_pending = true; }
void filament_pause_reset() { pause_pending = false; parked = false; }

static void park_now()
{
  pause_pending = false;
  if (parked || changing || IsStopped() || !homed()) return;
  st_synchronize();
  pk_x = current_position[X_AXIS]; pk_y = current_position[Y_AXIS]; pk_z = current_position[Z_AXIS]; pk_e = current_position[E_AXIS];
  pk_target = degTargetHotend(0);
  pk_park_z = pk_z + FILAMENT_Z_LIFT;
  if (pk_park_z < min_park_z()) pk_park_z = min_park_z();
  if (pk_park_z > max_pos[Z_AXIS]) pk_park_z = max_pos[Z_AXIS];
  parked = true;
  ui_busy(true);
  pk_retracted = false;
  if (hot_enough()) pk_retracted = e_move(-FILAMENT_RETRACT_MM, FILAMENT_RETRACT_SPEED);
  move_to(pk_x, pk_y, pk_park_z, FILAMENT_Z_SPEED); st_synchronize();
  move_to(0, park_y(), pk_park_z, FILAMENT_TRAVEL_SPEED); st_synchronize();
  ui_busy(false);
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("PRINT paused, nozzle parked: click (or M24) to go on, double click to change the filament, hold to cancel");
}

// Back to the print position. True when the print may go on.
bool filament_resume()
{
  if (pause_pending) { pause_pending = false; return true; }   // it had not left yet
  if (!parked) return true;
  ui_busy(true);
  bool ok = !(pk_target >= FILAMENT_MIN_EXTRUDE_TEMP_C) || heat_to(pk_target);
  if (ok) { move_to(pk_x, pk_y, pk_park_z, FILAMENT_TRAVEL_SPEED); st_synchronize(); ok = !stop_requested(); }
  if (ok) { move_to(pk_x, pk_y, pk_z, FILAMENT_Z_SPEED); st_synchronize(); ok = !stop_requested(); }
  if (ok && pk_retracted) ok = e_move(FILAMENT_RETRACT_MM, FILAMENT_RETRACT_SPEED);
  ui_busy(false);
  if (ok) {
    parked = false;
    current_position[E_AXIS] = pk_e;
    plan_set_e_position(pk_e);
  }
  return ok;
}

bool filament_change(bool service)
{
  if (pause_pending) park_now();
  if (changing || IsStopped()) return false;
  if (!homed()) {
    if (!service) { SERIAL_ERROR_START; SERIAL_ERRORLNPGM("M600: home first (G28)"); return false; }
    home_all_axes();
    if (!homed()) return false;
  }
  st_synchronize();

  changing = true; failed = false; cancel_heatup = false; waiting = false;
  ui_busy(true);
  ui_filament_mode(1);
  last_report = millis();
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("FILAMENT change started");

  if (service) {   // wait above the centre, away from anything on the bed
    move_to(0, 0, min_park_z(), FILAMENT_Z_SPEED);
    st_synchronize();
  }

  const bool was_parked = parked;   // paused by the button: the print position and the retraction were saved then
  const float x = was_parked ? pk_x : current_position[X_AXIS], y = was_parked ? pk_y : current_position[Y_AXIS];
  const float z = was_parked ? pk_z : current_position[Z_AXIS], e = was_parked ? pk_e : current_position[E_AXIS];
  const float saved_target = degTargetHotend(0);
  const float change_target = saved_target > FILAMENT_CHANGE_TEMP_C ? saved_target : FILAMENT_CHANGE_TEMP_C;
  bool was_sd = false;
  was_sd = card.sdprinting;
  if (was_sd) card.pauseSDPrint();
  const bool load_only = service && filament_absent();   // nothing to unload
  seen_absent = filament_absent() || load_only;
  bool retracted = was_parked && pk_retracted, ok = true;
  float park_z = z + FILAMENT_Z_LIFT;
  if (park_z < min_park_z()) park_z = min_park_z();
  if (park_z > max_pos[Z_AXIS]) park_z = max_pos[Z_AXIS];

  // pull back, lift, park
  if (ok && !was_parked && !load_only && hot_enough()) { ok = e_move(-FILAMENT_RETRACT_MM, FILAMENT_RETRACT_SPEED); retracted = ok; }
  if (ok && !was_parked) { move_to(x, y, park_z, FILAMENT_Z_SPEED); st_synchronize(); ok = !stop_requested(); }
  if (ok) { move_to(0, park_y(), park_z, FILAMENT_TRAVEL_SPEED); st_synchronize(); ok = !stop_requested(); }

  // unload
  if (ok && !load_only) ok = heat_to(change_target) && e_move(-(BOWDEN_LENGTH_MM + FILAMENT_UNLOAD_EXTRA_MM), FILAMENT_UNLOAD_SPEED);

  // wait for the new filament
  while (ok) {
    if (!wait_for_user(PSTR("insert the new filament, then click the button (or M701)"))) { ok = false; break; }
    if (!enabled || (filament_present() && seen_absent)) break;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("FILAMENT the sensor does not see new filament (the old one must be pulled out first)");
  }

  // load and purge
  if (ok) ok = heat_to(change_target);
  if (ok) ok = e_move(FILAMENT_SLOW_LOAD_MM, FILAMENT_SLOW_LOAD_SPEED);
  if (ok && enabled && !filament_present()) { fail("FILAMENT_LOST_DURING_LOAD"); ok = false; }
  if (ok) ok = e_move(BOWDEN_LENGTH_MM, FILAMENT_FAST_LOAD_SPEED);
  if (ok) ok = e_move(FILAMENT_PURGE_MM, FILAMENT_PURGE_SPEED);

  // confirm
  while (ok) {
    if (!wait_for_user(PSTR("click the button (or M24) to go on, hold it to cancel"))) { ok = false; break; }
    if (!enabled || filament_present()) break;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("FILAMENT filament required");
  }

  // go back
  if (ok) {
    float target = saved_target >= FILAMENT_MIN_EXTRUDE_TEMP_C ? saved_target : change_target;
    ok = heat_to(target);
  }
  if (ok) { move_to(x, y, park_z, FILAMENT_TRAVEL_SPEED); st_synchronize(); ok = !stop_requested(); }
  if (ok) { move_to(x, y, z, FILAMENT_Z_SPEED); st_synchronize(); ok = !stop_requested(); }
  if (ok && retracted) ok = e_move(FILAMENT_RETRACT_MM, FILAMENT_RETRACT_SPEED);

  changing = false;
  waiting = false;
  parked = false;
  ui_filament_mode(0);
  if (ok) {
    current_position[E_AXIS] = e;       // the new filament is primed to the nozzle: what the slicer counted stays valid
    plan_set_e_position(e);
    setTargetHotend(saved_target, 0);
    if (was_sd) card.startFileprint();
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("FILAMENT change done");
  } else {
    if (!failed) { SERIAL_ECHO_START; SERIAL_ECHOLNPGM("FILAMENT change cancelled"); }
    disable_heater();
    ui_busy(false);
    if (was_sd || card.isPaused()) ui_defer_cancel();   // the print cannot go on (not here: this runs inside a command)
    return false;
  }
  ui_busy(false);
  return true;
}

// Filament ran out while printing: change it by itself.
void filament_service(bool no_commands_queued)
{
  if (changing) return;
  if (pause_pending && no_commands_queued) park_now();
  bool printing = card.sdprinting;
  if (!printing && !blocks_queued() && machine_idle()) { armed = false; return; }   // nothing is printing any more
  // a sensor that never showed filament is not plugged in (an open input reads "no filament"): it must not stop a print
  if (!enabled || !armed || !seen_present || !filament_absent() || IsStopped()) return;
  if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) return;
  armed = false;
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("FILAMENT_RUNOUT");
  if (!filament_change(false)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("FILAMENT runout: the change failed");
  }
}
