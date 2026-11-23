// FSR Z probe and G33 delta calibration.
//
// The nozzle is the probe: a force sensing resistor under the bed reports contact. A probe stroke is one
// vertical move of all three towers; the stepper interrupt stops it as soon as the FSR reading moves by
// fsr_sensitivity counts from its value when the stroke started (see st_probe_arm()).

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "ConfigurationStore.h"
#include "delta_calibration.h"
#include "ui.h"
#include "probe.h"

unsigned int fsr_sensitivity = FSR_SENSITIVITY;
float probe_z_offset = 0.0;
bool delta_calibrated = false;

static uint8_t false_triggers = 0;   // spurious contacts in a row
static float deepest_contact = 0.0;  // lowest accepted contact height since the session began
static bool have_deepest = false;
static float bed_top = 0.0;          // highest contact found in the pass: the moves between points clear it
static bool have_top = false;

bool probe_set_sensitivity(unsigned int counts)
{
  if (counts < 1 || counts > 1023) return false;
  fsr_sensitivity = counts;
  return true;
}

static void wait_ms(unsigned long ms)
{
  unsigned long t = millis();
  while (millis() - t < ms) { manage_heater(); manage_inactivity(); }
}

// One vertical move of the nozzle at (x, y). With probing on, the move ends as soon as the probe fires.
// z_end is where the nozzle really is afterwards, the planner and current_position follow it.
static bool stroke(float x, float y, float z_from, float z_to, float feed, bool probing, float &z_end)
{
  long p0[3];
  float cart[3] = { x, y, z_from };
  for (int a = 0; a < 3; a++) p0[a] = st_get_position(a);
  cart[Z_AXIS] = z_to;
  calculate_delta(cart);
  if (probing) st_probe_arm(fsr_sensitivity);
  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], feed, active_extruder, fabs(z_to - z_from));
  st_synchronize();
  keep_steppers_on();
  bool hit = probing && st_probe_hit();
  st_probe_disarm();
  if (hit) {
    float moved = 0;
    for (int a = 0; a < 3; a++) moved += (st_get_position(a) - p0[a]) / axis_steps_per_unit[a];
    z_end = z_from + moved / 3.0;
  } else {
    z_end = z_to;
  }
  cart[Z_AXIS] = z_end;
  current_position[X_AXIS] = x; current_position[Y_AXIS] = y; current_position[Z_AXIS] = z_end;
  calculate_delta(cart);
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  return hit;
}

// The probe is quiet when idle; a noisy one gets a higher threshold.
static void adapt_to_noise()
{
  unsigned int lo = 1023, hi = 0;
  unsigned long t = millis();
  while (millis() - t < 150) {
    CRITICAL_SECTION_START;
    unsigned int v = fsr_raw;   // 16 bit: read it with interrupts off
    CRITICAL_SECTION_END;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
    manage_heater(); manage_inactivity();
  }
  unsigned int noise = hi - lo, cap = 8 * FSR_SENSITIVITY;
  if (cap < fsr_sensitivity) cap = fsr_sensitivity;
  if (noise >= fsr_sensitivity) {
    unsigned int s = noise + noise / 2 + 2;
    if (s > cap) s = cap;
    if (s > fsr_sensitivity) {
      fsr_sensitivity = s;
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("FSR is noisy, sensitivity raised to ", (unsigned long)fsr_sensitivity);
      SERIAL_ECHOLNPGM(" counts");
    }
  }
}

// Heights are only comparable within one pass: G33 re-homes with new settings between passes.
static void new_pass()
{
  false_triggers = 0;
  have_deepest = false;
  have_top = false;
}

static void begin_session()
{
  new_pass();
  adapt_to_noise();
}

// Two spurious contacts in a row: the threshold is too low for this sensor.
static void note_false_trigger()
{
  if (++false_triggers < 2) return;
  false_triggers = 0;
  unsigned int s = fsr_sensitivity + fsr_sensitivity / 2 + 1, cap = 8 * FSR_SENSITIVITY;
  if (cap < fsr_sensitivity) cap = fsr_sensitivity;
  if (s > cap) s = cap;
  if (s > fsr_sensitivity) {
    fsr_sensitivity = s;
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("spurious probe triggers, FSR sensitivity raised to ", (unsigned long)fsr_sensitivity);
    SERIAL_ECHOLNPGM(" counts");
  }
}

// Probe the bed at (x, y) and return its height, the average of strokes that agree.
// bed_guess: where the bed is expected, depth: how far below it to look.
static bool probe_point(float x, float y, float bed_guess, float depth, float &result)
{
  float z = current_position[Z_AXIS], zz;
  float samples[PROBE_AVERAGE_MAX_STROKES];
  uint8_t n = 0, attempts = 0;
  float start;

  // up, across, down to where the slow approach begins
  start = bed_guess + PROBE_SLOW_APPROACH_MM;
  float travel = have_top ? bed_top + PROBE_CLEARANCE_MM : PROBE_TRAVEL_Z_MM;
  if (travel < start + 1.0) travel = start + 1.0;
  if (z < travel) { stroke(current_position[X_AXIS], current_position[Y_AXIS], z, travel, PROBE_TRAVEL_Z_FEED_MM_S, false, zz); }
  move_to(x, y, travel, PROBE_TRAVEL_FEED_MM_S);
  st_synchronize();
  if (current_position[Z_AXIS] > start) {
    wait_ms(PROBE_SETTLE_MS);
    if (stroke(x, y, current_position[Z_AXIS], start, PROBE_FEED_FAST_MM_S, true, zz)) {
      // Contact before the slow approach: a bump of the fast move, or a bed higher than expected. Either way, lift
      // and approach slowly from above that point; the depth limit stays relative to where the bed was expected.
      start = zz + PROBE_SLOW_APPROACH_MM + 1.0;
      stroke(x, y, zz, start, PROBE_RETRACT_FEED_MM_S, false, zz);
    }
  }

  while (n < PROBE_AVERAGE_MAX_STROKES) {
    if (ui_cancel_requested() || IsStopped()) return false;
    if (attempts++ >= 2 * PROBE_AVERAGE_MAX_STROKES) return false;
    wait_ms(PROBE_SETTLE_MS);
    float from = current_position[Z_AXIS];
    if (stroke(x, y, from, bed_guess - depth, PROBE_FEED_MM_S, true, zz)) {
      if ((from - zz < PROBE_MIN_STROKE_MM) || (have_deepest && zz > deepest_contact + PROBE_FALSE_TRIGGER_MM)) {
        note_false_trigger();            // right at the start of the move, or in mid-air: vibration or noise, not the bed
        SERIAL_ECHO_START;
        SERIAL_ECHOPAIR("ignored a contact at Z", zz);
        SERIAL_ECHOPAIR(", ", from - zz);
        SERIAL_ECHOLNPGM(" mm after the stroke started");
        stroke(x, y, zz, zz + 1.0, PROBE_RETRACT_FEED_MM_S, false, zz);
        continue;
      } else {
        false_triggers = 0;
        if (!have_deepest || zz < deepest_contact) { deepest_contact = zz; have_deepest = true; }
        samples[n++] = zz;
        if (n >= PROBE_AVERAGE_SAMPLES) {  // the last strokes agree?
          float lo = samples[n - 1], hi = lo;
          for (uint8_t i = n - PROBE_AVERAGE_SAMPLES; i < n; i++) { if (samples[i] < lo) lo = samples[i]; if (samples[i] > hi) hi = samples[i]; }
          if (hi - lo <= PROBE_AVERAGE_TOLERANCE_MM) break;
        }
      }
      stroke(x, y, zz, zz + 1.5, PROBE_RETRACT_FEED_MM_S, false, zz);   // lift a little, probe again
    } else {
      // no contact within the depth limit
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM("probe: no contact");
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("the nozzle went down to Z", zz);
      SERIAL_ECHOPAIR(", the FSR changed by at most ", (unsigned long)st_probe_peak());
      SERIAL_ECHOPAIR(" counts, it needs ", (unsigned long)fsr_sensitivity);
      SERIAL_ECHOLN("");
      return false;
    }
  }
  if (n == 0) return false;
  uint8_t used = n < PROBE_AVERAGE_SAMPLES ? n : PROBE_AVERAGE_SAMPLES;
  float sum = 0;
  for (uint8_t i = n - used; i < n; i++) sum += samples[i];
  result = sum / used;
  if (!have_top || result > bed_top) { bed_top = result; have_top = true; }   // the moves to the next points clear this
  if (n > used) {
    SERIAL_ECHO_START;
    if (n >= PROBE_AVERAGE_MAX_STROKES) { SERIAL_ECHOPGM("probe strokes did not agree, using the last ones:"); }
    else { SERIAL_ECHOPGM("probe strokes before the last ones were off:"); }
    for (uint8_t i = 0; i < n; i++) { SERIAL_ECHOPGM(" "); SERIAL_PROTOCOL_F(samples[i], 2); }
    SERIAL_ECHOLN("");
  }
  return true;
}

static void lift_to_travel()
{
  float z;
  if (current_position[Z_AXIS] < PROBE_TRAVEL_Z_MM)
    stroke(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], PROBE_TRAVEL_Z_MM, PROBE_TRAVEL_Z_FEED_MM_S, false, z);
}

static bool ready_to_probe()
{
  if (IsStopped()) return false;
  if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) home_all_axes();
  if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) return false;
  return true;
}

void probe_g30()
{
  if (!ready_to_probe()) return;
  ui_busy(true);
  begin_session();
  float x = current_position[X_AXIS], y = current_position[Y_AXIS], z;
  if (probe_point(x, y, 0.0, CALIB_FIRST_PASS_DEPTH_MM, z)) {
    SERIAL_PROTOCOLPGM("Bed X: "); SERIAL_PROTOCOL_F(x, 3);
    SERIAL_PROTOCOLPGM(" Y: "); SERIAL_PROTOCOL_F(y, 3);
    SERIAL_PROTOCOLPGM(" Z: "); SERIAL_PROTOCOL_F(z - probe_z_offset, 3);
    SERIAL_PROTOCOLLN("");
  } else {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("probe: no contact");
  }
  lift_to_travel();
  ui_busy(false);
}

// ---- G33 ----------------------------------------------------------------------------------------------------------

struct Geometry3 {
  float R, L, ang[3], adj[3], home_z;
};

static void save_geometry(Geometry3 &g)
{
  g.R = delta_radius; g.L = delta_diagonal_rod;
  for (int a = 0; a < 3; a++) { g.ang[a] = delta_tower_angle[a]; g.adj[a] = endstop_adj[a]; }
  g.home_z = add_homing[Z_AXIS];
}

static void restore_geometry(const Geometry3 &g)
{
  delta_radius = g.R; delta_diagonal_rod = g.L;
  for (int a = 0; a < 3; a++) { delta_tower_angle[a] = g.ang[a]; endstop_adj[a] = g.adj[a]; }
  add_homing[Z_AXIS] = g.home_z;
  recalc_delta_settings(delta_radius, delta_diagonal_rod);
}

static void report_geometry()
{
  SERIAL_ECHO_START;
  SERIAL_ECHOPAIR("G33 R", delta_radius);
  SERIAL_ECHOPAIR(" L", delta_diagonal_rod);
  SERIAL_ECHOPAIR(" angles ", delta_tower_angle[0]);
  SERIAL_ECHOPAIR(" ", delta_tower_angle[1]);
  SERIAL_ECHOPAIR(" ", delta_tower_angle[2]);
  SERIAL_ECHOPAIR(" endstops ", endstop_adj[0]);
  SERIAL_ECHOPAIR(" ", endstop_adj[1]);
  SERIAL_ECHOPAIR(" ", endstop_adj[2]);
  SERIAL_ECHOPAIR(" home Z ", add_homing[Z_AXIS]);
  SERIAL_ECHOLN("");
}

// Make the solved geometry the machine's. out.off[] says how far each carriage really sits above where the
// firmware believed it: the endstop retraction (which can only lower a carriage) and the home height absorb it.
static bool apply_solution(const DeltaGeometry &out, const Geometry3 &start)
{
  float v[3], m = -1e9, adj[3];
  float H0 = sqrt(sq(delta_diagonal_rod) - sq(delta_radius));
  float H1 = sqrt(sq(out.L) - sq(out.R));
  for (int a = 0; a < 3; a++) { v[a] = out.off[a]; if (endstop_adj[a] - v[a] > m) m = endstop_adj[a] - v[a]; }
  for (int a = 0; a < 3; a++) adj[a] = endstop_adj[a] - v[a] - m;
  float home_z = add_homing[Z_AXIS] - m + (H0 - H1);

  bool ok = fabs(out.R - start.R) <= CALIB_MAX_RADIUS_DELTA_MM &&
            fabs(home_z - start.home_z) <= CALIB_MAX_HEIGHT_DELTA_MM;
  for (int a = 0; a < 3; a++) ok = ok && adj[a] >= -CALIB_MAX_TRIM_MM && fabs(out.ang[a]) <= CALIB_MAX_ANGLE_DEG;
  if (!ok) return false;

  delta_radius = out.R; delta_diagonal_rod = out.L;
  for (int a = 0; a < 3; a++) { delta_tower_angle[a] = out.ang[a]; endstop_adj[a] = adj[a]; }
  add_homing[Z_AXIS] = home_z;
  recalc_delta_settings(delta_radius, delta_diagonal_rod);
  return true;
}

void probe_g33(uint8_t factors, uint8_t points, bool save)
{
  if (!ready_to_probe()) return;
  ui_busy(true);
  begin_session();
  Geometry3 start;
  save_geometry(start);
  SERIAL_ECHO_START; SERIAL_ECHOPGM("G33 start: "); SERIAL_ECHOLN("");
  report_geometry();

  const float probe_radius = CALIB_PROBE_RADIUS_MM;
  float prev_rms = 1e9, rms = 0;
  uint8_t verdict = 0;  // 1 calibrated, 2 accepted (noise limited), 3 uncertified, 4 failed, 5 cancelled
  bool changed = false;

  for (uint8_t pass = 1; pass <= CALIB_MAX_PASSES; pass++) {
    float err[DCAL_MAX_POINTS], rms_after;
    new_pass();
    DeltaGeometry g0;
    g0.R = delta_radius; g0.L = delta_diagonal_rod;
    for (int a = 0; a < 3; a++) { g0.ang[a] = delta_tower_angle[a]; g0.off[a] = 0; }

    bool probed = true;
    uint8_t failed_point = 0;
    float last_z = 0;   // the next point is expected at the height of the previous one
    for (uint8_t i = 0; i < points && probed; i++) {
      float xy[2], z;
      dcal_point_xy(i, points, probe_radius, xy);
      if (i == 0) probed = probe_point(xy[0], xy[1], 0.0, pass == 1 ? CALIB_FIRST_PASS_DEPTH_MM : CALIB_PASS_DEPTH_MM, z);
      else probed = probe_point(xy[0], xy[1], last_z, CALIB_POINT_DEPTH_MM, z);
      if (!probed) failed_point = i;
      if (probed) {
        last_z = z;
        err[i] = -(z - probe_z_offset);
        SERIAL_ECHO_START;
        SERIAL_ECHOPAIR("G33 pass ", (unsigned long)pass);
        SERIAL_ECHOPAIR(" point ", (unsigned long)i);
        SERIAL_ECHOPAIR(" X", xy[0]); SERIAL_ECHOPAIR(" Y", xy[1]); SERIAL_ECHOPAIR(" Z", z - probe_z_offset);
        SERIAL_ECHOLN("");
      }
    }
    if (!probed) {
      verdict = (ui_cancel_requested() || IsStopped()) ? 5 : 4;
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("G33 pass ", (unsigned long)pass);
      SERIAL_ECHOPAIR(" stopped at point ", (unsigned long)failed_point);
      SERIAL_ECHOLN("");
      break;
    }

    DeltaGeometry out;
    if (!dcal_solve(g0, points, probe_radius, err, factors, out, &rms, &rms_after)) { verdict = 4; break; }
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("G33 pass ", (unsigned long)pass);
    SERIAL_ECHOPAIR(" rms ", rms);
    SERIAL_ECHOLNPGM(" mm");

    if (rms <= CALIB_VERIFY_RMS_MM) { verdict = 1; break; }
    if ((pass > 1 && rms >= CALIB_PLATEAU_RATIO * prev_rms) || pass == CALIB_MAX_PASSES) {
      verdict = rms <= CALIB_ACCEPT_RMS_MM ? 2 : 3;
      break;
    }
    if (!apply_solution(out, start)) { verdict = 4; break; }
    changed = true;
    prev_rms = rms;
    home_all_axes();
    if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) { verdict = 4; break; }
  }

  if (verdict >= 4) {
    if (changed) {
      restore_geometry(start);
      home_all_axes();   // the carriages were homed with the geometry that is now gone
    }
    SERIAL_ERROR_START;
    if (verdict == 5) SERIAL_ERRORLNPGM("G33 cancelled, the previous calibration is kept");
    else SERIAL_ERRORLNPGM("G33 failed (no contact, or a result outside the allowed range), the previous calibration is kept");
  } else {
    report_geometry();
    SERIAL_ECHO_START;
    if (verdict == 3) {
      SERIAL_ECHOPAIR("G33 UNCERTIFIED rms ", rms);
      SERIAL_ECHOLNPGM(" mm: not saved, run G33 again or M502 to go back");
    } else {
      delta_calibrated = true;
      if (verdict == 1) { SERIAL_ECHOPAIR("G33 OK calibrated rms ", rms); }
      else { SERIAL_ECHOPAIR("G33 OK (noise limited) rms ", rms); }
      SERIAL_ECHOLNPGM(" mm");
      if (save) Config_StoreSettings();
    }
    if (changed && verdict == 3) delta_calibrated = false;
  }
  lift_to_travel();
  ui_busy(false);
}
