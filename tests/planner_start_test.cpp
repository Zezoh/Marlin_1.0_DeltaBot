#include <vector>
#include <algorithm>
#include "Marlin.h"
#include "planner.h"
int extrudemultiply = 100; float volumetric_multiplier[1] = {1}; int fanSpeed = 0; uint8_t active_extruder = 0;
void manage_inactivity(bool) {} void manage_heater() {} float degHotend(uint8_t) { return 200; } float degTargetHotend(uint8_t) { return 200; }
void setTargetHotend0(float) {} void st_wake_up() {} void st_set_position(const long &, const long &, const long &, const long &) {} void st_set_e_position(const long &) {}
static float tx[3], ty[3]; static float L = 210.0f, R = 90.0f, dl[3];
static void calc(const float c[3]) { for (int i = 0; i < 3; i++) dl[i] = sqrt(L*L - (tx[i]-c[0])*(tx[i]-c[0]) - (ty[i]-c[1])*(ty[i]-c[1])) + c[2]; }
// A travel move from rest: the speed must rise from the first block on, with no dip after it
// (the first block used to brake to the safe speed because the planner did not join it to the second one).
int main(int argc, char **argv) {
  float acc = argc > 1 ? atof(argv[1]) : 600, jerk = argc > 2 ? atof(argv[2]) : 12, feed = argc > 3 ? atof(argv[3]) : 150, segps = argc > 4 ? atof(argv[4]) : 40, prod_ms = argc > 5 ? atof(argv[5]) : 6;
  const float S60 = 0.8660254f; tx[0] = -S60*R; ty[0] = -0.5f*R; tx[1] = S60*R; ty[1] = -0.5f*R; tx[2] = 0; ty[2] = R;
  for (int i = 0; i < 4; i++) { axis_steps_per_unit[i] = 80; max_feedrate[i] = 180; max_acceleration_units_per_sq_second[i] = 8000; }
  acceleration = acc; retract_acceleration = acc; max_xy_jerk = jerk; max_z_jerk = jerk; max_e_jerk = 5;
  reset_acceleration_rates(); plan_init();
  float a[3] = {0, 75, 5.1f}, b[3] = {65, 37, 5.1f};
  calc(a); plan_set_position(dl[0], dl[1], dl[2], 0);
  double dist = sqrt(pow(b[0]-a[0],2)+pow(b[1]-a[1],2)+pow(b[2]-a[2],2));
  float seconds = dist / feed; long steps = max(1L, min(long(segps * seconds), long(dist / 1.0)));
  // time stepping, 0.1 ms
  double t = 0; long added = 0; double next_add = 0; block_t *cur = NULL; double pos_ev = 0, rate = 0; 
  std::vector<double> win; double acc_steps = 0; long wi = 0; double wend = 10;
  double dt = 0.1; 
  double vmax_report = 0; long total = 0;
  while (t < 4000) {
    if (added < steps && t >= next_add && movesplanned() < BLOCK_BUFFER_SIZE - 1) {
      added++; float f = float(added)/steps; float p[3] = {a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f};
      calc(p); plan_buffer_line(dl[0], dl[1], dl[2], 0, feed, 0, dist/steps); next_add = t + prod_ms;
    }
    plan_hold = (added < steps);
    if (!cur) { cur = plan_get_current_block(); if (cur) { pos_ev = 0; rate = cur->initial_rate; } else if (added >= steps) break; }
    if (cur) {
      // trapezoid generator like stepper.cpp: rate follows the block's plan
      double r;
      if (pos_ev < cur->accelerate_until) { rate += cur->acceleration_st * dt/1000.0; r = rate; if (r > cur->nominal_rate) r = cur->nominal_rate; }
      else if (pos_ev >= cur->decelerate_after) { rate -= cur->acceleration_st * dt/1000.0; if (rate < cur->final_rate) rate = cur->final_rate; r = rate; }
      else { r = cur->nominal_rate; rate = r; }
      if (r < 1) r = 1;
      double ev = r * dt/1000.0; pos_ev += ev; acc_steps += ev * 1.0;
      if (pos_ev >= cur->step_event_count) { cur = NULL; plan_discard_current_block(); }
    }
    t += dt;
    if (t >= wend) { win.push_back(acc_steps); acc_steps = 0; wend += 10; }
  }
  printf("acc %.0f jerk %.0f feed %.0f seg/s %.0f -> %ld segments of %.2f mm, produced every %.0f ms\nstep events per 10 ms (largest axis count): ", acc, jerk, feed, segps, steps, dist/steps, prod_ms);
  for (size_t i = 0; i < win.size(); i++) printf("%.0f ", win[i]); printf("\n");
}
