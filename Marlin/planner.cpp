/*
  planner.cpp - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 Reasoning behind the mathematics in this module (s == speed, a == acceleration, t == time, d == distance):

 Distance to reach a specific speed m from speed s with constant acceleration:
   d = (m^2 - s^2)/(2 a)                      --> estimate_acceleration_distance()

 When to start braking (di) to reach a destination speed s2 after accelerating from speed s1
 over a total distance d without a plateau:
   di = (2 a d - s1^2 + s2^2)/(4 a)           --> intersection_distance()
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"

//===========================================================================
//============================= public variables ============================
//===========================================================================

float max_feedrate[NUM_AXIS];                                 // set the max speeds
float axis_steps_per_unit[NUM_AXIS];
unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
float minimumfeedrate;
float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
float retract_acceleration; // mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
float max_xy_jerk;          // speed that can be stopped at once
float max_z_jerk;
float max_e_jerk;
float mintravelfeedrate;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#ifdef ENABLE_AUTO_BED_LEVELING
// this holds the required transform to compensate for bed level
matrix_3x3 plan_bed_level_matrix = {
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0,
};
#endif // ENABLE_AUTO_BED_LEVELING

// The current position of the tool in absolute steps
long position[NUM_AXIS];  // rescaled from extern when axis_steps_per_unit are changed by gcode
static float previous_speed[NUM_AXIS];  // Speed of previous path line segment
static float previous_nominal_speed;    // Nominal speed of previous path line segment

#ifdef AUTOTEMP
float autotemp_max = 250;
float autotemp_min = 210;
float autotemp_factor = 0.1;
bool autotemp_enabled = false;
#endif

//===========================================================================
//================= semi-private variables, used in inline functions ========
//===========================================================================
block_t block_buffer[BLOCK_BUFFER_SIZE];   // A ring buffer for motion instructions
volatile unsigned char block_buffer_head;  // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;  // Index of the block to process now

//===========================================================================
//============================= private variables ===========================
//===========================================================================
#ifdef PREVENT_DANGEROUS_EXTRUDE
float extrude_min_temp = EXTRUDE_MINTEMP;
#endif

// Number of moves of each extruder since it was last used, to disable inactive extruders
static unsigned char extruder_last_move[2] = {0, 0};

// Index of the next / previous block in the ring buffer (BLOCK_BUFFER_SIZE is a power of 2)
#define NEXT_BLOCK_INDEX(i) (((i) + 1) & (BLOCK_BUFFER_SIZE - 1))
#define PREV_BLOCK_INDEX(i) (((i) - 1) & (BLOCK_BUFFER_SIZE - 1))

//===========================================================================
//============================= functions ===================================
//===========================================================================

// Distance (not time) it takes to accelerate from initial_rate to target_rate
FORCE_INLINE float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration == 0) return 0.0;
  return (target_rate*target_rate - initial_rate*initial_rate) / (2.0*acceleration);
}

// The point at which you must start braking (at the rate of -acceleration) if you started at speed
// initial_rate and accelerated until this point and want to end at final_rate after a total travel of
// distance. Used for blocks without a plateau.
FORCE_INLINE float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance)
{
  if (acceleration == 0) return 0.0;
  return (2.0*acceleration*distance - initial_rate*initial_rate + final_rate*final_rate) / (4.0*acceleration);
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
static void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor)
{
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor);  // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor);     // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if (initial_rate < 120) initial_rate = 120;
  if (final_rate < 120) final_rate = 120;

  long acceleration = block->acceleration_st;
  int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

  // Size of the plateau of nominal rate
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // No plateau (no cruising): find where to abort acceleration and start braking in order
  // to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0);  // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);
    plateau_steps = 0;
  }

  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (block->busy == false) {  // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}

// Maximum allowable speed at this point when you must be able to reach target_velocity
// using the acceleration within the allotted distance.
FORCE_INLINE float max_allowable_speed(float acceleration, float target_velocity, float distance)
{
  return sqrt(target_velocity*target_velocity - 2*acceleration*distance);
}

// The kernel called when scanning the plan from last to first entry.
static void planner_reverse_pass_kernel(block_t *current, block_t *next)
{
  if (!current || !next) return;  // Skip the last block: already initialized and set for recalculation

  // If entry speed is already at the maximum entry speed, no need to recheck: block is cruising.
  // If not, the block is accelerating or decelerating. Reset entry speed to maximum and
  // check for maximum allowable speed reductions to ensure maximum possible planned speed.
  if (current->entry_speed != current->max_entry_speed) {
    // If nominal length is true, max junction speed is guaranteed to be reached. Only compute
    // the max allowable speed if the block is decelerating and nominal length is false.
    if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
      current->entry_speed = min(current->max_entry_speed,
        max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
    }
    else {
      current->entry_speed = current->max_entry_speed;
    }
    current->recalculate_flag = true;
  }
}

// Reverse pass: scan the plan from the newest block to the oldest.
static void planner_reverse_pass()
{
  // Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END

  // The newest blocks are skipped (nothing newer to plan against), and the oldest block
  // (at tail) is never changed because it may be executing already.
  if (((block_buffer_head - tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) {
    uint8_t next = (block_buffer_head - 4) & (BLOCK_BUFFER_SIZE - 1);
    while (next != tail) {
      uint8_t current = PREV_BLOCK_INDEX(next);
      if (current == tail) break;
      planner_reverse_pass_kernel(&block_buffer[current], &block_buffer[next]);
      next = current;
    }
  }
}

// The kernel called when scanning the plan from first to last entry.
static void planner_forward_pass_kernel(block_t *previous, block_t *current)
{
  if (!previous) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, adjust the entry speed accordingly. Entry speeds have
  // already been reset, maximized, and reverse planned. If nominal length is true, max junction
  // speed is guaranteed to be reached: no need to recheck.
  if (!previous->nominal_length_flag && previous->entry_speed < current->entry_speed) {
    float entry_speed = min(current->entry_speed,
      max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));

    // Check for junction speed change
    if (current->entry_speed != entry_speed) {
      current->entry_speed = entry_speed;
      current->recalculate_flag = true;
    }
  }
}

// Forward pass: scan the plan from the oldest block to the newest.
static void planner_forward_pass()
{
  uint8_t block_index = block_buffer_tail;
  block_t *previous = NULL, *current = NULL;

  while (block_index != block_buffer_head) {
    previous = current;
    current = &block_buffer[block_index];
    planner_forward_pass_kernel(previous, current);
    block_index = NEXT_BLOCK_INDEX(block_index);
  }
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor of each junction. Must be called after updating the blocks.
static void planner_recalculate_trapezoids()
{
  uint8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    // Recalculate if the entry or exit junction speed of the current block has changed.
    if (current && (current->recalculate_flag || next->recalculate_flag)) {
      // NOTE: Entry and exit factors are always > 0 by all previous logic operations.
      calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
        next->entry_speed/current->nominal_speed);
      current->recalculate_flag = false;  // Reset current only to ensure next trapezoid is computed
    }
    block_index = NEXT_BLOCK_INDEX(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
      MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan:
//   1. Reverse pass: go over every block from the newest and reduce the junction speeds so that
//      no speed reduction within one block requires a faster deceleration than the constant acceleration.
//   2. Forward pass: go over every block from the oldest and reduce junction speeds if the speed increase
//      within one block would require a faster acceleration than the constant acceleration.
//   3. Recalculate the trapezoids of all blocks.
static void planner_recalculate()
{
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init()
{
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position));
  memset(previous_speed, 0, sizeof(previous_speed));
  previous_nominal_speed = 0.0;
}

#ifdef AUTOTEMP
// Adjust the hotend temperature to the fastest extrusion in the buffer
static void getHighESpeed()
{
  static float oldt = 0;
  if (!autotemp_enabled) return;
  if (degTargetHotend0() + 2 < autotemp_min) return;  // probably temperature set to zero.

  float high = 0.0;
  uint8_t block_index = block_buffer_tail;

  while (block_index != block_buffer_head) {
    block_t *block = &block_buffer[block_index];
    if (block->steps_x != 0 || block->steps_y != 0 || block->steps_z != 0) {
      float se = (float(block->steps_e) / float(block->step_event_count)) * block->nominal_speed;  // mm/sec
      if (se > high) high = se;
    }
    block_index = NEXT_BLOCK_INDEX(block_index);
  }

  float t = autotemp_min + high*autotemp_factor;
  if (t < autotemp_min) t = autotemp_min;
  if (t > autotemp_max) t = autotemp_max;
  if (oldt > t) {
    t = AUTOTEMP_OLDWEIGHT*oldt + (1 - AUTOTEMP_OLDWEIGHT)*t;
  }
  oldt = t;
  setTargetHotend0(t);
}
#endif

// Disable the axes that have nothing left to do and set the fan speed of the block being executed
void check_axes_activity()
{
  unsigned char x_active = 0;
  unsigned char y_active = 0;
  unsigned char z_active = 0;
  unsigned char e_active = 0;
  unsigned char tail_fan_speed = fanSpeed;

  if (block_buffer_tail != block_buffer_head)
  {
    uint8_t block_index = block_buffer_tail;
    tail_fan_speed = block_buffer[block_index].fan_speed;
    while (block_index != block_buffer_head)
    {
      block_t *block = &block_buffer[block_index];
      if (block->steps_x != 0) x_active++;
      if (block->steps_y != 0) y_active++;
      if (block->steps_z != 0) z_active++;
      if (block->steps_e != 0) e_active++;
      block_index = NEXT_BLOCK_INDEX(block_index);
    }
  }
  if ((DISABLE_X) && (x_active == 0)) disable_x();
  if ((DISABLE_Y) && (y_active == 0)) disable_y();
  if ((DISABLE_Z) && (z_active == 0)) disable_z();
  if ((DISABLE_E) && (e_active == 0))
  {
    disable_e0();
    disable_e1();
  }
#if defined(FAN_PIN) && FAN_PIN > -1
  #ifdef FAN_KICKSTART_TIME
    static unsigned long fan_kick_end;
    if (tail_fan_speed) {
      if (fan_kick_end == 0) {
        // Just starting up fan - run at full power.
        fan_kick_end = millis() + FAN_KICKSTART_TIME;
        tail_fan_speed = 255;
      } else if (fan_kick_end > millis())
        // Fan still spinning up.
        tail_fan_speed = 255;
    } else {
      fan_kick_end = 0;
    }
  #endif // FAN_KICKSTART_TIME
  #ifdef FAN_SOFT_PWM
    fanSpeedSoftPwm = tail_fan_speed;
  #else
    analogWrite(FAN_PIN, tail_fan_speed);
  #endif // !FAN_SOFT_PWM
#endif // FAN_PIN > -1
#ifdef AUTOTEMP
  getHighESpeed();
#endif
}

// Add a new linear movement to the buffer. x, y and z are the carriage positions in mm
// (the delta towers), e is the extruder position, feed_rate the speed in mm/s.
#ifdef ENABLE_AUTO_BED_LEVELING
void plan_buffer_line(float x, float y, float z, const float &e, float feed_rate, const uint8_t &extruder)
#else
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder)
#endif
{
  // Calculate the buffer head after we push this block
  int next_buffer_head = NEXT_BLOCK_INDEX(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head)
  {
    manage_heater();
    manage_inactivity();
  }

#ifdef ENABLE_AUTO_BED_LEVELING
  apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
#endif

  // The target position of the tool in absolute steps.
  // This is done after the wait, because otherwise an M92 code within the gcode disrupts this calculation.
  long target[NUM_AXIS];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);
  target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);

#ifdef PREVENT_DANGEROUS_EXTRUDE
  if (target[E_AXIS] != position[E_AXIS])
  {
    if (degHotend(active_extruder) < extrude_min_temp)
    {
      position[E_AXIS] = target[E_AXIS];  // behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
    }
  #ifdef PREVENT_LENGTHY_EXTRUDE
    if (labs(target[E_AXIS] - position[E_AXIS]) > axis_steps_per_unit[E_AXIS]*EXTRUDE_MAXLENGTH)
    {
      position[E_AXIS] = target[E_AXIS];  // behave as if the move really took place, but ignore E part
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
    }
  #endif
  }
#endif

  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Mark block as not busy (not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS] - position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS] - position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS] - position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS] - position[E_AXIS]);
  block->steps_e *= volumetric_multiplier[active_extruder];
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100;
  block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments) return;

  block->fan_speed = fanSpeed;

  // Compute direction bits for this block
  block->direction_bits = 0;
  for (uint8_t axis = 0; axis < NUM_AXIS; axis++)
  {
    if (target[axis] < position[axis]) block->direction_bits |= (1 << axis);
  }

  block->active_extruder = extruder;

  // Enable active axes
  if (block->steps_x != 0) enable_x();
  if (block->steps_y != 0) enable_y();
  if (block->steps_z != 0) enable_z();

  // Enable extruder(s)
  if (block->steps_e != 0)
  {
    if (DISABLE_INACTIVE_EXTRUDER)  // enable only the selected extruder
    {
      for (uint8_t i = 0; i < 2; i++)
      {
        if (extruder_last_move[i] > 0) extruder_last_move[i]--;
      }
      switch (extruder)
      {
        case 0:
          enable_e0();
          extruder_last_move[0] = BLOCK_BUFFER_SIZE*2;
          if (extruder_last_move[1] == 0) disable_e1();
          break;
        case 1:
          enable_e1();
          extruder_last_move[1] = BLOCK_BUFFER_SIZE*2;
          if (extruder_last_move[0] == 0) disable_e0();
          break;
      }
    }
    else  // enable all
    {
      enable_e0();
      enable_e1();
    }
  }

  if (block->steps_e == 0)
  {
    if (feed_rate < mintravelfeedrate) feed_rate = mintravelfeedrate;
  }
  else
  {
    if (feed_rate < minimumfeedrate) feed_rate = minimumfeedrate;
  }

  // Length of the movement in mm of each axis
  float delta_mm[NUM_AXIS];
  delta_mm[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / axis_steps_per_unit[Y_AXIS];
  delta_mm[Z_AXIS] = (target[Z_AXIS] - position[Z_AXIS]) / axis_steps_per_unit[Z_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS] - position[E_AXIS]) / axis_steps_per_unit[E_AXIS]) * volumetric_multiplier[active_extruder] * extrudemultiply / 100.0;
  if (block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments)
  {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  else
  {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  float inverse_millimeters = 1.0/block->millimeters;  // inverse millimeters to remove multiple divides
  float inverse_second = feed_rate * inverse_millimeters;  // 1 / (time of the block in seconds)

  block->nominal_speed = block->millimeters * inverse_second;  // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second);  // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0;  // factor <= 1 to decrease speed
  for (uint8_t i = 0; i < NUM_AXIS; i++)
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if (fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed
  if (speed_factor < 1.0)
  {
    for (uint8_t i = 0; i < NUM_AXIS; i++) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;
  if (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
  {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm);  // convert to: acceleration steps/sec^2
  }
  else
  {
    block->acceleration_st = ceil(acceleration * steps_per_mm);  // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    const long steps[NUM_AXIS] = { block->steps_x, block->steps_y, block->steps_z, block->steps_e };
    static const uint8_t limit_order[NUM_AXIS] = { X_AXIS, Y_AXIS, E_AXIS, Z_AXIS };
    for (uint8_t i = 0; i < NUM_AXIS; i++)
    {
      uint8_t axis = limit_order[i];
      if (((float)block->acceleration_st * (float)steps[axis] / (float)block->step_event_count) > axis_steps_per_sqr_second[axis])
        block->acceleration_st = axis_steps_per_sqr_second[axis];
    }
  }
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2;
  float vmax_junction_factor = 1.0;
  if (fabs(current_speed[Z_AXIS]) > max_z_jerk/2)
    vmax_junction = min(vmax_junction, max_z_jerk/2);
  if (fabs(current_speed[E_AXIS]) > max_e_jerk/2)
    vmax_junction = min(vmax_junction, max_e_jerk/2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  int moves_queued = (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(square(current_speed[X_AXIS] - previous_speed[X_AXIS]) + square(current_speed[Y_AXIS] - previous_speed[Y_AXIS]));
    vmax_junction = block->nominal_speed;
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    }
    if (fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    }
    if (fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    }
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor);  // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  float v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Set the flag if the block will always reach the maximum junction speed regardless of entry/exit
  // speeds: a block that can accelerate from or decelerate to a stop within its length always
  // has its neighbouring junction speeds at their maximum, so they can be ignored by the planner.
  block->nominal_length_flag = (block->nominal_speed <= v_allowable);
  block->recalculate_flag = true;  // Always calculate trapezoid for new block

  // Update previous speeds
  memcpy(previous_speed, current_speed, sizeof(previous_speed));
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, safe_speed/block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  memcpy(position, target, sizeof(target));

  planner_recalculate();

  st_wake_up();
}

#ifdef ENABLE_AUTO_BED_LEVELING
vector_3 plan_get_position()
{
  vector_3 position = vector_3(st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));
  matrix_3x3 inverse = matrix_3x3::transpose(plan_bed_level_matrix);
  position.apply_rotation(inverse);
  return position;
}

void plan_set_position(float x, float y, float z, const float &e)
{
  apply_rotation_xyz(plan_bed_level_matrix, x, y, z);
#else
void plan_set_position(const float &x, const float &y, const float &z, const float &e)
{
#endif
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
  st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
  previous_nominal_speed = 0.0;  // Resets planner junction speeds. Assumes start from rest.
  memset(previous_speed, 0, sizeof(previous_speed));
}

void plan_set_e_position(const float &e)
{
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
  st_set_e_position(position[E_AXIS]);
}

uint8_t movesplanned()
{
  return (block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
}

#ifdef PREVENT_DANGEROUS_EXTRUDE
void set_extrude_min_temp(float temp)
{
  extrude_min_temp = temp;
}
#endif

// Calculate the steps/s^2 acceleration rates, based on the mm/s^2
void reset_acceleration_rates()
{
  for (uint8_t i = 0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }
}
