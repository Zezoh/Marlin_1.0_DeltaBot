/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
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

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "language.h"

// Move the nozzle in a straight line from *from* to a Cartesian position and remember the new position in *from*.
// The planner works with the positions of the delta carriages, the length of the move of the nozzle is
// given so that the feed rate is the speed of the nozzle.
static bool plan_line_to(float *cartesian, float *from, float feed_rate, uint8_t extruder)
{
  if (!delta_reachable(cartesian)) return false;
  float length = sqrt(sq(cartesian[X_AXIS] - from[X_AXIS]) + sq(cartesian[Y_AXIS] - from[Y_AXIS]) + sq(cartesian[Z_AXIS] - from[Z_AXIS]));
  calculate_delta(cartesian);
  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], cartesian[E_AXIS], feed_rate, extruder, length);
  from[X_AXIS] = cartesian[X_AXIS];
  from[Y_AXIS] = cartesian[Y_AXIS];
  from[Z_AXIS] = cartesian[Z_AXIS];
  return true;
}

// The arc left the area the arms can reach: stop where it was, the parser continues from there
static void abort_arc(float *target, const float *last, float e)
{
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_OUT_OF_REACH);
  target[X_AXIS] = last[X_AXIS];
  target[Y_AXIS] = last[Y_AXIS];
  target[Z_AXIS] = last[Z_AXIS];
  target[E_AXIS] = e;
}

// The arc is approximated by generating a number of tiny, linear segments of MM_PER_ARC_SEGMENT mm.
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1,
  uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float extruder_travel = target[E_AXIS] - position[E_AXIS];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1 - r_axis1*rt_axis0, r_axis0*rt_axis0 + r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }

  // A full circle (start == target) gives an angle of zero: make it 2*Pi
  if (position[axis_0] == target[axis_0] && position[axis_1] == target[axis_1] && angular_travel == 0)
  {
    angular_travel += 2*M_PI;
  }

  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 0.001) { return; }
  uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  if (segments == 0) segments = 1;

  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  float extruder_per_segment = extruder_travel/segments;

  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation:
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi)] * r
     The center of the circle is the axis of rotation and the radius vector is defined from the circle
     center to the initial position. Each line segment is formed by successive vector rotations, with
     the small angle approximation for cos and sin. Error may accumulate from numerical round-off
     (single precision), so the exact radius vector is recomputed every N_ARC_CORRECTION segments. */
  float cos_T = 1 - 0.5*theta_per_segment*theta_per_segment;  // Small angle approximation
  float sin_T = theta_per_segment;

  float arc_target[NUM_AXIS];
  int8_t count = 0;

  arc_target[axis_linear] = position[axis_linear];
  arc_target[E_AXIS] = position[E_AXIS];
  float previous[3] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };  // where the nozzle is now

  for (uint16_t i = 1; i < segments; i++) {  // Increment (segments-1)
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix
      float r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector: exact location from the initial radius vector (= -offset)
      float cos_Ti = cos(i*theta_per_segment);
      float sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    arc_target[E_AXIS] += extruder_per_segment;

    clamp_to_software_endstops(arc_target);
    if (!plan_line_to(arc_target, previous, feed_rate, extruder)) {
      abort_arc(target, previous, arc_target[E_AXIS] - extruder_per_segment);
      return;
    }
  }
  // Ensure last segment arrives at target location.
  clamp_to_software_endstops(target);
  if (!plan_line_to(target, previous, feed_rate, extruder)) {
    abort_arc(target, previous, arc_target[E_AXIS]);
  }
}
