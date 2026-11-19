// Delta auto-calibration math (G33): least-squares fit of the delta geometry to probed bed heights.
// Pure functions, no hardware access (the host tests in tests/ build this file unchanged).

#ifndef DELTA_CALIBRATION_H
#define DELTA_CALIBRATION_H

#include <stdint.h>

#define DCAL_MAX_POINTS 10   // probe points per pass: 7 (quick) or 10

// Delta geometry as the calibration sees it. Towers sit at 210, 330 and 90 degrees plus the angle corrections.
struct DeltaGeometry {
  float R;        // tower circle radius, mm
  float L;        // diagonal rod length, mm
  float ang[3];   // tower angle corrections, degrees
  float off[3];   // endstop offsets, mm: how far each carriage really sits above where the firmware believes it
};

// XY of probe point i of n (7 or 10): an outer ring of six, an inner ring of three at half the radius, the centre last.
void dcal_point_xy(uint8_t i, uint8_t n, float probe_radius, float xy[2]);

// Carriage heights that put the effector at (x, y, z). False if a rod cannot reach.
bool dcal_carriage_heights(const DeltaGeometry &g, float x, float y, float z, float h[3]);

// Forward kinematics: effector height for the carriage heights h. False if there is no solution.
bool dcal_effector_z(const DeltaGeometry &g, const float h[3], float &z);

// Fit the geometry g0 to the probe errors err[] (nozzle height above the bed at each point, mm).
// factors: 3 = endstop offsets, 4 = + delta radius, 6 = + tower angles A and B (7 = + rod length).
// The result is in out; off[] holds the correction of each carriage (added to g0.off).
bool dcal_solve(const DeltaGeometry &g0, uint8_t n, float probe_radius, const float err[], uint8_t factors,
                DeltaGeometry &out, float *rms_before, float *rms_after);

#endif
