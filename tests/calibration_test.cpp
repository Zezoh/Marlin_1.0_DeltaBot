// G33 solver test: simulate a delta whose real geometry differs from the one the firmware believes,
// "probe" it, run the real solver and check that it finds the real geometry.
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "delta_calibration.h"

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

static float frand(float a) { return a * (2.0f * rand() / RAND_MAX - 1.0f); }

// What the probe sees: nozzle height above the bed (true machine, flat bed at 0) when the firmware thinks it is at z = 0.
static void simulate(const DeltaGeometry &belief, const DeltaGeometry &truth, uint8_t n, float pr, float err[])
{
  for (uint8_t i = 0; i < n; i++) {
    float xy[2], c[3], h[3], z;
    dcal_point_xy(i, n, pr, xy);
    dcal_carriage_heights(belief, xy[0], xy[1], 0.0f, c);
    for (int a = 0; a < 3; a++) h[a] = c[a] + truth.off[a];
    CHECK(dcal_effector_z(truth, h, z), "forward kinematics");
    err[i] = z;
  }
}

int main()
{
  srand(7);
  // forward kinematics inverts carriage heights
  DeltaGeometry g = {90, 210, {0, 0, 0}, {0, 0, 0}};
  for (int k = 0; k < 100; k++) {
    float x = frand(80), y = frand(80), z = 5 + frand(5), h[3], zz;
    dcal_carriage_heights(g, x, y, z, h);
    CHECK(dcal_effector_z(g, h, zz) && fabsf(zz - z) < 0.01f, "effector z %f vs %f", zz, z);
  }

  for (uint8_t factors = 3; factors <= 6; factors += (factors == 4 ? 2 : 1)) {
    for (uint8_t n = 7; n <= 10; n += 3) {
      for (int trial = 0; trial < 20; trial++) {
        DeltaGeometry belief = {90, 210, {0, 0, 0}, {0, 0, 0}};
        DeltaGeometry truth = belief;
        for (int a = 0; a < 3; a++) truth.off[a] = frand(2.0f);
        if (factors >= 4) truth.R += frand(1.5f);
        if (factors >= 6) {
          truth.ang[0] += frand(0.4f);
          truth.ang[1] += frand(0.4f);
        }
        float err[DCAL_MAX_POINTS], before, after;
        simulate(belief, truth, n, 85, err);
        DeltaGeometry out;
        bool ok = dcal_solve(belief, n, 85, err, factors, out, &before, &after);
        CHECK(ok, "solve failed F%d P%d", factors, n);
        if (!ok) continue;
        // the solved geometry must predict the measured errors
        CHECK(after < 0.01f, "F%d P%d trial %d: residual rms %.4f (before %.3f)", factors, n, trial, after, before);
        // and match the real machine; the endstop offsets are only defined up to a common shift when the model has no fixed height
        for (int a = 0; a < 3; a++)
          CHECK(fabsf(out.off[a] - truth.off[a]) < 0.05f, "F%d P%d trial %d: off[%d] %.3f vs %.3f", factors, n, trial, a, out.off[a], truth.off[a]);
        if (factors >= 4) CHECK(fabsf(out.R - truth.R) < 0.1f, "F%d P%d: R %.3f vs %.3f", factors, n, out.R, truth.R);
        if (factors >= 6)
          for (int a = 0; a < 2; a++)
            CHECK(fabsf(out.ang[a] - truth.ang[a]) < 0.03f, "F%d P%d: ang[%d] %.3f vs %.3f", factors, n, a, out.ang[a], truth.ang[a]);
      }
    }
  }

  // a flat, correct machine needs no correction
  {
    DeltaGeometry b = {90, 210, {0, 0, 0}, {0, 0, 0}}, out;
    float err[10], bf, af;
    simulate(b, b, 10, 85, err);
    CHECK(dcal_solve(b, 10, 85, err, 6, out, &bf, &af) && bf < 0.001f && fabsf(out.R - 90) < 0.01f, "identity");
  }

  // bad input is refused
  {
    DeltaGeometry b = {90, 210, {0, 0, 0}, {0, 0, 0}}, out;
    float err[10] = {0};
    CHECK(!dcal_solve(b, 8, 85, err, 6, out, 0, 0), "8 points refused");
    CHECK(!dcal_solve(b, 10, 85, err, 5, out, 0, 0), "5 factors refused");
  }

  printf(fails ? "calibration: %d FAILED\n" : "calibration: all ok\n", fails);
  return fails ? 1 : 0;
}
