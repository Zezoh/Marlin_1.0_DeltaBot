// Bed mesh math: interpolation, fade, the inverse mapping
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "mesh.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static float bowl(float x, float y) { return 0.00004f * (x * x + y * y) - 0.1f + 0.0005f * x; }   // some smooth bed

int main() {
  for (int i = 0; i < MESH_POINTS; i++) { float xy[2]; mesh_point_xy(i, xy); mesh_z[i] = bowl(xy[0], xy[1]); }
  mesh_have = mesh_active = true; mesh_changed();

  // the mesh passes through every probed point
  for (int i = 0; i < MESH_POINTS; i++) { float xy[2]; mesh_point_xy(i, xy); float o = mesh_offset(xy[0], xy[1]);
    CHECK(fabsf(o - mesh_z[i]) < 1e-4f, "point %d: %.5f vs %.5f", i, o, mesh_z[i]); }
  // point layout: 12 around at 80 mm, 6 at 40 mm, the centre
  { float a[2], b[2]; mesh_point_xy(0, a); mesh_point_xy(3, b);
    CHECK(fabsf(a[0]) < 0.01f && fabsf(a[1] - 80) < 0.01f && fabsf(b[0] - 80) < 0.01f && fabsf(b[1]) < 0.01f, "outer ring %f %f / %f %f", a[0], a[1], b[0], b[1]);
    mesh_point_xy(12, a); CHECK(fabsf(a[1] - 40) < 0.01f, "inner ring"); mesh_point_xy(18, a); CHECK(a[0] == 0 && a[1] == 0, "centre"); }
  // between the points it follows a smooth bed closely and has no jumps
  float worst = 0, jump = 0, prev = mesh_offset(-85, 0);
  for (float x = -85; x <= 85; x += 0.5f) for (float y = -85; y <= 85; y += 5) {
    if (x * x + y * y > 85 * 85) continue;
    float o = mesh_offset(x, y), t = bowl(x, y);
    if (x * x + y * y <= 78 * 78 && fabsf(o - t) > worst) worst = fabsf(o - t);
    if (y == -85 + 5 * 17) {}
  }
  for (float x = -84; x <= 84; x += 0.1f) { float o = mesh_offset(x, 10); if (fabsf(o - prev) > jump) jump = fabsf(o - prev); prev = o; }
  printf("mesh vs a smooth bed: worst %.4f mm inside the outer ring; biggest step along a line %.4f mm per 0.1 mm\n", worst, jump);
  CHECK(worst < 0.06f, "interpolation error %.4f", worst);
  CHECK(jump < 0.01f, "steps in the mesh %.4f", jump);
  // outside the ring it stays finite and close to the edge values
  for (int a = 0; a < 360; a += 7) { float o = mesh_offset(86 * sinf(a * 0.01745f), 86 * cosf(a * 0.01745f)); CHECK(isfinite(o) && fabsf(o) < 2, "edge %d: %f", a, o); }

  // machine z and back, with and without fade
  for (int fade = 0; fade < 2; fade++) {
    mesh_fade_height = fade ? 10.0f : 0.0f; double worst2 = 0;
    for (float x = -70; x <= 70; x += 14) for (float y = -70; y <= 70; y += 14) for (float z = 0; z <= 30; z += 1.5f) {
      float mz = mesh_machine_z(x, y, z), back;
      if (!mesh_logical_z(x, y, mz, back)) { CHECK(0, "no inverse at %f %f %f", x, y, z); continue; }
      if (fabs(back - z) > worst2) worst2 = fabs(back - z);
    }
    printf("fade %.0f: machine z and back, worst %.6f mm\n", mesh_fade_height, worst2);
    CHECK(worst2 < 1e-3, "inverse error %.6f", worst2);
  }
  mesh_fade_height = 10;
  CHECK(fabsf(mesh_machine_z(30, 20, 10) - 10) < 1e-6f && fabsf(mesh_machine_z(30, 20, 50) - 50) < 1e-6f, "no correction above the fade height");
  CHECK(fabsf(mesh_machine_z(30, 20, 0) - mesh_offset(30, 20)) < 1e-5f, "full correction at Z 0");
  mesh_active = false; CHECK(mesh_machine_z(30, 20, 5) == 5, "inactive mesh changes nothing");
  printf(fails ? "mesh: %d FAILED\n" : "mesh: all ok\n", fails);
  return fails ? 1 : 0;
}
