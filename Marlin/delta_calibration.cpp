#include "delta_calibration.h"
#include <math.h>

static const float D2R = 0.017453292519943295f;

static void towers(const DeltaGeometry &g, float tx[3], float ty[3])
{
  const float base[3] = {210.0f, 330.0f, 90.0f};
  for (uint8_t a = 0; a < 3; ++a) {
    const float ang = (base[a] + g.ang[a]) * D2R;
    tx[a] = g.R * cosf(ang);
    ty[a] = g.R * sinf(ang);
  }
}

// Unknowns as one vector v (changes relative to the starting geometry g0): v[0..2] endstop offsets, v[3] delta radius, v[4..5] tower A/B
// angles, v[6] rod length. Applying it to g0 gives the candidate machine.
static const float DAMPING = 0.02f;
// finite-difference step of factor j (mm, or degrees for the two angles): radius and rod use a longer step
static float stepOf(uint8_t j) { return (j == 3 || j == 6) ? 0.1f : 0.05f; }

// Effector height of the candidate machine v when its carriages sit at the heights c the firmware commanded plus the offsets v[0..2].
static bool modelZ(const DeltaGeometry &g0, const float v[7], const float c[3], float &z)
{
  DeltaGeometry g = g0;
  g.R += v[3];
  g.ang[0] += v[4];
  g.ang[1] += v[5];
  g.L += v[6];
  const float h[3] = {c[0] + v[0], c[1] + v[1], c[2] + v[2]};
  return dcal_effector_z(g, h, z);
}

void dcal_point_xy(uint8_t i, uint8_t n, float r, float xy[2])
{
  const float two_pi = 6.2831853f;
  if (i + 1 >= n) {
    xy[0] = 0.0f;
    xy[1] = 0.0f;
    return;
  }
  if (i < 6) {
    xy[0] = r * sinf(two_pi * i / 6.0f);
    xy[1] = r * cosf(two_pi * i / 6.0f);
    return;
  }
  const uint8_t k = i - 6;
  xy[0] = 0.5f * r * sinf(two_pi * k / 3.0f);
  xy[1] = 0.5f * r * cosf(two_pi * k / 3.0f);
}

bool dcal_carriage_heights(const DeltaGeometry &g, float x, float y, float z, float h[3])
{
  float tx[3], ty[3];
  towers(g, tx, ty);
  for (uint8_t a = 0; a < 3; ++a) {
    const float dx = x - tx[a], dy = y - ty[a];
    const float inside = g.L * g.L - dx * dx - dy * dy;
    if (inside <= 0.0f) return false;
    h[a] = z + sqrtf(inside);
  }
  return true;
}

bool dcal_effector_z(const DeltaGeometry &g, const float h[3], float &z)
{
  float tx[3], ty[3];
  towers(g, tx, ty);
  const float p1[3] = {tx[0], ty[0], h[0]}, p2[3] = {tx[1], ty[1], h[1]}, p3[3] = {tx[2], ty[2], h[2]};
  float ex[3], d2 = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    ex[i] = p2[i] - p1[i];
    d2 += ex[i] * ex[i];
  }
  const float d = sqrtf(d2);
  if (!(d > 1.0e-3f)) return false;
  for (uint8_t i = 0; i < 3; ++i) ex[i] /= d;
  float v[3], ii = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    v[i] = p3[i] - p1[i];
    ii += ex[i] * v[i];
  }
  float ey[3], j2 = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    ey[i] = v[i] - ii * ex[i];
    j2 += ey[i] * ey[i];
  }
  const float jn = sqrtf(j2);
  if (!(jn > 1.0e-3f)) return false;
  for (uint8_t i = 0; i < 3; ++i) ey[i] /= jn;
  const float ez[3] = {ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0]};
  const float xx = d * 0.5f;
  const float yy = (ii * ii + jn * jn - 2.0f * ii * xx) / (2.0f * jn);
  const float zz2 = g.L * g.L - xx * xx - yy * yy;
  if (!(zz2 > 0.0f)) return false;
  const float zz = sqrtf(zz2);
  const float a1 = p1[2] + xx * ex[2] + yy * ey[2] + zz * ez[2];
  const float a2 = p1[2] + xx * ex[2] + yy * ey[2] - zz * ez[2];
  z = a1 < a2 ? a1 : a2;
  return true;
}

bool dcal_solve(const DeltaGeometry &g0, uint8_t n, float probe_radius, const float err[], uint8_t factors, DeltaGeometry &out,
                float *rms_before, float *rms_after)
{
  if (n != 7 && n != 10) return false;
  if (factors != 3 && factors != 4 && factors != 6 && factors != 7) return false;
  const uint8_t F = factors;
  float v[7] = {0, 0, 0, 0, 0, 0, 0};
  float sq_before = 0.0f;
  for (uint8_t i = 0; i < n; ++i) sq_before += err[i] * err[i];
  if (rms_before) *rms_before = sqrtf(sq_before / n);
  for (uint8_t iter = 0; iter < 3; ++iter) {  // 0,1: Gauss-Newton steps; 2: only measures the fit left over (see rms_after)
    float A[7][8];
    for (uint8_t a = 0; a < F; ++a)
      for (uint8_t b = 0; b <= F; ++b) A[a][b] = 0.0f;
    float sq = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
      float xy[2], c[3], z0;
      dcal_point_xy(i, n, probe_radius, xy);
      if (!dcal_carriage_heights(g0, xy[0], xy[1], 0.0f, c) || !modelZ(g0, v, c, z0)) return false;
      const float f = z0 - err[i];  // model minus measured
      sq += f * f;
      if (iter == 2) continue;
      float J[7];
      for (uint8_t j = 0; j < F; ++j) {
        const float keep = v[j];
        float zj;
        v[j] = keep + stepOf(j);
        const bool ok = modelZ(g0, v, c, zj);
        v[j] = keep;
        if (!ok) return false;
        J[j] = (zj - z0) / stepOf(j);
      }
      for (uint8_t a = 0; a < F; ++a) {
        for (uint8_t b = 0; b < F; ++b) A[a][b] += J[a] * J[b];
        A[a][F] += -J[a] * f;
      }
    }
    if (iter == 2) {
      if (rms_after) *rms_after = sqrtf(sq / n);
      break;
    }
    // Levenberg-Marquardt damping: the diagonal is stretched slightly so weakly determined directions (delta radius against rod
    // length) cannot swing on probe quantization noise.
    for (uint8_t a = 0; a < F; ++a) A[a][a] *= 1.0f + DAMPING;
    // Gauss-Jordan with partial pivoting
    for (uint8_t col = 0; col < F; ++col) {
      uint8_t piv = col;
      for (uint8_t r = col + 1; r < F; ++r)
        if (fabsf(A[r][col]) > fabsf(A[piv][col])) piv = r;
      if (!(fabsf(A[piv][col]) > 1.0e-9f)) return false;  // singular: not enough information
      if (piv != col)
        for (uint8_t cc = 0; cc <= F; ++cc) {
          const float t = A[piv][cc];
          A[piv][cc] = A[col][cc];
          A[col][cc] = t;
        }
      const float inv = 1.0f / A[col][col];
      for (uint8_t cc = col; cc <= F; ++cc) A[col][cc] *= inv;
      for (uint8_t r = 0; r < F; ++r) {
        if (r == col) continue;
        const float k = A[r][col];
        if (k == 0.0f) continue;
        for (uint8_t cc = col; cc <= F; ++cc) A[r][cc] -= k * A[col][cc];
      }
    }
    for (uint8_t j = 0; j < F; ++j) {
      if (!(fabsf(A[j][F]) < 1.0e4f)) return false;  // NaN / blown up
      v[j] += A[j][F];
    }
  }
  out = g0;
  for (uint8_t a = 0; a < 3; ++a) out.off[a] += v[a];
  out.R += v[3];
  out.ang[0] += v[4];
  out.ang[1] += v[5];
  out.L += v[6];
  return true;
}
