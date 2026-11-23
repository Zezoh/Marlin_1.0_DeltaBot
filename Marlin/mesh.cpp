#include "mesh.h"
#include <math.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define MESH_PROGMEM PROGMEM
#define mesh_tri(t, v) pgm_read_byte(&MESH_TRI[t][v])
#else
#define MESH_PROGMEM
#define mesh_tri(t, v) MESH_TRI[t][v]
#endif

float mesh_z[MESH_POINTS];
bool mesh_have = false;
bool mesh_active = false;
float mesh_fade_height = 0.0f;

// Triangles over the points: the inner ring around the centre (18), then the outer ring around the inner ring.
static const uint8_t MESH_TRI[MESH_TRIANGLES][3] MESH_PROGMEM = {
    {18, 12, 13},
    {18, 13, 14},
    {18, 14, 15},
    {18, 15, 16},
    {18, 16, 17},
    {18, 17, 12},
    {12, 0, 1},
    {12, 1, 2},
    {12, 2, 13},
    {13, 2, 3},
    {13, 3, 4},
    {13, 4, 14},
    {14, 4, 5},
    {14, 5, 6},
    {14, 6, 15},
    {15, 6, 7},
    {15, 7, 8},
    {15, 8, 16},
    {16, 8, 9},
    {16, 9, 10},
    {16, 10, 17},
    {17, 10, 11},
    {17, 11, 0},
    {17, 0, 12},
};

// the triangle the last lookup was in: consecutive moves stay in it
static int8_t cache_tri = -1;
static float cx0, cy0, cz0, cx1, cy1, cz1, cx2, cy2, cz2, cinv;

// Probe positions are rounded down to 1/256 mm so a point is always the same point.
static float quantize(float v)
{
  return copysignf(floorf(fabsf(v) * 256.0f) / 256.0f, v);
}

void mesh_point_xy(uint8_t index, float xy[2])
{
  const float d2r = 0.017453292519943295f;
  if (index < 12) {
    xy[0] = quantize(MESH_OUTER_RADIUS_MM * sinf(index * 30.0f * d2r));
    xy[1] = quantize(MESH_OUTER_RADIUS_MM * cosf(index * 30.0f * d2r));
  } else if (index < 18) {
    xy[0] = quantize(MESH_INNER_RADIUS_MM * sinf((index - 12) * 60.0f * d2r));
    xy[1] = quantize(MESH_INNER_RADIUS_MM * cosf((index - 12) * 60.0f * d2r));
  } else {
    xy[0] = xy[1] = 0.0f;
  }
}

void mesh_changed() { cache_tri = -1; }

static bool load_triangle(uint8_t t)
{
  uint8_t i0 = mesh_tri(t, 0), i1 = mesh_tri(t, 1), i2 = mesh_tri(t, 2);
  float p0[2], p1[2], p2[2];
  mesh_point_xy(i0, p0);
  mesh_point_xy(i1, p1);
  mesh_point_xy(i2, p2);
  float denom = (p1[1] - p2[1]) * (p0[0] - p2[0]) + (p2[0] - p1[0]) * (p0[1] - p2[1]);
  if (fabsf(denom) < 1.0e-6f) return false;
  cx0 = p0[0];
  cy0 = p0[1];
  cz0 = mesh_z[i0];
  cx1 = p1[0];
  cy1 = p1[1];
  cz1 = mesh_z[i1];
  cx2 = p2[0];
  cy2 = p2[1];
  cz2 = mesh_z[i2];
  cinv = 1.0f / denom;
  cache_tri = t;
  return true;
}

// barycentric weights in the cached triangle; false if x, y is outside it
static bool weights(float x, float y, float w[3])
{
  if (cache_tri < 0) return false;
  w[0] = ((cy1 - cy2) * (x - cx2) + (cx2 - cx1) * (y - cy2)) * cinv;
  w[1] = ((cy2 - cy0) * (x - cx2) + (cx0 - cx2) * (y - cy2)) * cinv;
  w[2] = 1.0f - w[0] - w[1];
  return w[0] >= -0.0005f && w[1] >= -0.0005f && w[2] >= -0.0005f;
}

float mesh_offset(float x, float y)
{
  float w[3];
  if (weights(x, y, w)) return w[0] * cz0 + w[1] * cz1 + w[2] * cz2;
  for (uint8_t t = 0; t < MESH_TRIANGLES; t++) {
    if (!load_triangle(t)) continue;
    if (weights(x, y, w)) return w[0] * cz0 + w[1] * cz1 + w[2] * cz2;
  }
  // outside the outer ring: the height of the ring edge in that direction
  for (uint8_t edge = 0; edge < 12; edge++) {
    uint8_t next = (edge + 1) % 12;
    float a[2], b[2];
    mesh_point_xy(edge, a);
    mesh_point_xy(next, b);
    float ex = b[0] - a[0], ey = b[1] - a[1];
    float denom = x * ey - y * ex;
    if (fabsf(denom) < 1.0e-6f) continue;
    float scale = (a[0] * ey - a[1] * ex) / denom;
    float along = (a[0] * y - a[1] * x) / denom;
    if (scale < 0.0f || along < -1.0e-5f || along > 1.00001f) continue;
    float u = along < 0.0f ? 0.0f : (along > 1.0f ? 1.0f : along);
    return mesh_z[edge] + u * (mesh_z[next] - mesh_z[edge]);
  }
  return 0.0f;
}

float mesh_machine_z(float x, float y, float z)
{
  if (!mesh_active) return z;
  float fade = mesh_fade_height > 1.0e-6f ? (z >= mesh_fade_height ? 0.0f : 1.0f - z / mesh_fade_height) : 1.0f;
  return z + mesh_offset(x, y) * fade;
}

bool mesh_logical_z(float x, float y, float machine_z, float &z)
{
  z = machine_z;
  if (!mesh_active) return true;
  float offset = mesh_offset(x, y);
  if (mesh_fade_height <= 1.0e-6f) {
    z -= offset;
  } else if (machine_z < mesh_fade_height) {
    float slope = 1.0f - offset / mesh_fade_height;
    if (slope <= 1.0e-6f) return false;
    z = (machine_z - offset) / slope;
  }
  return true;
}
