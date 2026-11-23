// Bed mesh (G29): 19 probed heights on a circle, interpolated over triangles, added to Z while it is active.
// Pure functions and data, no hardware access (the host tests build mesh.cpp unchanged).

#ifndef MESH_H
#define MESH_H

#include <stdint.h>

#define MESH_POINTS 19      // 12 on the outer ring (every 30 degrees), 6 on the inner ring (every 60 degrees), the centre
#define MESH_TRIANGLES 24

#ifndef MESH_OUTER_RADIUS_MM
#define MESH_OUTER_RADIUS_MM 80.0f
#endif
#ifndef MESH_INNER_RADIUS_MM
#define MESH_INNER_RADIUS_MM 40.0f
#endif

extern float mesh_z[MESH_POINTS];  // bed height at each point, mm (what G29 probed)
extern bool mesh_have;             // mesh_z holds a probed mesh
extern bool mesh_active;           // Z is corrected by it
extern float mesh_fade_height;     // the correction fades out linearly up to this height, 0 = never

void mesh_point_xy(uint8_t index, float xy[2]);   // where point `index` is probed
void mesh_changed();                              // call after mesh_z was written
float mesh_offset(float x, float y);              // interpolated bed height at x, y
float mesh_machine_z(float x, float y, float z);  // commanded Z -> Z the nozzle goes to (mesh active)
bool mesh_logical_z(float x, float y, float machine_z, float &z);   // the inverse

#endif
