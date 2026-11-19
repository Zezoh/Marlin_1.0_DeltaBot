#pragma once
extern float axis_steps_per_unit[4];
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate, const uint8_t &extruder, float cartesian_mm = 0.0);
void plan_set_position(const float &x, const float &y, const float &z, const float &e);
