#pragma once
void st_synchronize();
long st_get_position(uint8_t axis);
void st_probe_arm(unsigned int threshold);
void st_probe_disarm();
bool st_probe_hit();
unsigned int st_probe_peak();
