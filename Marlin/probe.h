// FSR Z probe (G30) and delta calibration (G33)

#ifndef PROBE_H
#define PROBE_H

extern unsigned int fsr_sensitivity;  // trigger threshold in ADC counts (M974)
extern float probe_z_offset;          // nozzle height above the bed when the probe fires, mm (M851)
extern bool delta_calibrated;         // a G33 run finished and its result was kept

bool probe_set_sensitivity(unsigned int counts);  // false if out of range
void probe_g29(bool save);                        // probe the bed mesh
void mesh_set_active(bool on);                    // switch the mesh on or off without moving the nozzle
void probe_g30();                                 // probe at the current X and Y, report the bed height
void probe_g33(uint8_t factors, uint8_t points, bool save);

#endif
