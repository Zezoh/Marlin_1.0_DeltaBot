/*
  temperature.h - temperature controller of the hotend
  Part of Marlin, Copyright (c) 2011 Erik van der Zalm, GNU GPL v3 or later
*/

#ifndef temperature_h
#define temperature_h

#include "Marlin.h"
#include "planner.h"

void tp_init();          // initialize the heating
void manage_heater();    // it is critical that this is called periodically
void disable_heater();
void updatePID();
void PID_autotune(float temp, int ncycles);
int getHeaterPower(int heater);

extern volatile unsigned int fsr_raw;   // latest FSR (Z probe) ADC reading, 0..1023
extern int target_temperature[EXTRUDERS];
extern float current_temperature[EXTRUDERS];
extern float Kp, Ki, Kd;
float scalePID_i(float i);
float scalePID_d(float d);
float unscalePID_i(float i);
float unscalePID_d(float d);

// the extruder argument is always 0: there is one hotend
FORCE_INLINE float degHotend(uint8_t extruder) { return current_temperature[extruder]; }
FORCE_INLINE float degTargetHotend(uint8_t extruder) { return target_temperature[extruder]; }
FORCE_INLINE void setTargetHotend(const float &celsius, uint8_t extruder) { target_temperature[extruder] = celsius; }
FORCE_INLINE bool isHeatingHotend(uint8_t extruder) { return target_temperature[extruder] > current_temperature[extruder]; }
FORCE_INLINE bool isCoolingHotend(uint8_t extruder) { return target_temperature[extruder] < current_temperature[extruder]; }

#endif
