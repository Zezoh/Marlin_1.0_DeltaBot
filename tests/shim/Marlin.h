#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define F_CPU 16000000UL
#define OVERSAMPLENR 16
#define FORCE_INLINE __attribute__((always_inline)) inline
#define CRITICAL_SECTION_START
#define CRITICAL_SECTION_END
#define PSTR(x) x
#define PROGMEM
#include "Configuration.h"
#include "Configuration_adv.h"
#define pgm_read_word(x) (*(x))
#define pgm_read_byte(x) (*(x))
#define PGM_P const char*
#define SERIAL_ECHO_START
#define SERIAL_ECHOLNPGM(x)
#define SERIAL_ECHOPGM(x)
#define SERIAL_ECHOLN(x)
#define SERIAL_PROTOCOLLNPGM(x)
#define millis() 0UL
#define analogWrite(a,b)
#define FAN_PIN -1
#define NUM_AXIS_DUMMY 0
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#define square(x) ((x)*(x))
#define lround(x) ::lround(x)
#define enable_x()
#define enable_y()
#define enable_z()
#define disable_x()
#define disable_y()
#define disable_z()
#define enable_e0()
#define enable_e1()
#define enable_e2()
#define disable_e0()
#define disable_e1()
#define disable_e2()
extern int extrudemultiply;
extern float volumetric_multiplier[1];
extern int fanSpeed;
extern uint8_t active_extruder;
void manage_inactivity(bool ignore_stepper_queue=false);
float degHotend(uint8_t);
float degTargetHotend0();
void setTargetHotend0(float);
void st_wake_up();
void st_set_position(const long&,const long&,const long&,const long&);
void st_set_e_position(const long&);
