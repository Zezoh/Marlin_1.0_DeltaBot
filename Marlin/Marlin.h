// Marlin delta firmware: common definitions

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#define  HardwareSerial_h // trick to disable the standard HWserial

#include "Arduino.h"

#include "MarlinSerial.h"

// Newer Arduino cores no longer define min() and max() (they use std::min), older ones do: always have the macros
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define MYSERIAL MSerial

#define SERIAL_PROTOCOL(x) (MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) (MYSERIAL.print(x,y))
#define SERIAL_PROTOCOLPGM(x) (serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) (MYSERIAL.print(x),MYSERIAL.write('\n'))
#define SERIAL_PROTOCOLLNPGM(x) (serialprintPGM(PSTR(x)),MYSERIAL.write('\n'))


extern const char errormagic[] PROGMEM;
extern const char echomagic[] PROGMEM;

#define SERIAL_ERROR_START (serialprintPGM(errormagic))
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START (serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, unsigned long v);


//Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}


void get_command();
void process_commands();

void manage_inactivity(bool ignore_stepper_queue=false);

#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() { WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() { WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void calculate_delta(float cartesian[3]);
void home_all_axes();                                          // G28
void move_to(float x, float y, float z, float feed_mm_s);      // queue a straight move of the nozzle
void cancel_print();                                           // stop the SD print, heater off, home
void move_e(float mm, float feed_mm_s);                        // queue an extruder move of mm (negative: back), the nozzle stays
void keep_steppers_on();                                       // restart the idle timer that releases the steppers
bool machine_idle();                                           // no G-code waiting and no move queued
bool delta_reachable(const float cartesian[3]);
extern float delta[3];
void prepare_move();
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ASCII command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ASCII command at the end of the current buffer, read from flash
bool command_is(const char *line, char letter, int code); // the line is exactly this command (no substring matches)
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // extrusion factor in percent (M221)
extern float current_position[NUM_AXIS] ;
extern float add_homing[3];
extern float endstop_adj[3];
extern float delta_radius;
extern float delta_tower_angle[3];  // tower angle corrections, degrees (G33 / M665 A B C)
extern float delta_diagonal_rod;
extern float delta_segments_per_second;
void recalc_delta_settings(float radius, float diagonal_rod);
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern int fanSpeed;
extern bool cancel_heatup;


extern unsigned long starttime;
extern unsigned long stoptime;

extern uint8_t active_extruder;   // always 0: there is one extruder

#endif


