/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "probe.h"
#include "mesh.h"
#include "filament.h"
#include "fan.h"
#include "ui.h"
#include "pins_arduino.h"
#include "math.h"

// G-codes: G0/G1 move, G2/G3 arc, G4 dwell, G28 home, G29 bed mesh, G30 probe, G33 delta calibration,
//          G90/G91 absolute/relative, G92 set E
// M-codes: M17/M18/M84 steppers, M20-M32/M928 SD card, M80/M81, M82/M83, M92 steps per unit, M104/M109 hotend,
//          M105 temperatures, M106/M107 fan, M112 emergency stop, M114 position, M115 firmware, M119 endstops,
//          M201-M205 limits, M206 home offset, M220/M221 speed and flow, M301 PID, M302 cold extrusion, M303 PID autotune,
//          M400 finish moves, M412 runout sensor, M420 bed mesh, M500-M503 settings, M600 filament change,
//          M665/M666 delta geometry, M851 probe offset, M974 FSR sensitivity, M999 restart after an error.
// Accepted and ignored: M117, M140, M190 (there is no display and no heated bed), T0.

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
CardReader card;
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homing[3]={0,0,0};
float endstop_adj[3]={0,0,0};

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};

// Extruder offset
uint8_t active_extruder = 0;
int fanSpeed=0;

  float delta[3] = {0.0, 0.0, 0.0};
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  // these are the default values, can be overriden with M665
  float delta_radius= DELTA_RADIUS;
  float delta_tower1_x= -SIN_60*delta_radius; // front left tower
  float delta_tower1_y= -COS_60*delta_radius;	   
  float delta_tower2_x=  SIN_60*delta_radius; // front right tower
  float delta_tower2_y= -COS_60*delta_radius;	   
  float delta_tower3_x= 0.0;                  // back middle tower
  float delta_tower3_y= delta_radius;
  float delta_tower_angle[3] = {0, 0, 0}; // degrees added to the tower angles 210, 330 and 90
  float delta_diagonal_rod= DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_2= sq(delta_diagonal_rod);
  float delta_segments_per_second= DELTA_SEGMENTS_PER_SECOND;

bool cancel_heatup = false;   // M108: ends a wait for the temperature or a filament change

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

static float offset[3] = {0.0, 0.0, 0.0};
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc


//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;



bool Stopped=false;



//===========================================================================
//=============================Routines======================================
//===========================================================================

void get_arc_coordinates();

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

extern "C" {
  extern unsigned int __bss_end;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}

// Adds a command to the end of the command buffer. Not safe against mixing with
// commands arriving over serial while it runs: it is only called from the main loop.
static void enquecommand_internal(const char *cmd, bool from_flash)
{
  if (buflen < BUFSIZE)
  {
    char *dst = cmdbuffer[bufindw];
    if (from_flash) strncpy_P(dst, cmd, MAX_CMD_SIZE - 1);
    else strncpy(dst, cmd, MAX_CMD_SIZE - 1);
    dst[MAX_CMD_SIZE - 1] = 0;
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_Enqueing);
    SERIAL_ECHO(dst);
    SERIAL_ECHOLNPGM("\"");
    bufindw = (bufindw + 1) % BUFSIZE;
    buflen += 1;
  }
}

void enquecommand(const char *cmd) { enquecommand_internal(cmd, false); }
void enquecommand_P(const char *cmd) { enquecommand_internal(cmd, true); }

void setup()
{
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(" " SHORT_BUILD_VERSION);

      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  ui_init();
  filament_sensor_init();
  fan_init();
  


}


void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  card.checkautostart(false);
  if(buflen)
  {
      if(card.saving)
      {
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
        {
          card.write_command(cmdbuffer[bufindr]);
          if(card.logging)
          {
            process_commands();
          }
          else
          {
            SERIAL_PROTOCOLLNPGM(MSG_OK);
          }
        }
        else
        {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }
      }
      else
      {
        process_commands();
      }
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  filament_service();
  ui_run();
}

void get_command()
{
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if (Stopped == true) {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            }
            break;
          default:
            break;
          }

        }

        //If command was e-stop process now
        if(strcmp(cmdbuffer[bufindw], "M112") == 0)
          kill();
        
        if(strstr_P(cmdbuffer[bufindw], PSTR("M108")) != NULL) {   // ends a wait at once
          cancel_heatup = true;
          SERIAL_PROTOCOLLNPGM(MSG_OK);
        } else if(!filament_hotkey(cmdbuffer[bufindw])) {   // M24 and M701 go on at once while a filament change waits
          bufindw = (bufindw + 1)%BUFSIZE;
          buflen += 1;
        }
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  if(!card.sdprinting || serial_count!=0){
    return;
  }

  //'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
  // if it occurs, stop_buffering is triggered and the buffer is ran dry.
  // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

  static bool stop_buffering=false;
  if(buflen==0) stop_buffering=false;

  while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == '#' && comment_mode == false) ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    {
      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        card.printingHasFinished();
        card.checkautostart(true);

      }
      if(serial_char=='#')
        stop_buffering=true;

      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }


}


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);

static void axis_is_at_home(int axis) {
  current_position[axis] = base_home_pos(axis) + add_homing[axis];
  // the bed stays at Z = 0: the home offset of Z moves the home height, not the lowest position
  min_pos[axis] =          base_min_pos(axis) + (axis == Z_AXIS ? 0 : add_homing[axis]);
  max_pos[axis] =          base_max_pos(axis) + add_homing[axis];
}

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) (LETTER##_MAX_PIN > -1)

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


    destination[axis] = 1.5 * max_length(axis);  // homing is always towards the max endstop
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis);
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis);
    feedrate = HOMING_SLOW_FEEDRATE;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;


  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

// Home the three towers up against their endstops. A delta can only home all axes together.
void home_all_axes()
{
  ui_busy(true);
  bool mesh_was_active = mesh_active;
  mesh_active = false;   // the position is not known, and the carriages end up where the endstops are

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  previous_millis_cmd = millis();

  enable_endstops(true);

  for(int8_t i=0; i < NUM_AXIS; i++) {
    destination[i] = current_position[i];
  }
  feedrate = 0.0;

      // A delta can only safely home all axis at the same time
      // all axis have to home at the same time

      // Move all carriages up together until the first endstop is hit.
      current_position[X_AXIS] = 0;
      current_position[Y_AXIS] = 0;
      current_position[Z_AXIS] = 0;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      destination[X_AXIS] = 3 * Z_MAX_LENGTH;
      destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
      destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
      feedrate = 1.732 * homing_feedrate[X_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
      endstops_hit_on_purpose();

      current_position[X_AXIS] = destination[X_AXIS];
      current_position[Y_AXIS] = destination[Y_AXIS];
      current_position[Z_AXIS] = destination[Z_AXIS];

      // take care of back off and rehome now we are all at the top
      HOMEAXIS(X);
      HOMEAXIS(Y);
      HOMEAXIS(Z);

      calculate_delta(current_position);
      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

    enable_endstops(false);

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  previous_millis_cmd = millis();
  endstops_hit_on_purpose();
  mesh_active = mesh_was_active;   // the nozzle is where it is: only its Z in the corrected frame changes
  if (mesh_active) mesh_logical_z(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[Z_AXIS]);
  ui_busy(false);
}

void move_to(float x, float y, float z, float feed_mm_s)
{
  int saved = feedmultiply;
  float saved_feed = feedrate, saved_segments = delta_segments_per_second;
  feedmultiply = 100;
  feedrate = feed_mm_s * 60;
  delta_segments_per_second = PROBE_TRAVEL_SEGMENTS_PER_SECOND;   // a travel move does not have to follow its line closely
  destination[X_AXIS] = x;
  destination[Y_AXIS] = y;
  destination[Z_AXIS] = z;
  destination[E_AXIS] = current_position[E_AXIS];
  prepare_move();
  feedrate = saved_feed;
  feedmultiply = saved;
  delta_segments_per_second = saved_segments;
}

// An extruder only move, one block, exactly mm long whatever M221 says.
void move_e(float mm, float feed_mm_s)
{
  int saved = extrudemultiply;
  extrudemultiply = 100;
  float target = current_position[E_AXIS] + mm;
  calculate_delta(current_position);
  plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target, feed_mm_s, active_extruder);
  current_position[E_AXIS] = target;
  extrudemultiply = saved;
  previous_millis_cmd = millis();
}

void keep_steppers_on() { previous_millis_cmd = millis(); }

bool machine_idle() { return buflen == 0 && !blocks_queued(); }

void cancel_print()
{
  card.sdprinting = false;
  card.closefile();
  buflen = 0;
  bufindr = bufindw;
  quickStop();
  disable_heater();
  enquecommand_P(PSTR("G28"));
}

void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
  // The command letter is the first G, M or T of the line: a file name or a message can contain more of them
  // ("M23 SHAPE.GCO" is not a G-code)
  char *command = strpbrk(cmdbuffer[bufindr], "GMT");
  char letter = command ? *command : 0;
  if(letter == 'G' && code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
      }
      break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
      }
      break;
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
      }
      break;
    case 4: // G4 dwell
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis() < codenum) {
        manage_heater();
        manage_inactivity();
      }
      break;
    case 28: //G28 Home all Axis one at a time
      home_all_axes();
      break;

    case 29: // G29 [S0] - probe the bed mesh
      if(Stopped == false) probe_g29(!(code_seen('S') && code_value() == 0));
      break;

    case 30: // G30 - probe the bed at the current X and Y with the FSR
      if(Stopped == false) probe_g30();
      break;

    case 33: // G33 [F<3|4|6>] [P<7|10>] [S0] - delta calibration
      if(Stopped == false) {
        int factors = CALIB_DEFAULT_FACTORS, points = CALIB_DEFAULT_POINTS;
        if(code_seen('F')) factors = code_value();
        if(code_seen('P')) points = code_value();
        if((factors != 3 && factors != 4 && factors != 6) || (points != 7 && points != 10)) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("G33: F must be 3, 4 or 6 and P 7 or 10");
        } else {
          probe_g33(factors, points, !(code_seen('S') && code_value() == 0));
        }
      }
      break;

    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      // Only E can be set: a delta has no coordinate offsets, X, Y and Z are always the machine's
      if(code_seen(axis_codes[E_AXIS])) {
        current_position[E_AXIS] = code_value();
        plan_set_e_position(current_position[E_AXIS]);
      }
      break;
    default: // G29 and the other bed leveling codes are not implemented
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
      SERIAL_ECHO(cmdbuffer[bufindr]);
      SERIAL_ECHOLNPGM("\"");
      break;
    }
  }

  else if(letter == 'M' && code_seen('M'))
  {
    switch( (int)code_value() )
    {
    case 17:
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
      break;

    case 20: // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21: // M21 - init SD card

      card.initsd();

      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 32: //M32 - Select file and start SD print
    {
      if(card.sdprinting) {
        st_synchronize();

      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        card.openFile(namestartpos,true,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
        card.startFileprint();
        if(!call_procedure)
          starttime=millis(); //procedure calls count as normal print time.
      }
    } break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;


    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      }
      break;
    case 104: // M104
      if (code_seen('S')) setTargetHotend(code_value(), 0);
      break;
    case 112: //  M112 -Emergency Stop
      kill();
      break;
    case 105: // M105
      SERIAL_PROTOCOLPGM("ok T:");
      SERIAL_PROTOCOL_F(degHotend(0),1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(0),1);
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOL(getHeaterPower(0));
      SERIAL_PROTOCOLLN("");
      return;
    case 109: // M109 S<temp> (or R<temp>) - set the temperature and wait until it has been held for TEMP_RESIDENCY_TIME
    {
      if (code_seen('S') || code_seen('R')) setTargetHotend(code_value(), 0);
      codenum = millis();
      bool heating = isHeatingHotend(0);   // heating up or cooling down to the target
      cancel_heatup = false;               // M108 sets it
      long residencyStart = -1;
      while (!cancel_heatup && (residencyStart == -1 || (unsigned long)(millis() - residencyStart) < TEMP_RESIDENCY_TIME * 1000UL)) {
        if ((millis() - codenum) > 1000UL) {   // temperature and the seconds left, every second
          SERIAL_PROTOCOLPGM("T:");
          SERIAL_PROTOCOL_F(degHotend(0),1);
          SERIAL_PROTOCOLPGM(" E:0 W:");
          if (residencyStart > -1) SERIAL_PROTOCOLLN(((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL);
          else SERIAL_PROTOCOLLN("?");
          codenum = millis();
        }
        manage_heater();
        manage_inactivity();
        // start (or restart) the residency timer when the target is reached, or the temperature leaves the hysteresis
        if ((residencyStart == -1 &&  heating && (degHotend(0) >= (degTargetHotend(0)-TEMP_WINDOW))) ||
            (residencyStart == -1 && !heating && (degHotend(0) <= (degTargetHotend(0)+TEMP_WINDOW))) ||
            (residencyStart > -1 && labs(degHotend(0) - degTargetHotend(0)) > TEMP_HYSTERESIS))
          residencyStart = millis();
      }
      starttime = millis();
      previous_millis_cmd = millis();
    }
      break;
    case 117: // M117 display message: there is no display
    case 140: // M140 / M190: there is no heated bed
    case 190:
      break;

      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        break;

    case 80: // M80 - power supply on: there is no power supply control, accepted for host compatibility
      break;

    case 81: // M81 - all off
      disable_heater();
      finishAndDisableSteppers();
      fanSpeed = 0;
      break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          if(code_seen('E')) disable_e0();
        }
      }
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 114: // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
      break;
    case 119: // M119
    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      break;
      //TODO: update for all axis, use for loop
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only, X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) { add_homing[i] = code_value(); delta_calibrated = false; }
      }
      break;
	case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
		if(code_seen('L')) {
			delta_diagonal_rod= code_value();
		}
		if(code_seen('R')) {
			delta_radius= code_value();
		}
		if(code_seen('S')) {
			delta_segments_per_second= code_value();
		}
		if(code_seen('A')) delta_tower_angle[0] = code_value();  // tower angle corrections, degrees
		if(code_seen('B')) delta_tower_angle[1] = code_value();
		if(code_seen('C')) delta_tower_angle[2] = code_value();
		if(code_seen('L') || code_seen('R') || code_seen('A') || code_seen('B') || code_seen('C')) delta_calibrated = false;
		recalc_delta_settings(delta_radius, delta_diagonal_rod);
		break;
    case 666: // M666 set delta endstop adjustemnt
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) { endstop_adj[i] = code_value(); delta_calibrated = false; }
      }
      break;
    case 412: // M412 [S0|S1] - filament runout sensor on/off (until the next reset)
      if(code_seen('S')) filament_sensor_enable(code_value() != 0);
      filament_report();
      break;
    case 600: // M600 - filament change
      if(Stopped == false) filament_change(false);
      break;
    case 701: // M701 and M24 go on when a filament change waits, otherwise there is nothing to load
      break;
    case 420: // M420 [S0|S1] [Z<fade height>] - bed mesh on/off, height where its correction has faded out
      if(code_seen('Z')) {
        float fade = code_value();
        if(fade >= 0 && fade <= 100) {
          bool was = mesh_active;
          mesh_set_active(false);
          mesh_fade_height = fade;
          mesh_set_active(was);
        }
      }
      if(code_seen('S')) {
        if(code_value() != 0 && !mesh_have) { SERIAL_ERROR_START; SERIAL_ERRORLNPGM("M420: there is no mesh, run G29"); }
        else mesh_set_active(code_value() != 0);
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("Bed mesh: ", (unsigned long)mesh_active);
      SERIAL_ECHOPAIR(" (have one: ", (unsigned long)mesh_have);
      SERIAL_ECHOPAIR("), fade height ", mesh_fade_height);
      SERIAL_ECHOLN("");
      break;
    case 851: // M851 Z<mm> - how high above the bed the nozzle is when the probe fires
      if(code_seen('Z')) {
        float z = code_value();
        if(z >= -PROBE_MAX_OFFSET_MM && z <= PROBE_MAX_OFFSET_MM) probe_z_offset = z;
        else { SERIAL_ERROR_START; SERIAL_ERRORLNPGM("M851: Z must be within +-5 mm"); }
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("Probe Z offset: ", probe_z_offset);
      SERIAL_ECHOLN("");
      break;
    case 974: // M974 S<counts> - FSR sensitivity
      if(code_seen('S') && !probe_set_sensitivity(code_value())) {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM("M974: S must be 1..1023");
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPAIR("FSR sensitivity: ", (unsigned long)fsr_sensitivity);
      SERIAL_ECHOPAIR(" counts, reading now: ", (unsigned long)fsr_raw);   // watch it while pressing on the bed
      SERIAL_ECHOLN("");
      break;
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S') && code_value() > 0)  // 0 would divide by zero in prepare_move()
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S') && code_value() > 0) extrudemultiply = code_value();
    }
    break;

    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(Ki));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(Kd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
    case 303: // M303 [S<temperature>] [C<cycles>] PID autotune
    {
      float temp = 150.0;
      int c = 5;
      if (code_seen('S')) temp = code_value();
      if (code_seen('C')) c = code_value();
      PID_autotune(temp, c);
    }
    break;
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;




    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;


   case 999: // M999: Restart after being stopped
      Stopped = false;
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
    }
  }

  else if(letter == 'T')
  {
    // there is one extruder: T0 is accepted, nothing to select
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  if(fromsd[bufindr])
    return;
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
    }
    else destination[i] = current_position[i];
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
   get_coordinates();

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    
    float negative_z_offset = 0;
    
    if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];

    // the printable area is a circle
    float r2 = sq(target[X_AXIS]) + sq(target[Y_AXIS]);
    if (r2 > sq(DELTA_PRINTABLE_RADIUS)) {
      float scale = DELTA_PRINTABLE_RADIUS / sqrt(r2);
      target[X_AXIS] *= scale;
      target[Y_AXIS] *= scale;
    }
  }
}

void recalc_delta_settings(float radius, float diagonal_rod)
{
  const float d2r = 0.017453292519943295;
  delta_tower1_x = radius * cos((210 + delta_tower_angle[0]) * d2r); // front left tower
  delta_tower1_y = radius * sin((210 + delta_tower_angle[0]) * d2r);
  delta_tower2_x = radius * cos((330 + delta_tower_angle[1]) * d2r); // front right tower
  delta_tower2_y = radius * sin((330 + delta_tower_angle[1]) * d2r);
  delta_tower3_x = radius * cos((90 + delta_tower_angle[2]) * d2r);  // back middle tower
  delta_tower3_y = radius * sin((90 + delta_tower_angle[2]) * d2r);
  delta_diagonal_rod_2 = sq(diagonal_rod);
}

// True if the arms can reach the position: every rod needs a positive vertical part, otherwise calculate_delta()
// would take the square root of a negative number. The reachable area is convex, so the end point of a
// straight move is enough to know that the whole move is reachable.
bool delta_reachable(const float cartesian[3])
{
  return delta_diagonal_rod_2 - sq(delta_tower1_x-cartesian[X_AXIS]) - sq(delta_tower1_y-cartesian[Y_AXIS]) > 0 &&
         delta_diagonal_rod_2 - sq(delta_tower2_x-cartesian[X_AXIS]) - sq(delta_tower2_y-cartesian[Y_AXIS]) > 0 &&
         delta_diagonal_rod_2 - sq(delta_tower3_x-cartesian[X_AXIS]) - sq(delta_tower3_y-cartesian[Y_AXIS]) > 0;
}

void calculate_delta(float cartesian[3])
{
  float z = cartesian[Z_AXIS];
  if (mesh_active) z = mesh_machine_z(cartesian[X_AXIS], cartesian[Y_AXIS], z);   // follow the bed
  delta[X_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower1_x-cartesian[X_AXIS])
                       - sq(delta_tower1_y-cartesian[Y_AXIS])
                       ) + z;
  delta[Y_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower2_x-cartesian[X_AXIS])
                       - sq(delta_tower2_y-cartesian[Y_AXIS])
                       ) + z;
  delta[Z_AXIS] = sqrt(delta_diagonal_rod_2
                       - sq(delta_tower3_x-cartesian[X_AXIS])
                       - sq(delta_tower3_y-cartesian[Y_AXIS])
                       ) + z;
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
}

void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();

  if (!delta_reachable(destination)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_OUT_OF_REACH);
    memcpy(destination, current_position, sizeof(destination));
    return;
  }

  float difference[NUM_AXIS];
  for (int8_t i=0; i < NUM_AXIS; i++) {
    difference[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                            sq(difference[Y_AXIS]) +
                            sq(difference[Z_AXIS]));
  if (difference[E_AXIS] > 0.0001 && axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])
    filament_note_extrusion();   // from now on running out of filament matters
  if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
  long steps = max(1L, long(delta_segments_per_second * seconds));
  for (long s = 1; s <= steps; s++) {
    float fraction = float(s) / float(steps);
    for(int8_t i=0; i < NUM_AXIS; i++) {
      destination[i] = current_position[i] + difference[i] * fraction;
    }
    calculate_delta(destination);
    // the segment length is given to the planner: the feed rate is the speed of the nozzle
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                     destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                     active_extruder, cartesian_mm/steps);
  }
  
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}


void manage_inactivity(bool ignore_stepper_queue/*=false*/) //default argument set in Marlin.h
{
	
	
  if(buflen < (BUFSIZE-1))
    get_command();

  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false && ignore_stepper_queue == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();

      }
    }
  }
  
  fan_service();
  filament_sensor_poll();
  ui_poll();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();
  analogWrite(FAN_PIN, 255);   // cool what is hot
  #if FAN_DUAL
    WRITE(FAN_SAFETY_PIN, HIGH);
  #endif

  disable_x();
  disable_y();
  disable_z();
  disable_e0();


  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  
  cli();   // disable interrupts
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
  }
}

bool IsStopped() { return Stopped; };
