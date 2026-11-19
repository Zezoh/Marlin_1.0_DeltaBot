#ifndef PINS_H
#define PINS_H

#include "boards.h"

/****************************************************************************************
* RAMPS pin assignments (Arduino Mega 1280 / 2560)
****************************************************************************************/
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// Hardware SPI pins of the ATmega1280/2560, used by the SD card and the MAX6675 thermocouple
#define SS_PIN           53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
// Do not use pin 53 or 49 for the MAX6675: they are used by the SD card socket
#define MAX6675_SS       66

#define SDSS             53
#define LED_PIN          13

#if !MB(RAMPS_OLD)
  //---------------------------------------------------------------------------------
  // RAMPS 1.3 / 1.4
  //---------------------------------------------------------------------------------
  #define X_STEP_PIN         54
  #define X_DIR_PIN          55
  #define X_ENABLE_PIN       38
  #define X_MIN_PIN           3
  #define X_MAX_PIN           2

  #define Y_STEP_PIN         60
  #define Y_DIR_PIN          61
  #define Y_ENABLE_PIN       56
  #define Y_MIN_PIN          14
  #define Y_MAX_PIN          15

  #define Z_STEP_PIN         46
  #define Z_DIR_PIN          48
  #define Z_ENABLE_PIN       62
  #define Z_MIN_PIN          18
  #define Z_MAX_PIN          19

  #define E0_STEP_PIN        26
  #define E0_DIR_PIN         28
  #define E0_ENABLE_PIN      24

  #define E1_STEP_PIN        36
  #define E1_DIR_PIN         34
  #define E1_ENABLE_PIN      30

  #define SDPOWER            -1
  #define PS_ON_PIN          12
  #define KILL_PIN           -1

  // Power outputs: D8 (bed), D9, D10
  #if MB(RAMPS_13_EFB) || MB(RAMPS_13_EFF)
    #define FAN_PIN           9
  #elif MB(RAMPS_13_EEB)
    #define FAN_PIN           4  // IO pin. Buffer needed
  #else  // RAMPS_13_EEF
    #define FAN_PIN           8
  #endif

  #if MB(RAMPS_13_EFF)
    #define HEATER_0_PIN      8
    #define HEATER_1_PIN      9
    #define HEATER_BED_PIN   -1  // no bed
  #elif MB(RAMPS_13_EEF)
    #define HEATER_0_PIN     10
    #define HEATER_1_PIN      9
    #define HEATER_BED_PIN   -1  // no bed
  #elif MB(RAMPS_13_EFB)
    #define HEATER_0_PIN     10
    #define HEATER_1_PIN     -1
    #define HEATER_BED_PIN    8
  #else  // RAMPS_13_EEB
    #define HEATER_0_PIN     10
    #define HEATER_1_PIN      9
    #define HEATER_BED_PIN    8
  #endif

  #define TEMP_0_PIN         13  // ANALOG NUMBERING
  #define TEMP_1_PIN         15  // ANALOG NUMBERING
  #define TEMP_BED_PIN       14  // ANALOG NUMBERING

  #if MB(RAMPS_13_EEB) && defined(FILAMENT_SENSOR)
    // analog pin for the filament width sensor: RAMPS 1.4 analog input 5 on the AUX2 connector
    #define FILWIDTH_PIN      5
  #endif

  #ifdef NUM_SERVOS
    #define SERVO0_PIN       11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN      6
    #endif
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      5
    #endif
    #if NUM_SERVOS > 3
      #define SERVO3_PIN      4
    #endif
  #endif

#else
  //---------------------------------------------------------------------------------
  // RAMPS up to 1.2
  //---------------------------------------------------------------------------------
  #define X_STEP_PIN         26
  #define X_DIR_PIN          28
  #define X_ENABLE_PIN       24
  #define X_MIN_PIN           3
  #define X_MAX_PIN          -1  // 2

  #define Y_STEP_PIN         38
  #define Y_DIR_PIN          40
  #define Y_ENABLE_PIN       36
  #define Y_MIN_PIN          16
  #define Y_MAX_PIN          -1  // 17

  #define Z_STEP_PIN         44
  #define Z_DIR_PIN          46
  #define Z_ENABLE_PIN       42
  #define Z_MIN_PIN          18
  #define Z_MAX_PIN          -1  // 19

  #define E0_STEP_PIN        32
  #define E0_DIR_PIN         34
  #define E0_ENABLE_PIN      30

  #define SDPOWER            48
  #define PS_ON_PIN          -1
  #define KILL_PIN           -1

  // uncomment for RAMPS v1.0
  // #define RAMPS_V_1_0
  #ifdef RAMPS_V_1_0
    #define HEATER_0_PIN     12
    #define HEATER_BED_PIN   -1
    #define FAN_PIN          11
  #else  // RAMPS 1.1 / 1.2
    #define HEATER_0_PIN     10
    #define HEATER_BED_PIN    8
    #define FAN_PIN           9
  #endif
  #define HEATER_1_PIN       -1

  #define TEMP_0_PIN          2  // ANALOG NUMBERING
  #define TEMP_1_PIN         -1
  #define TEMP_BED_PIN        1  // ANALOG NUMBERING
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
  #undef X_MAX_PIN
  #undef Y_MAX_PIN
  #undef Z_MAX_PIN
  #define X_MAX_PIN          -1
  #define Y_MAX_PIN          -1
  #define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
  #undef X_MIN_PIN
  #undef Y_MIN_PIN
  #undef Z_MIN_PIN
  #define X_MIN_PIN          -1
  #define Y_MIN_PIN          -1
  #define Z_MIN_PIN          -1
#endif

// List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }

#endif //__PINS_H
