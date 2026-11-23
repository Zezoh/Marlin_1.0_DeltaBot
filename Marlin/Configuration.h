#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Delta printer firmware: basic settings (board, delta geometry, temperature sensors, axis scaling, endstops).
// Advanced settings can be found in Configuration_adv.h

#include "Default_Version.h"

// Build information printed to the host at startup, to confirm that a new build was uploaded.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
#define BAUDRATE 250000

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
#define EXTRUDERS 1

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 200

// NOTE NB all values for DELTA_* values MUST be floating point, so always have a decimal point in them

// Center-to-center distance of the holes in the diagonal push rods.
#define DELTA_DIAGONAL_ROD 210.0 // mm

// Effective horizontal distance bridged by the diagonal push rods: tower circle radius minus the
// effector and carriage joint offsets. G33 refines it.
#define DELTA_RADIUS 90.0 // mm

// Radius of the circular printable area around the centre. Moves outside it are clamped.
#define DELTA_PRINTABLE_RADIUS 85.0 // mm

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
// Hotend temperature sensor, a thermistor with a 4.7k pullup (the table is in thermistortables.h):
// 1 is 100k EPCOS (Beta 4092), 5 is 100k ATC Semitec 104GT-2 (Beta 4267),
// 104 is 100k TDK/EPCOS 104NT-4-R025H42G (Beta 4125, the Creality/Sprite hotend sensor)
#define TEMP_SENSOR_0 104

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// Below MINTEMP the heater is switched off and the printer stops: a broken wire or an unplugged thermistor reads cold.
// Above MAXTEMP too: this is the limit for the hotend, not a protection against a shorted thermistor.
#define HEATER_0_MINTEMP 5
#define HEATER_0_MAXTEMP 285

// PID of the hotend: the heater is full on further than PID_FUNCTIONAL_RANGE below the target, off further above it
#define PID_MAX 255 // the most the PID may give the heater (255 = full power)
#define PID_FUNCTIONAL_RANGE 10
#define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
#define K1 0.95 //smoothing factor within the PID
#define PID_dT ((OVERSAMPLENR * 10.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

// Default PID constants of the hotend (run M303 to tune your own)
#define  DEFAULT_Kp 32.48
#define  DEFAULT_Ki 6.4
#define  DEFAULT_Kd 41.25

// No extrusion below this temperature, and none of more than this length in one move
#define EXTRUDE_MINTEMP 170
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH)

// Thermal runaway protection: after the target was reached once, the printer stops if the temperature stays more than
// HYSTERESIS below it for PERIOD seconds (a thermistor that came off reads cold, the heater would burn forever).
#define THERMAL_RUNAWAY_PROTECTION_PERIOD 40 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 4 // in degree Celsius
//===========================================================================


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// A delta homes the three towers up against the max endstops: there are no min endstops.
// Comment this out to disable the endstop pullup resistors. The pullups are needed if you
// directly connect a mechanical endswitch between the signal and ground pins.
#define ENDSTOPPULLUPS

// Set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = true;
const bool Y_MAX_ENDSTOP_INVERTING = true;
const bool Z_MAX_ENDSTOP_INVERTING = true;

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled

#define INVERT_X_DIR true // DELTA does not invert
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false    // for direct drive extruder v9 set to true, for geared extruder set to false

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define X_MIN_POS (-DELTA_PRINTABLE_RADIUS)
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS (-DELTA_PRINTABLE_RADIUS)
#define Z_MAX_POS Z_HOME_POS
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)


// Position of the nozzle after homing: top and center of the print volume
#define X_HOME_POS 0
#define Y_HOME_POS 0
#define Z_HOME_POS 225 // Distance between nozzle and print surface after homing.

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E

// delta homing speeds must be the same on xyz
#define HOMING_FEEDRATE {120*60, 120*60, 120*60, 0}  // fast seek towards the endstops (mm/min)
#define HOMING_SLOW_FEEDRATE (8*60) // precision re-approach of each endstop (mm/min): accuracy critical, do not raise

// default settings
// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 98}  // towers: 200 steps * 16 / (GT2 2 mm * 20 teeth); extruder 98
#define DEFAULT_MAX_FEEDRATE          {180, 180, 180, 30}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {8000,8000,8000,1000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          2500    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  2500   // X, Y, Z and E max acceleration in mm/s^2 for retracts

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 20.0    // (mm/sec) Must be same as XY for delta
#define DEFAULT_EJERK                 5.0    // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

//SD support
#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
