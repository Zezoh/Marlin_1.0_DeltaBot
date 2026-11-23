#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================

#ifdef BED_LIMIT_SWITCHING
  #define BED_HYSTERESIS 2 //only disable heating if T>target+BED_HYSTERESIS and enable heating if T>target-BED_HYSTERESIS
#endif
#define BED_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

//automatic temperature: The hot end target temperature is calculated by all the buffered lines of gcode.
//The maximum buffered steps/sec of the extruder motor are called "se".
//You enter the autotemp mode by a M109 S<mintemp> T<maxtemp> F<factor>
// the target temperature is set to mintemp+factor*se[steps/sec] and limited by mintemp and maxtemp
// you exit the value by any M109 without F*
// Also, if the temperature is set to a value <mintemp, it is not changed by autotemp.
// on an ultimaker, some initial testing worked with M109 S215 B260 F1 in the start.gcode
#define AUTOTEMP
#ifdef AUTOTEMP
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

//These defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
#define CONTROLLERFAN_PIN -1 //Pin used for the fan to cool controller (-1 to disable)
#define CONTROLLERFAN_SECS 60 //How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED 255  // == full speed

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//#define FAN_KICKSTART_TIME 100

// Extruder cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// extruder temperature is above/below EXTRUDER_AUTO_FAN_TEMPERATURE.
// Multiple extruders can be assigned to the same pin in which case 
// the fan will turn on when any selected extruder is above the threshold.
#define EXTRUDER_0_AUTO_FAN_PIN   -1
#define EXTRUDER_1_AUTO_FAN_PIN   -1
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED   255  // == full speed

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_RETRACT_MM 5 
#define Y_HOME_RETRACT_MM 5 
#define Z_HOME_RETRACT_MM 5 // deltas need the same for all three axis

#define AXIS_RELATIVE_MODES {false, false, false, false}

#define MAX_STEP_FREQUENCY 40000 // Max step frequency (5000 pps / half step)

//By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

//default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 0 // 0 = never: released carriages would drop the nozzle onto the bed (M84 S<seconds> sets it)

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define SD_FINISHED_STEPPERRELEASE false // a delta must keep its carriages held: released they drop the nozzle onto the bed
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.

// The hardware watchdog should reset the Microcontroller disabling all outputs, in case the firmware gets stuck and doesn't do temperature regulation.
#define USE_WATCHDOG

#ifdef USE_WATCHDOG
// If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
// The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
//  However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
//#define WATCHDOG_RESET_MANUAL
#endif

// Enable the option to stop SD printing when hitting and endstops, needs to be enabled when this option is enabled.
//#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

#define dropsegments 5 //everything with less than this number of steps will be ignored as move and joined with the next movement

//===========================================================================
//=============================Z probe and calibration=======================
//===========================================================================

// The Z probe is a force sensing resistor (FSR) under the bed on analog input FSR_PIN: the nozzle itself is the probe.
// A probe stroke ends when the reading differs from its value at the start of the stroke by FSR_SENSITIVITY counts.
// M974 changes it at run time; a noisy sensor raises it automatically.
#define FSR_SENSITIVITY 12       // ADC counts, 1..1023

#define PROBE_FEED_MM_S 5.0            // speed of the last PROBE_SLOW_APPROACH_MM of every stroke: contact is detected here
#define PROBE_FEED_FAST_MM_S 45.0      // speed of the approach before that
#define PROBE_SLOW_APPROACH_MM 4.0
#define PROBE_RETRACT_FEED_MM_S 25.0   // lift after contact
#define PROBE_TRAVEL_FEED_MM_S 150.0   // moves between probe points
#define PROBE_TRAVEL_SEGMENTS_PER_SECOND 40 // travel moves need no 200 segments/s: fewer segments keep the planner ahead of the steppers
#define PROBE_TRAVEL_Z_MM 30.0         // height of those moves until the bed height is known
#define PROBE_CLEARANCE_MM 5.0         // then: this far above the highest contact found in the pass
#define PROBE_TRAVEL_Z_FEED_MM_S 80.0  // vertical moves between points
#define PROBE_MIN_STROKE_MM 0.3        // a contact sooner than this after a stroke started is vibration, not the bed
#define PROBE_SETTLE_MS 100            // wait after a move before the probe is armed
#define PROBE_AVERAGE_SAMPLES 3        // strokes in agreement that make a probed point
#define PROBE_AVERAGE_TOLERANCE_MM 0.05
#define PROBE_AVERAGE_MAX_STROKES 9    // give up waiting for agreement after this many strokes
#define PROBE_FALSE_TRIGGER_MM 5.0     // a contact this far above the deepest one seen is noise, not the bed
#define PROBE_MAX_OFFSET_MM 5.0        // M851 range

// G33 delta calibration
#define CALIB_DEFAULT_FACTORS 6        // endstops, delta radius, tower angles A and B (F3 = endstops, F4 = + radius)
#define CALIB_DEFAULT_POINTS 10        // P7 is the quick form
#define CALIB_VERIFY_RMS_MM 0.05       // a pass this flat ends the run: calibrated
#define CALIB_ACCEPT_RMS_MM 0.25       // a run that stops improving is accepted if its last pass is this flat
#define CALIB_PLATEAU_RATIO 0.7        // "stops improving": a pass is not below this fraction of the previous one
#define CALIB_MAX_PASSES 6             // probe, solve, re-home, verify: at most this many probe passes
#define CALIB_FIRST_PASS_DEPTH_MM 10.0 // how far below the expected bed the first pass looks for it
#define CALIB_PASS_DEPTH_MM 5.0
#define CALIB_POINT_DEPTH_MM 6.0       // after the first point of a pass, how far below the previous point's height to look
#define CALIB_PROBE_RADIUS_MM 75.0     // the outer ring of probe points (the inner ring is half of it): inside the bed edge
#define CALIB_MAX_RADIUS_DELTA_MM 15.0 // the solution may move the delta radius by this much from where G33 started,
#define CALIB_MAX_HEIGHT_DELTA_MM 20.0 // the home height by this much,
#define CALIB_MAX_TRIM_MM 15.0         // an endstop trim up to this much,
#define CALIB_MAX_ANGLE_DEG 2.0        // a tower angle correction up to this much; more is refused and the old values restored

//===========================================================================
//=============================One button and status LED=====================
//===========================================================================
// Button on BUTTON_PIN (active low, pullup). Short press: idle and not calibrated: G33; idle: print the newest
// file on the SD card; printing: pause; paused: resume. Long press: cancel the print or the calibration, heater off.
// Held at power-on: calibrate (G33) once. The LED on STATUS_LED_PIN blinks the state, the faster the more urgent.
#define BUTTON_DEBOUNCE_MS 40
#define BUTTON_LONG_MS 1500
#define BUTTON_BOOT_HOLD_MS 2500
#define LED_ERROR_PERIOD_MS 200
#define LED_BUSY_PERIOD_MS 800
#define LED_PAUSED_PERIOD_MS 1000
#define LED_PRINT_PERIOD_MS 1200
#define LED_HEAT_PERIOD_MS 2000
#define LED_COOL_PERIOD_MS 3000
#define LED_READY_PERIOD_MS 4000
#define LED_COOL_TEMP 50      // the hotend is "cooling" above this temperature (degC)

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.  
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#define BLOCK_BUFFER_SIZE 16

//The ASCII buffer for recieving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

//===========================================================================
//=============================  Define Defines  ============================
//===========================================================================

#if EXTRUDERS > 1 && defined TEMP_SENSOR_1_AS_REDUNDANT
  #error "You cannot use TEMP_SENSOR_1_AS_REDUNDANT if EXTRUDERS > 1"
#endif

#if TEMP_SENSOR_0 > 0
  #define THERMISTORHEATER_0 TEMP_SENSOR_0
  #define HEATER_0_USES_THERMISTOR
#endif
#if TEMP_SENSOR_1 > 0
  #define THERMISTORHEATER_1 TEMP_SENSOR_1
  #define HEATER_1_USES_THERMISTOR
#endif
#if TEMP_SENSOR_BED > 0
  #define THERMISTORBED TEMP_SENSOR_BED
  #define BED_USES_THERMISTOR
#endif
#if TEMP_SENSOR_0 == -1
  #define HEATER_0_USES_AD595
#endif
#if TEMP_SENSOR_1 == -1
  #define HEATER_1_USES_AD595
#endif
#if TEMP_SENSOR_BED == -1
  #define BED_USES_AD595
#endif
#if TEMP_SENSOR_0 == -2
  #define HEATER_0_USES_MAX6675
#endif
#if TEMP_SENSOR_0 == 0
  #undef HEATER_0_MINTEMP
  #undef HEATER_0_MAXTEMP
#endif
#if TEMP_SENSOR_1 == 0
  #undef HEATER_1_MINTEMP
  #undef HEATER_1_MAXTEMP
#endif
#if TEMP_SENSOR_BED == 0
  #undef BED_MINTEMP
  #undef BED_MAXTEMP
#endif

#endif //__CONFIGURATION_ADV_H
