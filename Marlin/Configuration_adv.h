#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Fan: FAN_DUAL 0 = one fan on FAN_PIN cools the print and the hotend (it never runs below FAN_MIN_PWM while the hotend is
// hot while printing, and at full power while it cools down idle); FAN_DUAL 1 = an on/off safety fan on FAN_SAFETY_PIN
// for the hotend and a print fan on FAN_PIN that only does what M106/M107 say. See fan.cpp.
#ifndef FAN_DUAL
#define FAN_DUAL 0
#endif
#define FAN_SAFETY_ON_C 50.0       // the safety fan runs from here...
#define FAN_SAFETY_OFF_C 49.0      // ...until the hotend and its target are below this
#define FAN_MIN_PWM 120            // one fan: the least it does while printing and hot
#define FAN_KICK_MS 400            // full power for this long when a fan starts
#define FAN_PRINTING_HOLD_MS 3000  // after the last move or command: still "printing" for this long

#define ENDSTOPS_ONLY_FOR_HOMING  // If defined the endstops will only be used for homing

// homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_RETRACT_MM 5
#define Y_HOME_RETRACT_MM 5
#define Z_HOME_RETRACT_MM 5  // deltas need the same for all three axis

#define AXIS_RELATIVE_MODES    \
  {                            \
    false, false, false, false \
  }

#define MAX_STEP_FREQUENCY 40000  // Max step frequency (5000 pps / half step)

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false

// default stepper release if idle
#define DEFAULT_STEPPER_DEACTIVE_TIME 0  // 0 = never: released carriages would drop the nozzle onto the bed (M84 S<seconds> sets it)

#define DEFAULT_MINIMUMFEEDRATE 0.0  // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE 0.0

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05  // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define SD_FINISHED_STEPPERRELEASE false          // a delta must keep its carriages held: released they drop the nozzle onto the bed
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E"  // You might want to keep the z enabled so your bed stays in place.

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

#define dropsegments 5  // everything with less than this number of steps will be ignored as move and joined with the next movement

//===========================================================================
//=============================Z probe and calibration=======================
//===========================================================================

// The Z probe is a force sensing resistor (FSR) under the bed on analog input FSR_PIN: the nozzle itself is the probe.
// A probe stroke ends when the reading differs from its value at the start of the stroke by FSR_SENSITIVITY counts.
// M974 changes it at run time; a noisy sensor raises it automatically.
#define FSR_SENSITIVITY 12  // ADC counts, 1..1023

#define PROBE_FEED_MM_S 5.0        // speed of the last PROBE_SLOW_APPROACH_MM of every stroke: contact is detected here
#define PROBE_FEED_FAST_MM_S 45.0  // speed of the approach before that
#define PROBE_SLOW_APPROACH_MM 4.0
#define PROBE_FAST_MIN_MM 3.0           // the fast approach is only used for a longer way down than this
#define PROBE_ACCELERATION_MM_S2 600.0  // acceleration and jerk while probing (printing values are restored afterwards)
#define PROBE_JERK_MM_S 12.0           // must stay above the speed change between two segments of a travel move (about 10 mm/s at 150 mm/s), or the planner brakes at every segment
#define PROBE_RETRACT_FEED_MM_S 12.0         // lift after contact
#define PROBE_TRAVEL_FEED_MM_S 150.0         // moves between probe points
#define PROBE_TRAVEL_SEGMENTS_PER_SECOND 100 // a travel move is cut into pieces: at 150 mm/s 40 pieces/s make the towers change speed by up to 10 mm/s 40 times a second (a rough buzz), 100 pieces/s about 4 mm/s
#define PROBE_TRAVEL_Z_MM 30.0               // height of those moves until the bed height is known
#define PROBE_CLEARANCE_MM 5.0               // then: this far above the highest contact found in the pass
#define PROBE_TRAVEL_Z_FEED_MM_S 80.0        // vertical moves between points
#define PROBE_MIN_TRAVEL_ABOVE_START_MM 1.0  // the travel height is at least this far above where the slow approach begins
#define PROBE_EARLY_CONTACT_EXTRA_MM 1.0     // after a contact during the fast approach: lift to this far above the approach start
#define PROBE_FALSE_LIFT_MM 1.0              // lift after a contact that was vibration or noise
#define PROBE_LIFT_MM 1.0                    // lift after every real contact, before the next stroke
#define PROBE_MIN_STROKE_MM 0.3              // a contact sooner than this after a stroke started is vibration, not the bed
#define PROBE_SETTLE_MS 100                  // wait after a move before the probe is armed
#define PROBE_AVERAGE_SAMPLES 3              // strokes in agreement that make a probed point
#define PROBE_AVERAGE_TOLERANCE_MM 0.05
#define PROBE_AVERAGE_MAX_STROKES 9  // give up waiting for agreement after this many strokes
#define PROBE_FALSE_TRIGGER_MM 5.0   // a contact this far above the deepest one seen is noise, not the bed
#define PROBE_MAX_OFFSET_MM 5.0      // M851 range

// G33 delta calibration
#define CALIB_DEFAULT_FACTORS 6         // endstops, delta radius, tower angles A and B (F3 = endstops, F4 = + radius)
#define CALIB_DEFAULT_POINTS 10         // P7 is the quick form
#define CALIB_VERIFY_RMS_MM 0.05        // a pass this flat ends the run: calibrated
#define CALIB_ACCEPT_RMS_MM 0.25        // a run that stops improving is accepted if its last pass is this flat
#define CALIB_PLATEAU_RATIO 0.7         // "stops improving": a pass is not below this fraction of the previous one
#define CALIB_MAX_PASSES 6              // probe, solve, re-home, verify: at most this many probe passes
#define CALIB_FIRST_PASS_DEPTH_MM 10.0  // how far below the expected bed the first pass looks for it
#define MESH_MAX_MM 5.0                 // G29 refuses a bed that is further than this from Z = 0: run G33 first
#define CALIB_PASS_DEPTH_MM 5.0
#define CALIB_POINT_DEPTH_MM 6.0        // after the first point of a pass, how far below the previous point's height to look
#define CALIB_PROBE_RADIUS_MM 75.0      // the outer ring of probe points (the inner ring is half of it): inside the bed edge
#define CALIB_MAX_RADIUS_DELTA_MM 15.0  // the solution may move the delta radius by this much from where G33 started,
#define CALIB_MAX_HEIGHT_DELTA_MM 20.0  // the home height by this much,
#define CALIB_MAX_TRIM_MM 15.0          // an endstop trim up to this much,
#define CALIB_MAX_ANGLE_DEG 2.0         // a tower angle correction up to this much; more is refused and the old values restored

//===========================================================================
//=============================One button and status LED=====================
//===========================================================================
// Button on BUTTON_PIN (active low, pullup). Short press: idle and not calibrated: G33; idle: print the newest
// file on the SD card; printing: pause; paused: resume. Long press: cancel the print or the calibration, heater off.
// Held at power-on: calibrate (G33) once. The LED on STATUS_LED_PIN blinks the state, the faster the more urgent.
#define BUTTON_DEBOUNCE_MS 40
#define BUTTON_DOUBLE_MS 350  // two clicks within this are a double click; a single click is reported after it
#define BUTTON_LONG_MS 1500
#define BUTTON_BOOT_HOLD_MS 2500
#define LED_ERROR_PERIOD_MS 200
#define LED_CHANGE_PERIOD_MS 400  // a filament change works
#define LED_WAIT_PERIOD_MS 1000   // ... and waits for you
#define LED_BUSY_PERIOD_MS 800
#define LED_PAUSED_PERIOD_MS 1000
#define LED_PRINT_PERIOD_MS 1200
#define LED_HEAT_PERIOD_MS 2000
#define LED_COOL_PERIOD_MS 3000
#define LED_READY_PERIOD_MS 4000
#define LED_COOL_TEMP 50  // the hotend is "cooling" above this temperature (degC)

//===========================================================================
//=============================Filament change (M600) and runout=============
//===========================================================================
// M600 pauses the print, parks the nozzle, unloads the filament, waits for the new one, loads it, purges, waits for a
// confirmation and goes back to the print. The runout sensor (RUNOUT_PIN) starts the same sequence by itself.
// Press the button (or send M701 / M24) to go on, hold it (or M108) to cancel.
// A click while printing from the SD card does the same park without the filament change: the print waits, the nozzle
// goes away from the part, a click (or M24) brings it back and the print goes on.
#define RUNOUT_ENABLED 1      // M412 S0/S1 changes it until the next reset
#define RUNOUT_ACTIVE_HIGH 1  // the pin reads HIGH when there is no filament
#define RUNOUT_PULLUP 1
#define RUNOUT_DEBOUNCE_MS 500           // "no filament" must last this long
#define FILAMENT_INSERT_DEBOUNCE_MS 750  // "filament inserted" must last this long

#define BOWDEN_LENGTH_MM 740.0        // extruder to hotend
#define FILAMENT_CHANGE_TEMP_C 210.0  // the nozzle is heated to the higher of this and the printing temperature
// The park position is worked out from the size of the printer: X0, on the front side (Y negative), at this percentage
// of the printable radius, and never lower than this percentage of the height (Z_MAX_POS) above the bed.
#define FILAMENT_PARK_RADIUS_PERCENT 80
#define FILAMENT_PARK_MIN_Z_PERCENT 15
#define FILAMENT_Z_LIFT 5.0         // the nozzle is lifted at least this much before parking
#define FILAMENT_TRAVEL_SPEED 50.0  // mm/s
#define FILAMENT_Z_SPEED 10.0
#define FILAMENT_RETRACT_MM 3.0  // pulled back before parking, pushed forward again on return
#define FILAMENT_RETRACT_SPEED 25.0
#define FILAMENT_UNLOAD_EXTRA_MM 5.0  // unload = bowden length + this
#define FILAMENT_UNLOAD_SPEED 30.0
#define FILAMENT_SLOW_LOAD_MM 10.0  // first the filament is fed slowly into the extruder,
#define FILAMENT_SLOW_LOAD_SPEED 5.0
#define FILAMENT_FAST_LOAD_SPEED 30.0  // then the bowden length quickly,
#define FILAMENT_PURGE_MM 5.0          // then purged
#define FILAMENT_PURGE_SPEED 3.0
#define FILAMENT_E_CHUNK_MM 100.0  // longer extruder moves are cut into pieces of this length
#define FILAMENT_MIN_EXTRUDE_TEMP_C 170.0
#define FILAMENT_TEMP_TOLERANCE_C 3.0
#define FILAMENT_TEMP_HOLD_MS 1000  // the temperature must stay within the tolerance this long
#define FILAMENT_HEAT_TIMEOUT_MS 180000UL
#define FILAMENT_IDLE_TIMEOUT_MS 120000UL  // the heater is switched off after waiting this long for the user

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#define BLOCK_BUFFER_SIZE 16
#define PLAN_HOLD_BLOCKS 10  // a move from rest starts when this many blocks are queued, or when it is fully queued

// The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

#endif  //__CONFIGURATION_ADV_H
