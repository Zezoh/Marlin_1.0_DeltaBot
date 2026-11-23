#ifndef PINS_H
#define PINS_H

/****************************************************************************************
* RAMPS pin assignments (Arduino Mega 1280 / 2560)
****************************************************************************************/
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// Hardware SPI pins of the ATmega1280/2560, used by the SD card
#define SS_PIN           53
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51

#define SDSS             53

//---------------------------------------------------------------------------------
// RAMPS 1.3 / 1.4
//---------------------------------------------------------------------------------
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MAX_PIN          19

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

// One button (D3, the RAMPS X-MIN header, active low with pullup) and status LED (D12, the RAMPS PS_ON header)
#define BUTTON_PIN          3
#define STATUS_LED_PIN     12

// Filament runout sensor on D14, the RAMPS Y-MIN socket (the delta only uses the MAX endstops)
#define RUNOUT_PIN         14

// FSR (force sensing resistor) Z probe analog input A14 (the RAMPS bed thermistor input)
#define FSR_PIN            14  // ANALOG NUMBERING

// Power outputs: D10 hotend heater, D9 fan. D8 (the bed output) is not used: there is no heated bed.
#define FAN_PIN            9
#define FAN_SAFETY_PIN     8   // the on/off safety fan of FAN_DUAL (the RAMPS bed output)
#define HEATER_0_PIN      10
#define HEATER_BED_PIN    -1

#define TEMP_0_PIN         13  // ANALOG NUMBERING
#define TEMP_BED_PIN       -1  // A14 is the FSR input

#endif //__PINS_H
