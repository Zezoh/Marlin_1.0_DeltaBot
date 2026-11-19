#ifndef PINS_H
#define PINS_H

#include "boards.h"

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1


/****************************************************************************************
* Arduino Mega pin assignment
*
****************************************************************************************/
#if IS_RAMPS || MB(AZTEEG_X3) || MB(AZTEEG_X3_PRO)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#if (IS_RAMPS && !MB(RAMPS_OLD)) || MB(AZTEEG_X3) || MB(AZTEEG_X3_PRO)

  #define LARGE_FLASH true

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

    #define Y2_STEP_PIN        36
    #define Y2_DIR_PIN         34
    #define Y2_ENABLE_PIN      30

    #define Z2_STEP_PIN        36
    #define Z2_DIR_PIN         34
    #define Z2_ENABLE_PIN      30

    #define E0_STEP_PIN        26
    #define E0_DIR_PIN         28
    #define E0_ENABLE_PIN      24

    #define E1_STEP_PIN        36
    #define E1_DIR_PIN         34
    #define E1_ENABLE_PIN      30

#if MB(RAMPS_13_EEB)  //FMM added for Filament Extruder
#ifdef FILAMENT_SENSOR
    //define analog pin for the filament width sensor input
    //Use the RAMPS 1.4 Analog input 5 on the AUX2 connector
      #define FILWIDTH_PIN        5
#endif
#endif

    #if MB(AZTEEG_X3_PRO)
      #define E2_STEP_PIN        23
      #define E2_DIR_PIN         25
      #define E2_ENABLE_PIN      40

      #define E3_STEP_PIN        27
      #define E3_DIR_PIN         29
      #define E3_ENABLE_PIN      41

      #define E4_STEP_PIN        43
      #define E4_DIR_PIN         37
      #define E4_ENABLE_PIN      42
    #endif

    #define SDPOWER            -1
    #define SDSS               53
    #define LED_PIN            13

  #if MB(RAMPS_13_EFB) || MB(RAMPS_13_EFF) || MB(AZTEEG_X3)
    #define FAN_PIN            9 // (Sprinter config)
  #elif MB(AZTEEG_X3_PRO)
    #define FAN_PIN            11 // Last Heater Pin on board
  #else
    #define FAN_PIN            4 // IO pin. Buffer needed
  #endif

  #if MB(RAMPS_13_EEF)
    #define FAN_PIN            8
  #endif

  #if MB(RAMPS_13_EFF)
    #define CONTROLLERFAN_PIN  -1 //Pin used for the fan to cool controller
  #endif

  #define PS_ON_PIN          12

  #if defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
    #define KILL_PIN           41
  #else
    #define KILL_PIN           -1
  #endif

  #if MB(RAMPS_13_EFF)
    #define HEATER_0_PIN       8
  #else
    #define HEATER_0_PIN       10   // EXTRUDER 1
  #endif

  #if MB(RAMPS_13_EFB) || MB(AZTEEG_X3)
    #define HEATER_1_PIN       -1
  #else
    #define HEATER_1_PIN       9    // EXTRUDER 2 (FAN On Sprinter)
  #endif

  #if   MB(AZTEEG_X3_PRO)
    #define HEATER_2_PIN       16
    #define HEATER_3_PIN       17
    #define HEATER_4_PIN       4
    #define HEATER_5_PIN       5
    #define HEATER_6_PIN       6
    #define HEATER_7_PIN       11
  #else
    #define HEATER_2_PIN       -1
  #endif

  #define TEMP_0_PIN         13   // ANALOG NUMBERING
  #define TEMP_1_PIN         15   // ANALOG NUMBERING
  #if MB(AZTEEG_X3_PRO)
    #define TEMP_2_PIN         12   // ANALOG NUMBERING
    #define TEMP_3_PIN         11   // ANALOG NUMBERING
    #define TEMP_4_PIN         10   // ANALOG NUMBERING
    #define TC1                4    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro
    #define TC2                5    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro
  #else
    #define TEMP_2_PIN         -1   // ANALOG NUMBERING
  #endif

  #if MB(RAMPS_13_EFF) || MB(RAMPS_13_EEF)
    #define HEATER_BED_PIN     -1    // NO BED
  #else
      #define HEATER_BED_PIN     8    // BED
  #endif

  #define TEMP_BED_PIN       14   // ANALOG NUMBERING

  #ifdef NUM_SERVOS
    #define SERVO0_PIN         11

    #if NUM_SERVOS > 1
      #define SERVO1_PIN         6
    #endif

    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
    #endif

    #if NUM_SERVOS > 3
      #define SERVO3_PIN         4
    #endif
  #endif

  #if MB(AZTEEG_X3_PRO)
    #define BEEPER 33
  #endif

  #ifdef TEMP_STAT_LEDS
    #if MB(AZTEEG_X3)
      #define STAT_LED_RED       6
      #define STAT_LED_BLUE     11
    #endif
  #endif

  #ifdef ULTRA_LCD

    #ifdef NEWPANEL
      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29

      #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
        #define BEEPER 37

        #define BTN_EN1 31
        #define BTN_EN2 33
        #define BTN_ENC 35

        #define SDCARDDETECT 49
      #elif defined(LCD_I2C_PANELOLU2)
        #define BTN_EN1 47  //reverse if the encoder turns the wrong way.
        #define BTN_EN2 43
        #define BTN_ENC 32
        #define LCD_SDSS 53
        #define SDCARDDETECT -1
        #define KILL_PIN 41
      #elif defined(LCD_I2C_VIKI)
        #define BTN_EN1 22  //reverse if the encoder turns the wrong way.
        #define BTN_EN2 7
        #define BTN_ENC -1
        #define LCD_SDSS 53
        #define SDCARDDETECT 49
      #else
        //arduino pin which triggers an piezzo beeper
        #define BEEPER 33  // Beeper on AUX-4

        //buttons are directly attached using AUX-2
        #ifdef REPRAPWORLD_KEYPAD
          #define BTN_EN1 64 // encoder
          #define BTN_EN2 59 // encoder
          #define BTN_ENC 63 // enter button
          #define SHIFT_OUT 40 // shift register
          #define SHIFT_CLK 44 // shift register
          #define SHIFT_LD 42 // shift register
        #else
          #define BTN_EN1 37
          #define BTN_EN2 35
          #define BTN_ENC 31  //the click
        #endif

        #ifdef G3D_PANEL
          #define SDCARDDETECT 49
        #else
          #define SDCARDDETECT -1  // Ramps does not use this port
        #endif

      #endif

    #else //old style panel with shift register
      //arduino pin witch triggers an piezzo beeper
      #define BEEPER 33   //No Beeper added

      //buttons are attached to a shift register
  // Not wired this yet
      //#define SHIFT_CLK 38
      //#define SHIFT_LD 42
      //#define SHIFT_OUT 40
      //#define SHIFT_EN 17

      #define LCD_PINS_RS 16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4 23
      #define LCD_PINS_D5 25
      #define LCD_PINS_D6 27
      #define LCD_PINS_D7 29
    #endif
  #endif //ULTRA_LCD

#else // RAMPS_V_1_1 or RAMPS_V_1_2 as default (BOARD_RAMPS_OLD)

#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define X_MIN_PIN           3
#define X_MAX_PIN          -1    //2

#define Y_STEP_PIN         38
#define Y_DIR_PIN          40
#define Y_ENABLE_PIN       36
#define Y_MIN_PIN          16
#define Y_MAX_PIN          -1    //17

#define Z_STEP_PIN         44
#define Z_DIR_PIN          46
#define Z_ENABLE_PIN       42
#define Z_MIN_PIN          18
#define Z_MAX_PIN          -1    //19

#define E0_STEP_PIN         32
#define E0_DIR_PIN          34
#define E0_ENABLE_PIN       30

#define SDPOWER            48
#define SDSS               53
#define LED_PIN            13
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#ifdef RAMPS_V_1_0 // RAMPS_V_1_0
  #define HEATER_0_PIN     12    // RAMPS 1.0
  #define HEATER_BED_PIN   -1    // RAMPS 1.0
  #define FAN_PIN          11    // RAMPS 1.0
#else // RAMPS_V_1_1 or RAMPS_V_1_2
  #define HEATER_0_PIN     10    // RAMPS 1.1
  #define HEATER_BED_PIN    8    // RAMPS 1.1
  #define FAN_PIN           9    // RAMPS 1.1
#endif
#define HEATER_1_PIN        -1
#define HEATER_2_PIN        -1
#define TEMP_0_PIN          2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_1_PIN          -1
#define TEMP_2_PIN          -1
#define TEMP_BED_PIN        1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!

#endif // RAMPS_13_EFB || RAMPS_13_EEB || RAMPS_13_EFF || 3DRAG

// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN          52
  #define MISO_PIN         50
  #define MOSI_PIN         51
  #define MAX6675_SS       66// Do not use pin 53 if there is even the remote possibility of using Dsplay/SD card
#else
  #define MAX6675_SS       66// Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
#endif

#endif // RAMPS_OLD || RAMPS_13_EFB || RAMPS_13_EEB || RAMPS_13_EFF || 3DRAG

/****************************************************************************************
* Elefu RA Board Pin Assignments
*
****************************************************************************************/
#if MB(ELEFU_3)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN         49
#define X_DIR_PIN          13
#define X_ENABLE_PIN       48
#define X_MIN_PIN          35
#define X_MAX_PIN          -1 //34

#define Y_STEP_PIN         11
#define Y_DIR_PIN          9
#define Y_ENABLE_PIN       12
#define Y_MIN_PIN          33
#define Y_MAX_PIN          -1 //32

#define Z_STEP_PIN         7
#define Z_DIR_PIN          6
#define Z_ENABLE_PIN       8
#define Z_MIN_PIN          31
#define Z_MAX_PIN          -1 //30

#define E2_STEP_PIN        43
#define E2_DIR_PIN         47
#define E2_ENABLE_PIN      42

#define E1_STEP_PIN        18
#define E1_DIR_PIN         19
#define E1_ENABLE_PIN      38

#define E0_STEP_PIN        40
#define E0_DIR_PIN         41
#define E0_ENABLE_PIN      37

#define SDPOWER            -1
#define LED_PIN            -1 //Use +12V Aux port for LED Ring

#define FAN_PIN            16 //5V PWM

#define PS_ON_PIN          10 //Set to -1 if using a manual switch on the PWRSW Connector
#define SLEEP_WAKE_PIN     26 //This feature still needs work

#define HEATER_0_PIN       45 //12V PWM1
#define HEATER_1_PIN       46 //12V PWM2
#define HEATER_2_PIN       17 //12V PWM3
#define HEATER_BED_PIN     44 //DOUBLE 12V PWM
#define TEMP_0_PIN         3  //ANALOG NUMBERING
#define TEMP_1_PIN         2  //ANALOG NUMBERING
#define TEMP_2_PIN         1  //ANALOG NUMBERING
#define TEMP_BED_PIN       0  //ANALOG NUMBERING

#define BEEPER             36

#define KILL_PIN           -1

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
#define PHOTOGRAPH_PIN     29

#ifdef RA_CONTROL_PANEL

  #define SDSS             53
  #define SDCARDDETECT     28

  #define BTN_EN1          14
  #define BTN_EN2          39
  #define BTN_ENC          15  //the click

  #define BLEN_C           2
  #define BLEN_B           1
  #define BLEN_A           0

  //encoder rotation values
  #define encrot0          0
  #define encrot1          2
  #define encrot2          3
  #define encrot3          1

#endif //RA_CONTROL_PANEL

#ifdef RA_DISCO
  //variables for which pins the TLC5947 is using
  #define TLC_CLOCK_PIN    25
  #define TLC_BLANK_PIN    23
  #define TLC_XLAT_PIN     22
  #define TLC_DATA_PIN     24

  //We also need to define pin to port number mapping for the 2560 to match the pins listed above. If you change the TLC pins, update this as well per the 2560 datasheet!
  //This currently only works with the RA Board.
  #define TLC_CLOCK_BIT 3 //bit 3 on port A
  #define TLC_CLOCK_PORT &PORTA //bit 3 on port A

  #define TLC_BLANK_BIT 1 //bit 1 on port A
  #define TLC_BLANK_PORT &PORTA //bit 1 on port A

  #define TLC_DATA_BIT 2 //bit 2 on port A
  #define TLC_DATA_PORT &PORTA //bit 2 on port A

  #define TLC_XLAT_BIT 0 //bit 0 on port A
  #define TLC_XLAT_PORT &PORTA //bit 0 on port A

  //change this to match your situation. Lots of TLCs takes up the arduino SRAM very quickly, so be careful
  //Leave it at at least 1 if you have enabled RA_LIGHTING
  //The number of TLC5947 boards chained together for use with the animation, additional ones will repeat the animation on them, but are not individually addressable and mimic those before them. You can leave the default at 2 even if you only have 1 TLC5947 module.
  #define NUM_TLCS 2

  //These TRANS_ARRAY values let you change the order the LEDs on the lighting modules will animate for chase functions.
  //Modify them according to your specific situation.
  //NOTE: the array should be 8 long for every TLC you have. These defaults assume (2) TLCs.
  #define TRANS_ARRAY {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8} //forwards
  //#define TRANS_ARRAY {7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15} //backwards
#endif //RA_LIGHTING

#endif // ELEFU_3

/****************************************************************************************
* RUMBA pin assignment
*
****************************************************************************************/
#if MB(RUMBA)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN         17
#define X_DIR_PIN          16
#define X_ENABLE_PIN       48
#define X_MIN_PIN          37
#define X_MAX_PIN          36

#define Y_STEP_PIN         54
#define Y_DIR_PIN          47
#define Y_ENABLE_PIN       55
#define Y_MIN_PIN          35
#define Y_MAX_PIN          34

#define Z_STEP_PIN         57
#define Z_DIR_PIN          56
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          33
#define Z_MAX_PIN          32

#define E0_STEP_PIN        23
#define E0_DIR_PIN         22
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        26
#define E1_DIR_PIN         25
#define E1_ENABLE_PIN      27

#define E2_STEP_PIN        29
#define E2_DIR_PIN         28
#define E2_ENABLE_PIN      39

#define LED_PIN            13

#define FAN_PIN            7
//additional FAN1 PIN (e.g. useful for electronics fan or light on/off) on PIN 8

#define PS_ON_PIN          45
#define KILL_PIN           46

#if (TEMP_SENSOR_0==0)
 #define TEMP_0_PIN         -1
 #define HEATER_0_PIN       -1
#else
 #define HEATER_0_PIN        2    // EXTRUDER 1
 #if (TEMP_SENSOR_0==-1)
  #define TEMP_0_PIN         6    // ANALOG NUMBERING - connector *K1* on RUMBA thermocouple ADD ON is used
 #else
  #define TEMP_0_PIN         15   // ANALOG NUMBERING - default connector for thermistor *T0* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_1==0)
 #define TEMP_1_PIN         -1
 #define HEATER_1_PIN       -1
#else
 #define HEATER_1_PIN        3    // EXTRUDER 2
 #if (TEMP_SENSOR_1==-1)
  #define TEMP_1_PIN         5    // ANALOG NUMBERING - connector *K2* on RUMBA thermocouple ADD ON is used
 #else
  #define TEMP_1_PIN         14   // ANALOG NUMBERING - default connector for thermistor *T1* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_2==0)
 #define TEMP_2_PIN         -1
 #define HEATER_2_PIN       -1
#else
 #define HEATER_2_PIN        6    // EXTRUDER 3
 #if (TEMP_SENSOR_2==-1)
  #define TEMP_2_PIN         7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_BED is defined as thermocouple
 #else
  #define TEMP_2_PIN         13   // ANALOG NUMBERING - default connector for thermistor *T2* on rumba board is used
 #endif
#endif

//optional for extruder 4 or chamber: #define TEMP_X_PIN         12   // ANALOG NUMBERING - default connector for thermistor *T3* on rumba board is used
//optional FAN1 can be used as 4th heater output: #define HEATER_3_PIN       8    // EXTRUDER 4

#if (TEMP_SENSOR_BED==0)
 #define TEMP_BED_PIN       -1
 #define HEATER_BED_PIN     -1
#else
 #define HEATER_BED_PIN      9    // BED
 #if (TEMP_SENSOR_BED==-1)
  #define TEMP_BED_PIN       7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_2 is defined as thermocouple
 #else
  #define TEMP_BED_PIN       11   // ANALOG NUMBERING - default connector for thermistor *THB* on rumba board is used
 #endif
#endif

#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

#endif // RUMBA

//
// SAV Mk-I
// -----------------------------------------------------------------------------------

/*****************************************************************
* Rambo Pin Assignments
******************************************************************/
#if MB(RAMBO)
#define KNOWN_BOARD
#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define X_STEP_PIN 37
#define X_DIR_PIN 48
#define X_MIN_PIN 12
#define X_MAX_PIN 24
#define X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define Y_STEP_PIN 36
#define Y_DIR_PIN 49
#define Y_MIN_PIN 11
#define Y_MAX_PIN 23
#define Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define Z_STEP_PIN 35
#define Z_DIR_PIN 47
#define Z_MIN_PIN 10
#define Z_MAX_PIN 30
#define Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define HEATER_BED_PIN 3
#define TEMP_BED_PIN 2

#define HEATER_0_PIN  9
#define TEMP_0_PIN 0

#define HEATER_1_PIN 7
#define TEMP_1_PIN 1

#ifdef BARICUDA
#define HEATER_2_PIN 6
#else
#define HEATER_2_PIN -1
#endif
#define TEMP_2_PIN -1

#define E0_STEP_PIN         34
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       26
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66

#define E1_STEP_PIN         33
#define E1_DIR_PIN          42
#define E1_ENABLE_PIN       25
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64

#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define FAN_PIN            8
#define PS_ON_PIN          4
#define KILL_PIN           -1 //80 with Smart Controller LCD
#define SUICIDE_PIN        -1  //PIN that has to be turned on right after start, to keep power flowing.

#ifdef ULTRA_LCD
  #define KILL_PIN 80
  #ifdef NEWPANEL
   //arduino pin which triggers an piezzo beeper
    #define BEEPER 79      // Beeper on AUX-4
    #define LCD_PINS_RS 70
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4 72
    #define LCD_PINS_D5 73
    #define LCD_PINS_D6 74
    #define LCD_PINS_D7 75

    //buttons are directly attached using AUX-2
    #define BTN_EN1 76
    #define BTN_EN2 77
    #define BTN_ENC 78  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SDCARDDETECT 81    // Ramps does not use this port

    //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define BEEPER 33    No Beeper added
    //buttons are attached to a shift register
    // Not wired this yet
    // #define SHIFT_CLK 38
    // #define SHIFT_LD 42
    // #define SHIFT_OUT 40
    // #define SHIFT_EN 17

    #define LCD_PINS_RS 75
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1

    //bits in the shift register that carry the buttons for:
    // left up center down right red
    #define BL_LE 7
    #define BL_UP 6
    #define BL_MI 5
    #define BL_DW 4
    #define BL_RI 3
    #define BL_ST 2
    #define BLEN_B 1
    #define BLEN_A 0
  #endif
#endif //ULTRA_LCD

#ifdef FILAMENT_SENSOR
  //Filip added pin for Filament sensor analog input 
  #define FILWIDTH_PIN        3
#endif //FILAMENT_SENSOR

#endif // RAMBO

/****************************************************************************************
* MegaTronics
*
****************************************************************************************/
#if MB(MEGATRONICS)
#define KNOWN_BOARD 1

 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif

#define LARGE_FLASH        true

#define X_STEP_PIN         26
#define X_DIR_PIN          28
#define X_ENABLE_PIN       24
#define X_MIN_PIN          41
#define X_MAX_PIN          37

#define Y_STEP_PIN         60 // A6
#define Y_DIR_PIN          61 // A7
#define Y_ENABLE_PIN       22
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         54 // A0
#define Z_DIR_PIN          55 // A1
#define Z_ENABLE_PIN       56 // A2
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN        31
#define E0_DIR_PIN         32
#define E0_ENABLE_PIN      38

#define E1_STEP_PIN        34
#define E1_DIR_PIN         36
#define E1_ENABLE_PIN      30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            7 // IO pin. Buffer needed
#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       9    // EXTRUDER 1
#define HEATER_1_PIN       8    // EXTRUDER 2 (FAN On Sprinter)
#define HEATER_2_PIN       -1

#if TEMP_SENSOR_0 == -1
#define TEMP_0_PIN         8   // ANALOG NUMBERING
#else
#define TEMP_0_PIN         13   // ANALOG NUMBERING

#endif

#define TEMP_1_PIN         15   // ANALOG NUMBERING
#define TEMP_2_PIN         -1   // ANALOG NUMBERING
#define HEATER_BED_PIN     10   // BED
#define TEMP_BED_PIN       14   // ANALOG NUMBERING

#define BEEPER 33     // Beeper on AUX-4

#ifdef ULTRA_LCD

  #ifdef NEWPANEL
  //arduino pin which triggers an piezzo beeper

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //buttons are directly attached using AUX-2
    #define BTN_EN1 59
    #define BTN_EN2 64
    #define BTN_ENC 43  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SDCARDDETECT -1   // Ramps does not use this port

      //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
#endif
#endif //ULTRA_LCD

#endif // MEGATRONICS

/****************************************************************************************
* MegaTronics v2.0
*
****************************************************************************************/
#if MB(MEGATRONICS_2)
 #define KNOWN_BOARD 1

 #ifndef __AVR_ATmega2560__
 #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif

 #define LARGE_FLASH        true

 #define X_STEP_PIN 26
 #define X_DIR_PIN 27
 #define X_ENABLE_PIN 25
 #define X_MIN_PIN 37
 #define X_MAX_PIN 40 //2 //Max endstops default to disabled "-1", set to commented value to enable.

 #define Y_STEP_PIN 4 // A6
 #define Y_DIR_PIN 54 // A0
 #define Y_ENABLE_PIN 5
 #define Y_MIN_PIN 41
 #define Y_MAX_PIN 38 //15

 #define Z_STEP_PIN 56 // A2
 #define Z_DIR_PIN 60 // A6
 #define Z_ENABLE_PIN 55 // A1
 #define Z_MIN_PIN 18
 #define Z_MAX_PIN 19

 #define E0_STEP_PIN 35
 #define E0_DIR_PIN 36
 #define E0_ENABLE_PIN 34

 #define E1_STEP_PIN 29
 #define E1_DIR_PIN 39
 #define E1_ENABLE_PIN 28

 #define E2_STEP_PIN 23
 #define E2_DIR_PIN 24
 #define E2_ENABLE_PIN 22

 #define SDPOWER -1
 #define SDSS 53
 #define LED_PIN 13

 #define FAN_PIN 7
 #define FAN2_PIN 6
 #define PS_ON_PIN 12
 #define KILL_PIN -1

 #define HEATER_0_PIN 9 // EXTRUDER 1
 #define HEATER_1_PIN 8 // EXTRUDER 2
 #define HEATER_2_PIN -1

 #if TEMP_SENSOR_0 == -1
   #define TEMP_0_PIN 4 // ANALOG NUMBERING
 #else
   #define TEMP_0_PIN 13 // ANALOG NUMBERING
 #endif

 #if TEMP_SENSOR_1 == -1
   #define TEMP_1_PIN 8 // ANALOG NUMBERING
 #else
   #define TEMP_1_PIN 15 // ANALOG NUMBERING
 #endif

 #define TEMP_2_PIN -1 // ANALOG NUMBERING

 #define HEATER_BED_PIN 10 // BED

 #if TEMP_SENSOR_BED == -1
   #define TEMP_BED_PIN 8 // ANALOG NUMBERING
 #else
   #define TEMP_BED_PIN 14 // ANALOG NUMBERING
 #endif

 #define BEEPER 64

 #define LCD_PINS_RS 14
 #define LCD_PINS_ENABLE 15
 #define LCD_PINS_D4 30
 #define LCD_PINS_D5 31
 #define LCD_PINS_D6 32
 #define LCD_PINS_D7 33

 //buttons are directly attached using keypad
 #define BTN_EN1 61
 #define BTN_EN2 59
 #define BTN_ENC 43 //the click

 #define BLEN_C 2
 #define BLEN_B 1
 #define BLEN_A 0

 #define SDCARDDETECT -1  // Megatronics does not use this port

   //encoder rotation values
 #define encrot0 0
 #define encrot1 2
 #define encrot2 3
 #define encrot3 1

#endif // MEGATRONICS_2

/****************************************************************************************
* Minitronics v1.0
*
****************************************************************************************/
#if MB(MEGATRONICS_1)
 #define KNOWN_BOARD 1

 #error Oops! Make sure you have 'Minitronics ' selected from the 'Tools -> Boards' menu.

 #define LARGE_FLASH        true

 #define X_STEP_PIN 48
 #define X_DIR_PIN 47
 #define X_ENABLE_PIN 49
 #define X_MIN_PIN 5
 #define X_MAX_PIN -1 //2 //Max endstops default to disabled "-1", set to commented value to enable.

 #define Y_STEP_PIN 39 // A6
 #define Y_DIR_PIN 40 // A0
 #define Y_ENABLE_PIN 38
 #define Y_MIN_PIN 2
 #define Y_MAX_PIN -1 //15

 #define Z_STEP_PIN 42 // A2
 #define Z_DIR_PIN 43 // A6
 #define Z_ENABLE_PIN 41 // A1
 #define Z_MIN_PIN 6
 #define Z_MAX_PIN -1

 #define E0_STEP_PIN 45
 #define E0_DIR_PIN 44
 #define E0_ENABLE_PIN 27

 #define E1_STEP_PIN 36
 #define E1_DIR_PIN 35
 #define E1_ENABLE_PIN 37

 #define E2_STEP_PIN -1
 #define E2_DIR_PIN -1
 #define E2_ENABLE_PIN -1

 #define SDPOWER -1
 #define SDSS 16
 #define LED_PIN 46

 #define FAN_PIN 9
 #define FAN2_PIN -1
 #define PS_ON_PIN -1
 #define KILL_PIN -1

 #define HEATER_0_PIN 7 // EXTRUDER 1
 #define HEATER_1_PIN 8 // EXTRUDER 2
 #define HEATER_2_PIN -1

 #define TEMP_0_PIN 7 // ANALOG NUMBERING
 #define TEMP_1_PIN 6 // ANALOG NUMBERING
 #define TEMP_2_PIN -1 // ANALOG NUMBERING

 #define HEATER_BED_PIN 3 // BED
 #define TEMP_BED_PIN 6 // ANALOG NUMBERING

 #define BEEPER -1

 #define LCD_PINS_RS -1
 #define LCD_PINS_ENABLE -1
 #define LCD_PINS_D4 -1
 #define LCD_PINS_D5 -1
 #define LCD_PINS_D6 -1
 #define LCD_PINS_D7 -1

 //buttons are directly attached using keypad
 #define BTN_EN1 -1
 #define BTN_EN2 -1
 #define BTN_ENC -1 //the click

 #define BLEN_C 2
 #define BLEN_B 1
 #define BLEN_A 0

 #define SDCARDDETECT -1  // Megatronics does not use this port

   //encoder rotation values
 #define encrot0 0
 #define encrot1 2
 #define encrot2 3
 #define encrot3 1

#endif // MEGATRONICS_1

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
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
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }

#endif //__PINS_H
