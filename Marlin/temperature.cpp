/*
  temperature.c - temperature control
  Part of Marlin

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

#include "Marlin.h"
#include "temperature.h"
#include "watchdog.h"
#include "language.h"
#include "fan.h"

int target_temperature[EXTRUDERS] = {0};
volatile int current_temperature_raw[EXTRUDERS] = {0};
float current_temperature[EXTRUDERS] = {0.0};
float Kp = DEFAULT_Kp;
float Ki = DEFAULT_Ki * PID_dT;
float Kd = DEFAULT_Kd / PID_dT;

static volatile bool temp_meas_ready = false;

// Latest FSR (Z probe) reading, 10 bit. Sampled twice per 10 ms from the idle ticks of the temperature
// state machine; the stepper interrupt compares it with a reference while a probe stroke runs.
volatile unsigned int fsr_raw = 0;
volatile unsigned char fsr_seq = 0;  // counts the readings: the stepper interrupt updates its reference once per new reading

static float temp_iState = 0, temp_dState = 0, dTerm = 0, temp_iState_max;
static bool pid_reset = true;
static volatile unsigned char soft_pwm = 0;  // heater power, 0..127; the ISR reads it, the main loop writes it

// The thermistor reads LOW raw values when it is hot. Limits are found at start from the temperatures in Configuration.h.
static int minttemp_raw = 16383;  // at or above this raw value: too cold, wire broken
static int maxttemp_raw = 0;      // at or below: too hot

static float analog2temp(int raw);
static void updateTemperaturesFromRawValues();

//===========================================================================
//=============================   functions      ============================
//===========================================================================

void PID_autotune(float temp, int ncycles)
{
  float input = 0.0;
  int cycles = 0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1 = temp_millis;
  unsigned long t2 = temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max_temp = 0, min_temp = 10000;

  if (temp < EXTRUDE_MINTEMP) temp = EXTRUDE_MINTEMP;
  if (temp > HEATER_0_MAXTEMP - 15) temp = HEATER_0_MAXTEMP - 15;  // the autotune cycles the heater around it, keep room for the emergency stop
  SERIAL_ECHOLN("PID Autotune start");

  disable_heater();
  soft_pwm = (PID_MAX) / 2;
  bias = d = (PID_MAX) / 2;

  for (;;) {
    if (IsStopped()) {  // the emergency stop fired: get out, manage_hotends can no longer hold the heater back
      disable_heater();
      return;
    }
    fan_service();                  // the hotend fan must run during an autotune
    if (temp_meas_ready == true) {  // temp sample ready
      updateTemperaturesFromRawValues();

      input = current_temperature[0];
      max_temp = max(max_temp, input);
      min_temp = min(min_temp, input);

      if (heating == true && input > temp) {
        if (millis() - t2 > 5000) {
          heating = false;
          soft_pwm = (bias - d) >> 1;
          t1 = millis();
          t_high = t1 - t2;
          max_temp = temp;
        }
      }
      if (heating == false && input < temp) {
        if (millis() - t1 > 5000) {
          heating = true;
          t2 = millis();
          t_low = t2 - t1;
          if (cycles > 0) {
            bias += (d * (t_high - t_low)) / (t_low + t_high);
            bias = constrain(bias, 20, (PID_MAX)-20);
            if (bias > (PID_MAX) / 2) d = (PID_MAX)-1 - bias;
            else d = bias;

            SERIAL_PROTOCOLPGM(" bias: ");
            SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: ");
            SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: ");
            SERIAL_PROTOCOL(min_temp);
            SERIAL_PROTOCOLPGM(" max: ");
            SERIAL_PROTOCOLLN(max_temp);
            if (cycles > 2) {
              Ku = (4.0 * d) / (3.14159 * (max_temp - min_temp) / 2.0);
              Tu = ((float)(t_low + t_high) / 1000.0);
              SERIAL_PROTOCOLPGM(" Ku: ");
              SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: ");
              SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6 * Ku;
              Ki = 2 * Kp / Tu;
              Kd = Kp * Tu / 8;
              SERIAL_PROTOCOLLNPGM(" Classic PID ");
              SERIAL_PROTOCOLPGM(" Kp: ");
              SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: ");
              SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: ");
              SERIAL_PROTOCOLLN(Kd);
            }
          }
          soft_pwm = (bias + d) >> 1;
          cycles++;
          min_temp = temp;
        }
      }
    }
    if (input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
      disable_heater();
      return;
    }
    if (millis() - temp_millis > 2000) {
      SERIAL_PROTOCOLPGM("ok T:");
      SERIAL_PROTOCOL(input);
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOLLN((int)soft_pwm);
      temp_millis = millis();
    }
    if (((millis() - t1) + (millis() - t2)) > (10L * 60L * 1000L * 2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      disable_heater();
      return;
    }
    if (cycles > ncycles) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished! Send M301 P I D with the last Kp, Ki and Kd from above, then M500");
      disable_heater();
      return;
    }
  }
}

void updatePID()
{
  temp_iState_max = PID_INTEGRAL_DRIVE_MAX / Ki;
}

int getHeaterPower(int heater) { return soft_pwm; }

// Thermal runaway: after the target was reached once, the temperature must not stay too far below it.
static int runaway_state = 0;
static unsigned long runaway_timer = 0;
static int timer_target = 0;  // the target the current stability period started with
static bool thermal_runaway = false;

static void thermal_runaway_protection(float temperature, float target)
{
  if ((target == 0) || thermal_runaway) {
    runaway_state = 0;
    runaway_timer = 0;
    return;
  }
  switch (runaway_state) {
    case 0:  // "Heater Inactive" state
      if (target > 0) runaway_state = 1;
      break;
    case 1:  // "First Heating" state
      if (temperature >= target) {
        runaway_state = 2;
        runaway_timer = millis();
        timer_target = target;
      }
      break;
    case 2:  // "Temperature Stable" state
      if (target > timer_target) {
        runaway_state = 1;
        break;
      }  // the target was raised: allow heating up to it first
      if (temperature >= (target - THERMAL_RUNAWAY_PROTECTION_HYSTERESIS)) {
        runaway_timer = millis();
      } else if ((millis() - runaway_timer) > ((unsigned long)THERMAL_RUNAWAY_PROTECTION_PERIOD) * 1000) {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM("Thermal Runaway, system stopped!");
        thermal_runaway = true;
        while (1) {  // the watchdog resets the board: nothing here may feed it (no manage_heater())
          disable_heater();
          disable_x();
          disable_y();
          disable_z();
          disable_e0();
          // no manage_heater()/fan_service() here: they reach watchdog_reset() and would stop the reset
        }
      }
      break;
  }
}

void manage_heater()
{
  float pid_output;

  if (temp_meas_ready != true) return;
  updateTemperaturesFromRawValues();

  thermal_runaway_protection(current_temperature[0], target_temperature[0]);

  float pid_input = current_temperature[0];
  float pid_error = target_temperature[0] - pid_input;
  if (pid_error > PID_FUNCTIONAL_RANGE) {
    pid_output = PID_MAX;
    pid_reset = true;
  } else if (pid_error < -PID_FUNCTIONAL_RANGE || target_temperature[0] == 0) {
    pid_output = 0;
    pid_reset = true;
  } else {
    if (pid_reset == true) {
      temp_iState = 0.0;
      pid_reset = false;
    }
    float pTerm = Kp * pid_error;
    temp_iState += pid_error;
    temp_iState = constrain(temp_iState, 0, temp_iState_max);
    float iTerm = Ki * temp_iState;

#define K2 (1.0 - K1)
    dTerm = (Kd * (pid_input - temp_dState)) * K2 + (K1 * dTerm);
    pid_output = pTerm + iTerm - dTerm;
    if (pid_output > PID_MAX) {
      if (pid_error > 0) temp_iState -= pid_error;  // conditional un-integration
      pid_output = PID_MAX;
    } else if (pid_output < 0) {
      if (pid_error < 0) temp_iState -= pid_error;  // conditional un-integration
      pid_output = 0;
    }
  }
  temp_dState = pid_input;

  // the heater is only on while the temperature is inside its limits
  if (current_temperature[0] > HEATER_0_MINTEMP && current_temperature[0] < HEATER_0_MAXTEMP) soft_pwm = (int)pid_output >> 1;
  else soft_pwm = 0;
}

#define PGM_RD_W(x) (short)pgm_read_word(&x)
// Raw (oversampled) ADC value to degrees by interpolating the table of the thermistor
static float analog2temp(int raw)
{
  uint8_t i;
  for (i = 1; i < HEATER_0_TEMPTABLE_LEN; i++) {
    if (PGM_RD_W(HEATER_0_TEMPTABLE[i][0]) > raw) {
      return PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][1]) +
             (raw - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][0])) *
                 (float)(PGM_RD_W(HEATER_0_TEMPTABLE[i][1]) - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][1])) /
                 (float)(PGM_RD_W(HEATER_0_TEMPTABLE[i][0]) - PGM_RD_W(HEATER_0_TEMPTABLE[i - 1][0]));
    }
  }
  return PGM_RD_W(HEATER_0_TEMPTABLE[HEATER_0_TEMPTABLE_LEN - 1][1]);  // beyond the table: its last value
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
  current_temperature[0] = analog2temp(current_temperature_raw[0]);
  watchdog_reset();  // we know we have a temperature measurement

  CRITICAL_SECTION_START;
  temp_meas_ready = false;
  CRITICAL_SECTION_END;
}

void tp_init()
{
  updatePID();
  SET_OUTPUT(HEATER_0_PIN);

  // Set analog inputs
  ADCSRA = 1 << ADEN | 1 << ADSC | 1 << ADIF | 0x07;
  DIDR0 = 0;
#ifdef DIDR2
  DIDR2 = 0;
#endif
  DIDR2 |= 1 << (TEMP_0_PIN - 8);
#if FSR_PIN < 8
  DIDR0 |= 1 << FSR_PIN;
#else
  DIDR2 |= 1 << (FSR_PIN - 8);
#endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1 << OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);

  // The raw values of the minimum and maximum temperature
  while (analog2temp(minttemp_raw) < HEATER_0_MINTEMP) minttemp_raw -= OVERSAMPLENR;
  while (analog2temp(maxttemp_raw) > HEATER_0_MAXTEMP) maxttemp_raw += OVERSAMPLENR;
}

void disable_heater()
{
  fan_start_cooldown();  // the hotend is hot and the heater is off: cool it at full fan
  target_temperature[0] = 0;
  soft_pwm = 0;
  WRITE(HEATER_0_PIN, LOW);
}

static void temp_error(bool too_hot)
{
  disable_heater();
  if (IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Hotend switched off. ");
    if (too_hot) {
      SERIAL_ERRORLNPGM("MAXTEMP triggered!");
    } else {
      SERIAL_ERRORLNPGM("MINTEMP triggered!");
    }
  }
  Stop();
}

// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  // these variables are only accessible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned char temp_state = 10;
  static unsigned char pwm_count = (1 << SOFT_PWM_SCALE);
  static unsigned char soft_pwm_0;

  // Software PWM of the heaters and the fan
  if (pwm_count == 0) {
    soft_pwm_0 = soft_pwm;
    if (soft_pwm_0 > 0) {
      WRITE(HEATER_0_PIN, 1);
    } else WRITE(HEATER_0_PIN, 0);
  }
  if (soft_pwm_0 < pwm_count) {
    WRITE(HEATER_0_PIN, 0);
  }

  pwm_count += (1 << SOFT_PWM_SCALE);
  pwm_count &= 0x7f;

  switch (temp_state) {
    case 0:  // Prepare TEMP_0
      ADCSRB = 1 << MUX5;
      ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
      ADCSRA |= 1 << ADSC;  // Start conversion
      temp_state = 1;
      break;
    case 1:  // Measure TEMP_0
      raw_temp_0_value += ADC;
      temp_state = 2;
      break;
    case 2:  // idle ticks keep the sample at 10 ticks, the period PID_dT is calculated for
    case 3:
    case 4:
    case 5:
      temp_state++;
      break;
    case 6:  // Prepare FSR. The first conversion after a channel change is discarded.
#if FSR_PIN > 7
      ADCSRB = 1 << MUX5;
#else
      ADCSRB = 0;
#endif
      ADMUX = ((1 << REFS0) | (FSR_PIN & 0x07));
      ADCSRA |= 1 << ADSC;  // Start conversion
      temp_state = 7;
      break;
    case 7:  // Discard the first FSR conversion, start the next
      ADCSRA |= 1 << ADSC;
      temp_state = 8;
      break;
    case 8:  // Measure FSR, start another one
      fsr_raw = ADC;
      fsr_seq++;
      ADCSRA |= 1 << ADSC;
      temp_state = 9;
      break;
    case 9:  // Measure FSR; this tick also keeps the sample at 10 ticks, the period PID_dT is calculated for
      fsr_raw = ADC;
      fsr_seq++;
      temp_state = 0;
      temp_count++;
      break;

    case 10:  // startup: delay the first reading a tiny bit so the hardware can settle
      temp_state = 0;
      break;
  }

  if (temp_count >= OVERSAMPLENR)  // 10 * 16 * 1/(16000000/64/256)  = 164ms.
  {
    if (!temp_meas_ready)  // Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw[0] = raw_temp_0_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;

    // thermistor: the raw value is low for a high temperature
    if (current_temperature_raw[0] <= maxttemp_raw) temp_error(true);
    if (current_temperature_raw[0] >= minttemp_raw) temp_error(false);
  }
}

// Apply the scale factors to the PID values
float scalePID_i(float i) { return i * PID_dT; }
float unscalePID_i(float i) { return i / PID_dT; }
float scalePID_d(float d) { return d / PID_dT; }
float unscalePID_d(float d) { return d * PID_dT; }
