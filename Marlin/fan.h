// The fan, with a safety floor for the hotend

#ifndef FAN_H
#define FAN_H

void fan_init();
void fan_service();          // called all the time (manage_inactivity())
void fan_start_cooldown();   // the heater was shut down or a print ended: full fan until the hotend is cool, M106 forgotten
unsigned char fan_output();  // what the print fan is set to, 0..255
unsigned char fan_pwm_steps(unsigned char duty);   // its on time in the 64 steps of the slow PWM

#endif
