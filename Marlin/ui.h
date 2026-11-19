// One button and status LED

#ifndef UI_H
#define UI_H

void ui_init();             // pins; a button held at power-on queues a calibration
void ui_poll();             // button and LED, called from manage_inactivity()
void ui_run();              // carries out the button's requests between G-code commands
void ui_busy(bool busy);    // a blocking operation (homing, probing, calibration) starts / ends
bool ui_cancel_requested(); // the button was held during a blocking operation

#endif
