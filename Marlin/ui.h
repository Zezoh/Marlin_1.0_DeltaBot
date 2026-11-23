// One button and status LED

#ifndef UI_H
#define UI_H

void ui_init();             // pins; a button held at power-on queues a calibration
void ui_poll();             // button and LED, called from manage_inactivity()
void ui_run();              // carries out the button's requests between G-code commands
void ui_busy(bool busy);    // a blocking operation (homing, probing, calibration) starts / ends
bool ui_cancel_requested(); // the button was held during a blocking operation
void ui_filament_mode(uint8_t mode);   // 0 no filament change, 1 it works, 2 it waits for the user
bool ui_take_short();       // a click during a filament change (or M24 / M701 for it): true once
void ui_virtual_short();
void ui_defer_cancel();     // cancel_print() as soon as the running G-code command has ended (it clears the command buffer)

#endif
