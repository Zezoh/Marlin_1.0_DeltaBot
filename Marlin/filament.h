// Filament runout sensor and filament change (M600)

#ifndef FILAMENT_H
#define FILAMENT_H

void filament_sensor_init();
void filament_sensor_poll();        // debounces the sensor, called from manage_inactivity()
bool filament_sensor_enabled();
void filament_sensor_enable(bool on);
bool filament_present();             // the sensor is on, has settled and sees filament
bool filament_absent();              // the sensor is on, has settled and sees none
void filament_note_extrusion();      // an extrusion was started: runout is watched from now on
void filament_service();             // called between G-code commands: starts a change when the filament ran out
bool filament_waiting();             // a filament change is waiting for the user
bool filament_hotkey(const char *line);   // M24 and M701 are answered at once while one is waiting
void filament_report();              // M412

// Pause, park, unload, wait for new filament, load, purge, wait for the confirmation, return and go on.
// service: from the button with nothing printing (home, wait above the centre, load only if there is no filament).
bool filament_change(bool service);

#endif
