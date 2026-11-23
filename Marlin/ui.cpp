// One button and status LED. The button is decoded in manage_inactivity(), which every blocking loop calls;
// what it asks for is carried out from loop(), between G-code commands.

#include "Marlin.h"
#include "temperature.h"
#include "cardreader.h"
#include "stepper.h"
#include "planner.h"
#include "probe.h"
#include "filament.h"
#include "onebutton.h"
#include "ui.h"

static OneButton button(BUTTON_DEBOUNCE_MS, BUTTON_DOUBLE_MS, BUTTON_LONG_MS, BUTTON_BOOT_HOLD_MS);
static OneButton::Event pending = OneButton::NONE;
static uint8_t busy_depth = 0;
static bool cancel_flag = false;
static uint8_t filament_mode = 0;
static bool short_flag = false;
static bool defer_cancel = false;

static bool button_pressed() { return !READ(BUTTON_PIN); }

void ui_init()
{
  SET_INPUT(BUTTON_PIN);
  WRITE(BUTTON_PIN, HIGH);   // pullup
  SET_OUTPUT(STATUS_LED_PIN);
  WRITE(STATUS_LED_PIN, LOW);
  button.begin(button_pressed(), millis());
}

void ui_busy(bool busy)
{
  if (busy) {
    busy_depth++;
    if (pending == OneButton::SHORT || pending == OneButton::DOUBLE) pending = OneButton::NONE;   // do not carry a click out after the job it was not meant for
  }
  else if (busy_depth > 0 && --busy_depth == 0) cancel_flag = false;
}

bool ui_cancel_requested() { return cancel_flag; }
void ui_filament_mode(uint8_t mode) { filament_mode = mode; }
bool ui_take_short() { bool s = short_flag; short_flag = false; return s; }
void ui_virtual_short() { short_flag = true; }
void ui_defer_cancel() { defer_cancel = true; }

static unsigned int led_period()
{
  if (IsStopped()) return LED_ERROR_PERIOD_MS;
  if (filament_mode == 2) return LED_WAIT_PERIOD_MS;
  if (filament_mode == 1) return LED_CHANGE_PERIOD_MS;
  if (target_temperature[0] > 0 && abs(target_temperature[0] - (int)degHotend(0)) > 3) return LED_HEAT_PERIOD_MS;
  if (card.sdprinting) return LED_PRINT_PERIOD_MS;
  if (card.isPaused()) return LED_PAUSED_PERIOD_MS;
  if (busy_depth || blocks_queued()) return LED_BUSY_PERIOD_MS;
  if (degHotend(0) > LED_COOL_TEMP) return LED_COOL_PERIOD_MS;
  return LED_READY_PERIOD_MS;
}

void ui_poll()
{
  unsigned long now = millis();
  OneButton::Event e = button.update(button_pressed(), now);
  if (e == OneButton::LONG) {
    if (busy_depth) cancel_flag = true;
    pending = e;
  } else if (e == OneButton::SHORT) {
    if (filament_mode) short_flag = true;   // goes on with the filament change
    else if (!busy_depth) pending = e;
  } else if (e == OneButton::DOUBLE) {
    if (!busy_depth) pending = e;
  } else if (e == OneButton::BOOT_HOLD) {
    pending = e;
  }
  static uint32_t last_led = 0;
  if (now - last_led >= 20) {   // 50 Hz is plenty for a blinking LED
    last_led = now;
    unsigned int period = led_period();
    WRITE(STATUS_LED_PIN, (now % period) < period / 2);
  }
}

void ui_run()
{
  if (busy_depth) return;
  if (defer_cancel) {
    defer_cancel = false;
    cancel_print();
  }
  if (pending == OneButton::NONE) return;
  OneButton::Event e = pending;
  pending = OneButton::NONE;
  if (e == OneButton::BOOT_HOLD) {
    enquecommand_P(PSTR("G33"));
  } else if (e == OneButton::LONG) {
    if (card.sdprinting || card.isPaused()) cancel_print(); else
    disable_heater();   // idle: heater off, no motion
  } else if (e == OneButton::SHORT) {
    if (card.sdprinting) {
      card.pauseSDPrint();
      filament_pause_request();   // the nozzle leaves the part when the commands already queued have run
    } else if (card.isPaused()) {
      if (filament_sensor_enabled() && !filament_present()) {
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("FILAMENT_REQUIRED: double click to change it");
      } else {
        if (filament_resume()) { card.startFileprint(); starttime = millis(); }
      }
    } else
    if (!machine_idle() || target_temperature[0] > 0) {
      // a host is printing (or something is heating): the button does not start anything
    } else if (!delta_calibrated) {
      enquecommand_P(PSTR("G33"));
    } else if (filament_absent()) {
      filament_change(true);   // load filament first; a click then starts the print
    }
    else {
      card.startNewest();
    }
  } else if (e == OneButton::DOUBLE) {
    if (card.isPaused()) filament_change(false);   // paused print: change the filament, a click resumes it
    else if (!card.sdprinting && machine_idle() && target_temperature[0] == 0) filament_change(true);
  }
}
