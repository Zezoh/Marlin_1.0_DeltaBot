// One button and status LED. The button is decoded in manage_inactivity(), which every blocking loop calls;
// what it asks for is carried out from loop(), between G-code commands.

#include "Marlin.h"
#include "temperature.h"
#include "cardreader.h"
#include "stepper.h"
#include "planner.h"
#include "probe.h"
#include "onebutton.h"
#include "ui.h"

static OneButton button(BUTTON_DEBOUNCE_MS, BUTTON_LONG_MS, BUTTON_BOOT_HOLD_MS);
static OneButton::Event pending = OneButton::NONE;
static uint8_t busy_depth = 0;
static bool cancel_flag = false;

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
    if (pending == OneButton::SHORT) pending = OneButton::NONE;   // do not carry a click out after the job it was not meant for
  }
  else if (busy_depth > 0 && --busy_depth == 0) cancel_flag = false;
}

bool ui_cancel_requested() { return cancel_flag; }

static unsigned int led_period()
{
  if (IsStopped()) return LED_ERROR_PERIOD_MS;
  if (target_temperature[0] > 0 && abs(target_temperature[0] - (int)degHotend(0)) > 3) return LED_HEAT_PERIOD_MS;
  #ifdef SDSUPPORT
  if (card.sdprinting) return LED_PRINT_PERIOD_MS;
  if (card.isPaused()) return LED_PAUSED_PERIOD_MS;
  #endif
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
  if (pending == OneButton::NONE || busy_depth) return;
  OneButton::Event e = pending;
  pending = OneButton::NONE;
  if (e == OneButton::BOOT_HOLD) {
    enquecommand_P(PSTR("G33"));
  } else if (e == OneButton::LONG) {
    #ifdef SDSUPPORT
    if (card.sdprinting || card.isPaused()) cancel_print(); else
    #endif
    disable_heater();   // idle: heater off, no motion
  } else if (e == OneButton::SHORT) {
    #ifdef SDSUPPORT
    if (card.sdprinting) {
      card.pauseSDPrint();
    } else if (card.isPaused()) {
      card.startFileprint();
      starttime = millis();
    } else
    #endif
    if (!machine_idle() || target_temperature[0] > 0) {
      // a host is printing (or something is heating): the button does not start anything
    } else if (!delta_calibrated) {
      enquecommand_P(PSTR("G33"));
    }
    #ifdef SDSUPPORT
    else {
      card.startNewest();
    }
    #endif
  }
}
