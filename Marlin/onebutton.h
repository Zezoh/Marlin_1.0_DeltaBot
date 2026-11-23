// One button decoder: debounce, click, double click, long press, and a press held since power-on.
// Pure logic (time and pin level are passed in), the host tests build it unchanged.

#ifndef ONEBUTTON_H
#define ONEBUTTON_H

#include <stdint.h>

class OneButton {
public:
  // SHORT is reported once the double click window has passed without a second press
  enum Event { NONE, SHORT, DOUBLE, LONG, BOOT_HOLD };

  OneButton(uint32_t debounce, uint32_t double_click, uint32_t long_press, uint32_t boot_hold)
    : debounce_(debounce), double_(double_click), long_(long_press), boot_hold_(boot_hold),
      raw_(false), stable_(false), spent_(false), boot_(false), pending_(false), second_(false), edge_(0), down_(0), released_(0) {}

  // pressed: the level at power-on; a button held then is a boot hold
  void begin(bool pressed, uint32_t now) {
    raw_ = stable_ = boot_ = pressed;
    spent_ = pending_ = second_ = false;
    edge_ = down_ = released_ = now;
  }

  Event update(bool pressed, uint32_t now) {
    if (pending_ && !stable_ && !raw_ && now - released_ >= double_) {   // the window closed: it was a single click
      pending_ = false;
      return SHORT;
    }
    if (pressed != raw_) { raw_ = pressed; edge_ = now; }
    if (raw_ != stable_ && now - edge_ >= debounce_) {
      stable_ = raw_;
      if (stable_) {                       // pressed
        spent_ = false;
        down_ = now;
        second_ = pending_;                // pressed again inside the window: a double click if it is released as a click
        pending_ = false;
      } else {                             // released
        const bool click = !spent_ && !boot_;   // a press that began at power-on is never a click
        boot_ = false;
        spent_ = true;
        if (click) {
          if (second_) { second_ = false; return DOUBLE; }
          pending_ = true;
          released_ = now;
        }
        second_ = false;
      }
    }
    if (stable_ && !spent_ && now - down_ >= (boot_ ? boot_hold_ : long_)) {
      spent_ = true;                       // one event per press
      pending_ = second_ = false;
      return boot_ ? BOOT_HOLD : LONG;
    }
    return NONE;
  }

private:
  uint32_t debounce_, double_, long_, boot_hold_;
  bool raw_, stable_, spent_, boot_, pending_, second_;
  uint32_t edge_, down_, released_;
};

#endif
