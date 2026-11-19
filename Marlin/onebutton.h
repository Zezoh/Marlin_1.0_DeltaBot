// One button decoder: debounce, short press, long press, and a press held since power-on.
// Pure logic (time and pin level are passed in), the host tests build it unchanged.

#ifndef ONEBUTTON_H
#define ONEBUTTON_H

#include <stdint.h>

class OneButton {
public:
  enum Event { NONE, SHORT, LONG, BOOT_HOLD };

  // pressed: the level at power-on; a button held then is a boot hold
  void begin(bool pressed, uint32_t now) {
    raw_ = stable_ = boot_ = pressed;
    spent_ = false;
    edge_ = down_ = now;
  }

  Event update(bool pressed, uint32_t now) {
    if (pressed != raw_) { raw_ = pressed; edge_ = now; }
    if (raw_ != stable_ && now - edge_ >= debounce_) {
      stable_ = raw_;
      if (stable_) {                       // pressed
        spent_ = false;
        down_ = now;
      } else {                             // released
        const bool click = !spent_ && !boot_;   // a press that began at power-on is never a click
        boot_ = false;
        spent_ = true;
        if (click) return SHORT;
      }
    }
    if (stable_ && !spent_ && now - down_ >= (boot_ ? boot_hold_ : long_)) {
      spent_ = true;                       // one event per press
      return boot_ ? BOOT_HOLD : LONG;
    }
    return NONE;
  }

  OneButton(uint32_t debounce, uint32_t long_press, uint32_t boot_hold)
    : debounce_(debounce), long_(long_press), boot_hold_(boot_hold), raw_(false), stable_(false), spent_(false), boot_(false), edge_(0), down_(0) {}

private:
  uint32_t debounce_, long_, boot_hold_;
  bool raw_, stable_, spent_, boot_;
  uint32_t edge_, down_;
};

#endif
