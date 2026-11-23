// One button decoder: clicks, long presses, boot hold, bounce
#include <stdio.h>
#include <stdint.h>
#include "onebutton.h"

static int fails = 0;
#define CHECK(c, ...)      \
  do {                     \
    if (!(c)) {            \
      printf("FAIL: ");    \
      printf(__VA_ARGS__); \
      printf("\n");        \
      fails++;             \
    }                      \
  } while (0)

struct Rig {
  OneButton b;
  uint32_t t;
  int shorts, doubles, longs, boots;
  Rig(bool pressed_at_start) : b(40, 350, 1500, 2500), t(1000), shorts(0), doubles(0), longs(0), boots(0) { b.begin(pressed_at_start, t); }
  void run(bool pressed, uint32_t ms)
  {
    for (uint32_t i = 0; i < ms; i++, t++) {
      OneButton::Event e = b.update(pressed, t);
      shorts += e == OneButton::SHORT;
      doubles += e == OneButton::DOUBLE;
      longs += e == OneButton::LONG;
      boots += e == OneButton::BOOT_HOLD;
    }
  }
};

int main()
{
  {
    Rig r(false);
    r.run(false, 500);
    r.run(true, 200);
    r.run(false, 500);
    CHECK(r.shorts == 1 && r.longs == 0 && r.boots == 0, "a click is one SHORT (%d %d %d)", r.shorts, r.longs, r.boots);
  }
  {
    Rig r(false);
    r.run(true, 3000);
    CHECK(r.longs == 1 && r.shorts == 0, "a long press is one LONG");
    r.run(false, 500);
    CHECK(r.shorts == 0 && r.longs == 1, "releasing a long press is silent");
  }
  {
    Rig r(false);
    r.run(true, 1400);
    r.run(false, 500);
    CHECK(r.shorts == 1 && r.longs == 0, "1.4 s is still a click");
  }
  {
    Rig r(false);
    r.run(true, 30);
    r.run(false, 500);
    CHECK(r.shorts == 0, "a 30 ms glitch is ignored");
  }
  {
    Rig r(false);
    for (int i = 0; i < 20; i++) {
      r.run(true, 5);
      r.run(false, 5);
    }
    r.run(false, 200);
    CHECK(r.shorts == 0 && r.longs == 0, "contact bounce while idle does nothing");
  }
  {
    Rig r(false);
    for (int i = 0; i < 10; i++) {
      r.run(true, 3);
      r.run(false, 2);
    }
    r.run(true, 200);
    r.run(false, 500);
    CHECK(r.shorts == 1, "bounce at the press still gives one click (%d)", r.shorts);
  }
  {
    Rig r(true);
    r.run(true, 2000);
    CHECK(r.boots == 0, "boot hold needs 2.5 s");
    r.run(true, 700);
    CHECK(r.boots == 1 && r.longs == 0, "held since power-on: one BOOT_HOLD, no LONG (%d %d)", r.boots, r.longs);
    r.run(false, 300);
    CHECK(r.shorts == 0, "releasing a boot hold is silent");
    r.run(true, 200);
    r.run(false, 500);
    CHECK(r.shorts == 1, "the next press is an ordinary click");
  }
  {
    Rig r(true);
    r.run(true, 500);
    r.run(false, 300);
    CHECK(r.shorts == 0 && r.boots == 0, "a short press at power-on is not a click");
  }
  {
    Rig r(false);
    r.t = 0xFFFFFF00UL;
    r.b.begin(false, r.t);
    r.run(true, 1700);
    CHECK(r.longs == 1, "works across the millis() wrap");
  }
  {
    Rig r(false);
    r.run(false, 500);
    r.run(true, 100);
    r.run(false, 150);
    r.run(true, 100);
    r.run(false, 600);
    CHECK(r.doubles == 1 && r.shorts == 0, "two clicks are one DOUBLE (%d %d)", r.doubles, r.shorts);
  }
  {
    Rig r(false);
    r.run(true, 100);
    r.run(false, 200);
    CHECK(r.shorts == 0, "a click is not reported before the double click window closes");
    r.run(false, 200);
    CHECK(r.shorts == 1 && r.doubles == 0, "after the window it is a SHORT");
  }
  {
    Rig r(false);
    r.run(true, 100);
    r.run(false, 600);
    r.run(true, 100);
    r.run(false, 600);
    CHECK(r.shorts == 2 && r.doubles == 0, "two clicks far apart are two SHORTs (%d %d)", r.shorts, r.doubles);
  }
  {
    Rig r(false);
    r.run(true, 100);
    r.run(false, 150);
    r.run(true, 1700);
    r.run(false, 600);
    CHECK(r.longs == 1 && r.doubles == 0 && r.shorts == 0, "click then long press: LONG only (%d %d %d)", r.longs, r.doubles, r.shorts);
  }
  {
    Rig r(false);
    r.run(true, 100);
    r.run(false, 150);
    r.run(true, 100);
    r.run(false, 150);
    r.run(true, 100);
    r.run(false, 600);
    CHECK(r.doubles == 1 && r.shorts == 1, "three clicks: a DOUBLE and a SHORT (%d %d)", r.doubles, r.shorts);
  }
  printf(fails ? "onebutton: %d FAILED\n" : "onebutton: all ok\n", fails);
  return fails ? 1 : 0;
}
