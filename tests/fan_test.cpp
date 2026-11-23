// The fan (fan.cpp) with one fan and with two: M106/M107, the safety floor, the kick start, cooldown, unknown temperature.
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "Marlin.h"
#include "fan.h"
#include "cardreader.h"

CardReader card;
int fanSpeed = 0, g_verbose = 0;
bool cancel_heatup = false;
static unsigned long now_ms = 1000;
static float temp = 20, target = 0;
static bool stopped = false, moving = false, idle = true;
static int pwm = 0, safety_pin = 0, request = 0;

unsigned long millis() { return now_ms; }
int sim_read(int) { return 0; }
float degHotend(uint8_t) { return temp; }
float degTargetHotend(uint8_t) { return target; }
bool IsStopped() { return stopped; }
bool blocks_queued() { return moving; }
bool machine_idle() { return idle; }
unsigned char requested_fan_speed() { return request; }
// WRITE(FAN_SAFETY_PIN, v) is a no-op in the shim: the dual mode is tested through a hook
extern "C" void sim_write(int pin, int v)
{
  if (pin == 8) safety_pin = v;
}

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
static void run(unsigned long ms)
{
  for (unsigned long i = 0; i < ms; i += 10) {
    now_ms += 10;
    fan_service();
  }
  pwm = fan_output();
}
static void reset()
{
  now_ms += 100000;
  temp = 20;
  target = 0;
  stopped = moving = false;
  idle = true;
  request = 0;
  fanSpeed = 0;
  card.sdprinting = card.paused = false;
  pwm = 0;
  safety_pin = 0;
  fan_init();
  run(5000);
  fan_start_cooldown();
  run(100);  // cold: the cooldown ends at once
}

int main()
{
  // the slow PWM: 64 steps, 0 and 255 are plain off and on, a low duty is never rounded to off
  CHECK(fan_pwm_steps(0) == 0 && fan_pwm_steps(255) == 64 && fan_pwm_steps(1) == 1 && fan_pwm_steps(254) == 64 - 0 + 0 || fan_pwm_steps(254) == 64, "steps at the ends");
  CHECK(fan_pwm_steps(120) == 30 && fan_pwm_steps(128) == 32 && fan_pwm_steps(60) == 15, "steps in between (%d %d %d)", fan_pwm_steps(120), fan_pwm_steps(128), fan_pwm_steps(60));
  // a cold hotend: M106 / M107 as they are, with a kick start
  reset();
  CHECK(pwm == 0, "cold, no request: off (%d)", pwm);
  request = 100;
  run(100);
  CHECK(pwm == 255, "a starting fan gets full power first (%d)", pwm);
  run(500);
  CHECK(pwm == 100, "then the requested duty (%d)", pwm);
  request = 180;
  run(50);
  CHECK(pwm == 180, "a change of duty: no new kick (%d)", pwm);
  request = 0;
  run(50);
  CHECK(pwm == 0, "M107");
  request = 60;
  run(100);
  CHECK(pwm == 255, "it kicks again after a stop");

#if !FAN_DUAL
  printf("one fan:\n");
  // hot and printing: at least FAN_MIN_PWM, M106 higher goes through
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  request = 0;
  run(1000);
  CHECK(pwm == FAN_MIN_PWM, "hot and printing, no M106: the floor (%d)", pwm);
  request = 200;
  run(1000);
  CHECK(pwm == 200, "M106 S200 above the floor (%d)", pwm);
  request = 30;
  run(1000);
  CHECK(pwm == FAN_MIN_PWM, "M106 S30 below the floor (%d)", pwm);
  // hot and idle: full power
  card.sdprinting = false;
  request = 0;
  run(4000);
  CHECK(pwm == 255, "hot and idle: full power (%d)", pwm);
  // a print that just ended is still "printing" for a moment
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  run(500);
  card.sdprinting = false;
  request = 0;
  run(1000);
  CHECK(pwm == FAN_MIN_PWM, "1 s after the last move it still counts as printing (%d)", pwm);
  run(3000);
  CHECK(pwm == 255, "3 s later it does not (%d)", pwm);
  // hysteresis: on at 50, off below 49 (and the target)
  reset();
  request = 0;
  target = 0;
  temp = 49.9;
  run(200);
  CHECK(pwm == 0, "49.9 degC: no");
  temp = 50.0;
  run(200);
  CHECK(pwm == 255, "50 degC: on (idle: full)");
  temp = 49.5;
  run(200);
  CHECK(pwm == 255, "49.5 degC: still on (hysteresis)");
  temp = 48.9;
  run(200);
  CHECK(pwm == 0, "48.9 degC: off");
  // the target counts too
  reset();
  target = 200;
  temp = 30;
  run(200);
  CHECK(pwm == 255, "a hot target: on before the hotend is hot");
  // heater shut down: cooldown at full fan, and the M106 of the print is forgotten
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  request = 200;
  fanSpeed = 200;
  run(1000);
  card.sdprinting = false;
  target = 0;
  fan_start_cooldown();
  run(100);
  CHECK(pwm == 255 && fanSpeed == 0, "cooldown: full power, M106 forgotten (%d, %d)", pwm, fanSpeed);
  request = 0;
  temp = 120;
  run(10000);
  CHECK(pwm == 255, "still cooling at 120");
  temp = 48;
  run(500);
  CHECK(pwm == 0, "cool: off (%d)", pwm);
  // a new print after a cooldown: the cooldown must not hold the fan at full power
  reset();
  temp = 215;
  target = 0;
  fan_start_cooldown();
  run(100);
  target = 210;
  card.sdprinting = true;
  request = 90;
  run(1000);
  CHECK(pwm == FAN_MIN_PWM || pwm == 120, "heating for the next print: the cooldown is over (%d)", pwm);
  request = 200;
  run(500);
  CHECK(pwm == 200, "and M106 works (%d)", pwm);
  // unknown temperature and a stopped printer: full power
  reset();
  temp = -300;
  run(200);
  CHECK(pwm == 255, "sensor gone: full power");
  reset();
  temp = 20;
  stopped = true;
  run(200);
  CHECK(pwm == 255, "stopped: full power");
  stopped = false;
  temp = 20;
  run(200);
#else
  printf("two fans:\n");
  reset();
  request = 0;
  temp = 49.9;
  run(200);
  CHECK(safety_pin == 0 && pwm == 0, "cold: both off");
  temp = 50;
  run(200);
  CHECK(safety_pin == 1, "50 degC: the safety fan runs");
  CHECK(pwm == 0, "the print fan is not the safety fan's business (%d)", pwm);
  temp = 49.5;
  run(200);
  CHECK(safety_pin == 1, "hysteresis");
  temp = 48;
  run(200);
  CHECK(safety_pin == 0, "cool: off");
  reset();
  temp = 215;
  target = 215;
  card.sdprinting = true;
  request = 70;
  run(1000);
  CHECK(pwm == 70 && safety_pin == 1, "hot, printing: M106 as asked, safety fan on (%d)", pwm);
  request = 0;
  run(200);
  CHECK(pwm == 0, "M107 turns the print fan off even when hot");
  card.sdprinting = false;
  target = 0;
  fan_start_cooldown();
  run(200);
  CHECK(pwm == 255 && safety_pin == 1, "cooldown: both on");
  temp = 40;
  run(200);
  CHECK(pwm == 0 && safety_pin == 0, "cool: both off");
  reset();
  temp = -300;
  run(200);
  CHECK(safety_pin == 1 && pwm == 255, "sensor gone: both on");
#endif
  printf(fails ? "fan: %d FAILED\n" : "fan: all ok\n", fails);
  return fails ? 1 : 0;
}
