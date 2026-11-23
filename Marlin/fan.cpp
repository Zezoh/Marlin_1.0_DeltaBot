// The fan. Its duty is what M106/M107 asked for (the speed of the block being executed), with these rules:
//
//  One fan (FAN_DUAL 0): the fan on FAN_PIN cools the print and the heat break of the hotend.
//   - the temperature of the hotend is unknown, the printer is stopped or a cooldown is on: full power
//   - the hotend (or its target) is at FAN_SAFETY_ON_C or above, until both are below FAN_SAFETY_OFF_C:
//       printing: at least FAN_MIN_PWM, M106 can go higher;  not printing: full power for the quickest cooldown
//   - otherwise: what M106/M107 ask for
//  Two fans (FAN_DUAL 1): FAN_SAFETY_PIN is an on/off fan that only cares about the temperature (on from
//   FAN_SAFETY_ON_C to FAN_SAFETY_OFF_C, and when the temperature is unknown or the printer is stopped); the fan on
//   FAN_PIN is the print fan: what M106/M107 ask for, full power during a cooldown, no safety floor.
//  A fan that starts gets full power for FAN_KICK_MS so that it spins up before the PWM duty is applied.
//  A cooldown starts when the heater is shut down or a print ends or is cancelled (fan_start_cooldown()); it ends when
//  the hotend and its target are below FAN_SAFETY_OFF_C. It also forgets the M106 of the print, as an M107 would.

#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "cardreader.h"
#include "fan.h"

static bool hot_latched = false;     // the hotend is at the safety temperature (with hysteresis)
static bool cooldown = false;
static bool was_stopped = true;      // the fan was off: it starts with a kick
static unsigned long kick_until = 0;
static unsigned long last_activity = 0;
static unsigned char output = 0;
static bool forget_request = false;

unsigned char fan_output() { return output; }

static void apply(unsigned char duty)
{
  if (duty == output) return;
  output = duty;
  analogWrite(FAN_PIN, duty);   // 255 and 0 are plain high and low
}

// A duty with the kick start
static void apply_with_kick(unsigned char duty)
{
  if (duty == 0) {
    apply(0);
    was_stopped = true;
    kick_until = 0;
    return;
  }
  if (was_stopped) {
    kick_until = millis() + FAN_KICK_MS;
    was_stopped = false;
  }
  if (kick_until != 0 && (long)(millis() - kick_until) < 0) {
    apply(255);
  } else {
    kick_until = 0;
    apply(duty);
  }
}

static void apply_full()
{
  apply(255);
  was_stopped = false;
  kick_until = 0;
}

void fan_init()
{
  SET_OUTPUT(FAN_PIN);
  analogWrite(FAN_PIN, 0);
  #if FAN_DUAL
    SET_OUTPUT(FAN_SAFETY_PIN);
    WRITE(FAN_SAFETY_PIN, LOW);
  #endif
}

void fan_start_cooldown()
{
  cooldown = true;
  forget_request = true;
}

void fan_service()
{
  unsigned long now = millis();
  if (card.sdprinting || card.isPaused() || blocks_queued() || !machine_idle()) last_activity = now;
  bool printing = card.sdprinting || card.isPaused() || (now - last_activity) < FAN_PRINTING_HOLD_MS;

  unsigned char request = requested_fan_speed();
  if (forget_request) {          // a cooldown after a print: the M106 of the print is over
    fanSpeed = 0;
    request = 0;
    if (!blocks_queued()) forget_request = false;
  }

  float actual = degHotend(0), target = degTargetHotend(0);
  bool unknown = !(actual > -50 && actual < 1000);
  bool fault = unknown || IsStopped();
  bool cool_now = !unknown && actual <= FAN_SAFETY_OFF_C && target <= FAN_SAFETY_OFF_C;

  if (cooldown && (cool_now || target >= FAN_SAFETY_ON_C)) cooldown = false;   // cooled, or heating up for the next print
  if (unknown) hot_latched = true;                                       // nothing is known: assume it is hot
  else if (actual >= FAN_SAFETY_ON_C || target >= FAN_SAFETY_ON_C) hot_latched = true;
  else if (cool_now) hot_latched = false;

  #if FAN_DUAL
    WRITE(FAN_SAFETY_PIN, (hot_latched || fault) ? HIGH : LOW);
    if (cooldown || fault) apply_full();
    else apply_with_kick(request);
  #else
    if (fault || cooldown) apply_full();
    else if (hot_latched) {
      if (printing) apply_with_kick(request < FAN_MIN_PWM ? FAN_MIN_PWM : request);
      else apply_full();
    } else {
      apply_with_kick(request);
    }
  #endif
}
