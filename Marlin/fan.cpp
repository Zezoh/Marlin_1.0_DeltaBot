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
//  The PWM is slow (64 steps at 976 Hz = 15 Hz, from the overflow of timer 2): a fan with its own driver electronics
//  only hums when it gets a PWM of hundreds of Hz.
//  A cooldown starts when the heater is shut down or a print ends or is cancelled (fan_start_cooldown()); it ends when
//  the hotend and its target are below FAN_SAFETY_OFF_C. It also forgets the M106 of the print, as an M107 would.

#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "cardreader.h"
#include "fan.h"

static bool hot_latched = false;  // the hotend is at the safety temperature (with hysteresis)
static bool cooldown = false;
static bool was_stopped = true;  // the fan was off: it starts with a kick
static unsigned long kick_until = 0;
static unsigned long last_activity = 0;
static unsigned char output = 0;
static bool forget_request = false;

#define FAN_PWM_STEPS 64

unsigned char fan_output() { return output; }

// the on time of a duty, in steps of the slow PWM
unsigned char fan_pwm_steps(unsigned char duty)
{
  if (duty == 0) return 0;
  if (duty == 255) return FAN_PWM_STEPS;
  unsigned char steps = ((unsigned int)duty * FAN_PWM_STEPS + 127) / 255;
  return steps ? steps : 1;
}

static volatile unsigned char pwm_steps = 0;
static volatile unsigned char pwm_phase = 0;

#ifdef __AVR__
// Timer 2 runs free at 16 MHz / 64 / 256 = 976 Hz; every overflow is one step of the slow PWM on FAN_PIN
ISR(TIMER2_OVF_vect)
{
  unsigned char steps = pwm_steps;
  if (steps == 0) {
    WRITE(FAN_PIN, LOW);
    return;
  }
  if (steps >= FAN_PWM_STEPS) {
    WRITE(FAN_PIN, HIGH);
    return;
  }
  unsigned char phase = pwm_phase + 1;
  if (phase >= FAN_PWM_STEPS) phase = 0;
  pwm_phase = phase;
  WRITE(FAN_PIN, phase < steps ? HIGH : LOW);
}
#endif

static void apply(unsigned char duty)
{
  if (duty == output) return;
  output = duty;
  CRITICAL_SECTION_START;
  pwm_steps = fan_pwm_steps(duty);
  pwm_phase = 0;
  CRITICAL_SECTION_END;
  WRITE(FAN_PIN, duty ? HIGH : LOW);  // from here the interrupt takes over
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
  WRITE(FAN_PIN, LOW);
#ifdef __AVR__
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));                                             // the pin is not driven by the timer
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));                                             // OC2A is the heater pin (PB4/D10): keep the timer off it
  TCCR2A |= _BV(WGM20) | _BV(WGM21);                                                  // fast PWM mode: it counts 0..255 and overflows
  TCCR2B = (TCCR2B & ~(_BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20))) | _BV(CS22);  // prescaler 64
  TIMSK2 |= _BV(TOIE2);
#endif
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
  if (forget_request) {  // a cooldown after a print: the M106 of the print is over
    fanSpeed = 0;
    request = 0;
    if (!blocks_queued()) forget_request = false;
  }

  float actual = degHotend(0), target = degTargetHotend(0);
  bool unknown = !(actual > -50 && actual < 1000);
  bool fault = unknown || IsStopped();
  bool cool_now = !unknown && actual <= FAN_SAFETY_OFF_C && target <= FAN_SAFETY_OFF_C;

  if (cooldown && (cool_now || target >= FAN_SAFETY_ON_C)) cooldown = false;  // cooled, or heating up for the next print
  if (unknown) hot_latched = true;                                            // nothing is known: assume it is hot
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
