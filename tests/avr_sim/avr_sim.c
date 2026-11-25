// Runs the real firmware (the ELF built for the ATmega2560) in simavr with a simulated delta:
// serial port, thermistor and FSR on the ADC, endstops, button, runout sensor, step counters.
// Usage: avr_sim firmware.elf script.txt     (script commands: see below)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_adc.h>

static avr_t *avr;
static void no_sleep(avr_t *a, avr_cycle_count_t howLong) {}  // run as fast as possible, not in real time
static avr_irq_t *adc_irq;
static int invert_dir = 1;  // INVERT_X_DIR... in Configuration.h

// ---- pins: name, port letter, bit
typedef struct {
  const char *name;
  char port;
  int bit;
  int level;
  avr_irq_t *irq;
  long rises;
  long highcycles;
  avr_cycle_count_t last;
  int resets;
} pin_t;
enum { X_STEP,
       X_DIR,
       X_EN,
       X_MAX,
       Y_STEP,
       Y_DIR,
       Y_EN,
       Y_MAX,
       Z_STEP,
       Z_DIR,
       Z_EN,
       Z_MAX,
       E_STEP,
       E_DIR,
       E_EN,
       HEATER,
       FAN,
       LED,
       BUTTON,
       RUNOUT,
       NPINS };
static pin_t pins[NPINS] = {
    {"X_STEP", 'F', 0}, {"X_DIR", 'F', 1}, {"X_EN", 'D', 7}, {"X_MAX", 'E', 4}, {"Y_STEP", 'F', 6}, {"Y_DIR", 'F', 7}, {"Y_EN", 'F', 2}, {"Y_MAX", 'J', 0}, {"Z_STEP", 'L', 3}, {"Z_DIR", 'L', 1}, {"Z_EN", 'K', 0}, {"Z_MAX", 'D', 2}, {"E_STEP", 'A', 4}, {"E_DIR", 'A', 6}, {"E_EN", 'A', 2}, {"HEATER", 'B', 4}, {"FAN", 'H', 6}, {"LED", 'B', 6}, {"BUTTON", 'E', 5}, {"RUNOUT", 'J', 1}};

// ---- the machine
static long tower[3];  // carriage position in steps, up is positive, 0 = the endstop
static long estep;     // extruder steps, forward positive
static long tower_start[3] = {-12000, -11000, -13000};
static double temp_c = 25.0;
static int fsr_mv = 1500;
static unsigned long long out_bytes = 0;
#define MAXREC 400000
static unsigned long long rec_cycle[MAXREC];
static unsigned char rec_pin[MAXREC];
static signed char rec_dir[MAXREC];
static long nrec = 0;
static int recording = 0;
static long oks = 0, sent_lines = 0;
static char linebuf[256];
static int linelen = 0;
static char *queue[20000];
static int qhead = 0, qtail = 0;

static char outbuf[1 << 20];
static size_t outlen = 0, mark = 0;

static void set_input(pin_t *p, int level)  // external drive: LOW is a switch to ground, HIGH is released (pullup)
{
  avr_ioport_external_t e = {.name = p->port, .mask = 1 << p->bit, .value = level ? (1 << p->bit) : 0};
  if (level) {
    e.mask = 0;
    e.value = 0;
  }  // released: no external drive, the pullup decides
  avr_ioctl(avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(p->port), &e);
}

static int dir_positive(int dirpin) { return invert_dir ? !pins[dirpin].level : pins[dirpin].level; }

static void pin_cb(avr_irq_t *irq, uint32_t value, void *param)
{
  pin_t *p = param;
  int idx = p - pins;
  int old = p->level;
  p->level = value & 1;
  if (old && !p->level && p->last) p->highcycles += avr->cycle - p->last;
  if (!old && p->level) p->last = avr->cycle;
  if (!old && p->level) {
    p->rises++;
    if (recording && nrec < MAXREC && (idx == X_STEP || idx == Y_STEP || idx == Z_STEP || idx == E_STEP)) {
      rec_cycle[nrec] = avr->cycle;
      rec_dir[nrec] = dir_positive(idx == X_STEP ? X_DIR : idx == Y_STEP ? Y_DIR : idx == Z_STEP ? Z_DIR : E_DIR) ? 1 : -1;
      rec_pin[nrec++] = idx;
    }
    if (idx == X_STEP) tower[0] += dir_positive(X_DIR) ? 1 : -1;
    if (idx == Y_STEP) tower[1] += dir_positive(Y_DIR) ? 1 : -1;
    if (idx == Z_STEP) tower[2] += dir_positive(Z_DIR) ? 1 : -1;
    if (idx == E_STEP) estep += dir_positive(E_DIR) ? 1 : -1;  // E_DIR pin: HIGH is forward when INVERT_E0_DIR is false
  }
}

static void uart_out(avr_irq_t *irq, uint32_t value, void *param)
{
  if (outlen < sizeof(outbuf) - 1) outbuf[outlen++] = (char)value;
  if ((char)value == '\n') {
    linebuf[linelen] = 0;
    if (!strncmp(linebuf, "ok", 2)) oks++;
    linelen = 0;
  } else if (linelen < (int)sizeof(linebuf) - 1) linebuf[linelen++] = (char)value;
  outbuf[outlen] = 0;
  out_bytes++;
}

static double thermistor_mv(double t)  // 100k, beta 4125, 4.7k pullup, 5 V
{
  double r = 100000.0 * exp(4125.0 * (1.0 / (t + 273.15) - 1.0 / 298.15));
  return 5000.0 * r / (r + 4700.0);
}

// The ADC result is answered here, not by simavr (its mux table of the mega2560 does not follow MUX5):
// the channel is read from ADMUX and ADCSRB, the voltage is what the machine model says
static uint32_t adc_value(void)
{
  int ch = (avr->data[0x7C] & 7) | (avr->data[0x7B] & 8);
  double mv = 0;
  if (ch == 13) mv = thermistor_mv(temp_c);
  else if (ch == 14) mv = fsr_mv;
  double v = mv * 1023.0 / 5000.0;
  return v > 1023 ? 1023 : (uint32_t)v;
}
static uint8_t adc_read_l(avr_t *a, avr_io_addr_t addr, void *param) { return adc_value() & 0xff; }
static uint8_t adc_read_h(avr_t *a, avr_io_addr_t addr, void *param) { return adc_value() >> 8; }

static void update_inputs(void)
{
  // endstops: closed (LOW) when the carriage is at the top
  static int last[3] = {-1, -1, -1};
  static const int es[3] = {X_MAX, Y_MAX, Z_MAX};
  for (int i = 0; i < 3; i++) {
    int hit = tower[i] >= 0;
    if (hit != last[i]) {
      last[i] = hit;
      set_input(&pins[es[i]], !hit);
    }
  }
}

// ---- input feeding
static char inbuf[1 << 16];
static size_t inhead = 0, intail = 0;
static avr_cycle_count_t next_feed = 0;

static void send_line(const char *l);
static void feed(void)
{
  // a host sends the next line when it got an ok for an earlier one: two lines in flight
  if (qhead != qtail && inhead == intail && sent_lines - oks < 2) {
    send_line(queue[qhead++]);
    sent_lines++;
  }
  if (inhead == intail || avr->cycle < next_feed) return;
  avr_irq_t *in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
  avr_raise_irq(in, (uint8_t)inbuf[intail++]);
  next_feed = avr->cycle + 16000000 / 25000;  // 250000 baud, 10 bits per byte
}

static void send_line(const char *l)
{
  size_t n = strlen(l);
  memcpy(inbuf + inhead, l, n);
  inhead += n;
  inbuf[inhead++] = '\n';
}

static void run_ms(double ms)
{
  avr_cycle_count_t end = avr->cycle + (avr_cycle_count_t)(ms * 16000.0);
  avr_cycle_count_t next_update = 0;
  while (avr->cycle < end) {
    if (avr->cycle >= next_update) {
      update_inputs();
      next_update = avr->cycle + 16000;
    }
    feed();
    // simavr clears UDRE when the firmware writes UCSR0A (U2X0) and nothing sets it again before the first byte:
    // give the first byte the "data register empty" a real ATmega has after reset
    static int udre_forced = 0;
    if (!udre_forced && (avr->data[0xC1] & 0x08) && !(avr->data[0xC0] & 0x20)) {
      avr->data[0xC0] |= 0x20;
      udre_forced = 1;
    }
    int st = avr_run(avr);
    if (getenv("AVR_DEBUG")) {
      static long n = 0;
      static int lastst = -1;
      n++;
      if (avr->state != lastst || n % 5000000 == 0) {
        fprintf(stderr, "run %ld cycle %llu pc 0x%x state %d\n", n, (unsigned long long)avr->cycle, avr->pc, avr->state);
        lastst = avr->state;
      }
    }
    if (st == cpu_Done || st == cpu_Crashed) {
      printf("!! the cpu stopped (%d) at pc 0x%x\n", st, avr->pc);
      exit(2);
    }
  }
}

static int expect(const char *s, int from_mark)
{
  return strstr(outbuf + (from_mark ? mark : 0), s) != NULL;
}

int main(int argc, char **argv)
{
  if (argc < 3) {
    fprintf(stderr, "usage: %s firmware.elf script\n", argv[0]);
    return 1;
  }
  elf_firmware_t f;
  memset(&f, 0, sizeof f);
  if (elf_read_firmware(argv[1], &f)) {
    fprintf(stderr, "cannot read %s\n", argv[1]);
    return 1;
  }
  strcpy(f.mmcu, "atmega2560");
  f.frequency = 16000000;
  avr = avr_make_mcu_by_name("atmega2560");
  avr_init(avr);
  avr->frequency = 16000000;
  avr->vcc = avr->avcc = avr->aref = 5000;
  avr_load_firmware(avr, &f);
  avr->log = getenv("AVR_LOG") ? atoi(getenv("AVR_LOG")) : 0;
  avr->sleep = no_sleep;
  adc_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, 0);
  avr->io[AVR_DATA_TO_IO(0x78)].r.c = adc_read_l;
  avr->io[AVR_DATA_TO_IO(0x78)].r.param = NULL;  // replaces the handler of simavr
  avr->io[AVR_DATA_TO_IO(0x79)].r.c = adc_read_h;
  avr->io[AVR_DATA_TO_IO(0x79)].r.param = NULL;
  for (int i = 0; i < NPINS; i++) {
    pins[i].irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(pins[i].port), pins[i].bit);
    avr_irq_register_notify(pins[i].irq, pin_cb, &pins[i]);
  }
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out, NULL);
  // UART: 250000 baud is what the firmware sets itself
  uint32_t flags = 0;
  avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
  flags &= ~AVR_UART_FLAG_STDIO;
  avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
  for (int i = 0; i < 3; i++) tower[i] = tower_start[i];
  set_input(&pins[RUNOUT], 0);  // filament present is LOW (RUNOUT_ACTIVE_HIGH)

  FILE *sc = fopen(argv[2], "r");
  char line[512];
  int fails = 0;
  while (fgets(line, sizeof line, sc)) {
    line[strcspn(line, "\r\n")] = 0;
    if (!line[0] || line[0] == '#') continue;
    char *arg = strchr(line, ' ');
    if (arg) *arg++ = 0;
    if (!strcmp(line, "run")) run_ms(atof(arg));
    else if (!strcmp(line, "send")) {
      send_line(arg);
    } else if (!strcmp(line, "temp")) temp_c = atof(arg);
    else if (!strcmp(line, "fsr")) fsr_mv = atoi(arg);
    else if (!strcmp(line, "invert")) invert_dir = atoi(arg);
    else if (!strcmp(line, "mark")) mark = outlen;
    else if (!strcmp(line, "clear")) {
      for (int i = 0; i < NPINS; i++) {
        pins[i].rises = 0;
        pins[i].highcycles = 0;
      }
      estep = 0;
    } else if (!strcmp(line, "press")) {
      set_input(&pins[BUTTON], 0);
      run_ms(atof(arg));
      set_input(&pins[BUTTON], 1);
    } else if (!strcmp(line, "runout")) set_input(&pins[RUNOUT], atoi(arg));  // 1 = no filament
    else if (!strcmp(line, "print")) {
      printf("%s", outbuf + mark);
      mark = outlen;
    } else if (!strcmp(line, "gcode")) {  // gcode file first count: lines to send like a host does
      char path[400];
      int first, count;
      sscanf(arg, "%399s %d %d", path, &first, &count);
      FILE *g = fopen(path, "r");
      char l[300];
      int n = 0, used = 0;
      while (g && fgets(l, sizeof l, g)) {
        char *c = strchr(l, ';');
        if (c) *c = 0;
        l[strcspn(l, "\r\n")] = 0;
        char *t = l;
        while (*t == ' ') t++;
        if (!*t) continue;
        n++;
        if (n < first || used >= count) continue;
        queue[qtail++] = strdup(t);
        used++;
      }
      if (g) fclose(g);
      printf("  queued %d lines\n", used);
    } else if (!strcmp(line, "wait_queue")) {
      while (qhead != qtail || inhead != intail || sent_lines > oks) { run_ms(10); }
    } else if (!strcmp(line, "rec")) {
      recording = atoi(arg);
      if (recording) nrec = 0;
    } else if (!strcmp(line, "rates")) {  // rates window_ms count: tower steps per window since rec on
      double w = atof(arg) * 16000.0;
      int cnt = atoi(strchr(arg, ' ') ? strchr(arg, ' ') : "10");
      unsigned long long t0 = nrec ? rec_cycle[0] : 0;
      printf("  tower steps (X+Y+Z) per %.0f ms window, E steps in brackets:\n  ", w / 16000.0);
      long i = 0;
      for (int k = 0; k < cnt; k++) {
        long tw = 0, ew = 0;
        while (i < nrec && rec_cycle[i] < t0 + (unsigned long long)((k + 1) * w)) {
          if (rec_pin[i] == E_STEP) ew++;
          else tw++;
          i++;
        }
        printf("%ld(%ld) ", tw, ew);
        if ((k + 1) % 12 == 0) printf("\n  ");
      }
      printf("\n");
    } else if (!strcmp(line, "dump")) {  // dump <file>: every recorded step as "cycle tower direction" (tower 0..2, 3 = E)
      FILE *f = fopen(arg, "w");
      if (f) {
        for (long i = 0; i < nrec; i++)
          fprintf(f, "%llu %d %d\n", rec_cycle[i], rec_pin[i] == X_STEP ? 0 : rec_pin[i] == Y_STEP ? 1 : rec_pin[i] == Z_STEP ? 2 : 3, rec_dir[i]);
        fclose(f);
      }
    } else if (!strcmp(line, "pokef")) {
      unsigned a;
      float v;
      sscanf(arg, "%x %f", &a, &v);
      memcpy(avr->data + (a & 0xffff), &v, 4);
    }  // a float in the SRAM of the firmware
    else if (!strcmp(line, "pokeb")) {
      unsigned a, v;
      sscanf(arg, "%x %x", &a, &v);
      avr->data[a & 0xffff] = v;
    } else if (!strcmp(line, "status")) printf("  cycle %llu  pc 0x%x  state %d  uart bytes out %llu  UCSR0A %02x UCSR0B %02x UBRR0 %02x%02x  ADC %02x%02x ADMUX %02x ADCSRB %02x  irq13 %u irq14 %u vcc %u avcc %u aref %u\n", (unsigned long long)avr->cycle, avr->pc * 1, avr->state, out_bytes, avr->data[0xC0], avr->data[0xC1], avr->data[0xC5], avr->data[0xC4], avr->data[0x79], avr->data[0x78], avr->data[0x7C], avr->data[0x7B], adc_irq[ADC_IRQ_ADC13].value, adc_irq[ADC_IRQ_ADC14].value, avr->vcc, avr->avcc, avr->aref);
    else if (!strcmp(line, "say")) printf("--- %s\n", arg);
    else if (!strcmp(line, "show")) {
      printf("  towers (steps from the endstop): %ld %ld %ld   E steps: %ld   steps counted X %ld Y %ld Z %ld E %ld\n", tower[0], tower[1], tower[2], estep, pins[X_STEP].rises, pins[Y_STEP].rises, pins[Z_STEP].rises, pins[E_STEP].rises);
    } else if (!strcmp(line, "duty")) {  // duty of a pin over the time since clear: needs a window of run
      int idx = -1;
      for (int i = 0; i < NPINS; i++)
        if (!strcmp(pins[i].name, arg)) idx = i;
      printf("  %s: %ld rising edges, level now %d\n", arg, pins[idx].rises, pins[idx].level);
    } else if (!strcmp(line, "expect") || !strcmp(line, "expect_not")) {
      int want = !strcmp(line, "expect");
      int got = expect(arg, 1);
      printf("  %s %s: \"%s\"\n", got == want ? "ok  " : "FAIL", want ? "expect" : "expect_not", arg);
      if (got != want) fails++;
    } else if (!strcmp(line, "check")) {  // check <name> <op> <value> on E steps / towers
      char name[32], op[8];
      double v;
      sscanf(arg, "%31s %7s %lf", name, op, &v);
      double x = !strcmp(name, "e") ? estep : !strcmp(name, "t0")   ? tower[0]
                                          : !strcmp(name, "t1")     ? tower[1]
                                          : !strcmp(name, "t2")     ? tower[2]
                                          : !strcmp(name, "xsteps") ? pins[X_STEP].rises
                                          : !strcmp(name, "esteps") ? pins[E_STEP].rises
                                                                    : 0;
      int ok = !strcmp(op, "==") ? fabs(x - v) < 0.5 : !strcmp(op, ">") ? x > v
                                                   : !strcmp(op, "<")   ? x < v
                                                   : !strcmp(op, "~")   ? fabs(x - v) <= 3
                                                                        : 0;
      printf("  %s check %s %s %g (is %g)\n", ok ? "ok  " : "FAIL", name, op, v, x);
      if (!ok) fails++;
    } else if (!strcmp(line, "pin")) {
      int idx = -1;
      for (int i = 0; i < NPINS; i++)
        if (!strncmp(pins[i].name, arg, strlen(pins[i].name))) idx = i;
      printf("  %s level %d\n", pins[idx].name, pins[idx].level);
    } else printf("?? %s\n", line);
  }
  printf("%s\n", fails ? "FAILED" : "script done");
  return fails ? 1 : 0;
}
