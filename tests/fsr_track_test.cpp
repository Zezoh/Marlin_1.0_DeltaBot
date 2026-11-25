// The FSR stroke reference follows slow drift (a bed relaxing after a press) but not a contact.
#include <stdio.h>
#include <stdlib.h>
#include "fsr_track.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

// readings every 5 ms: base value, a drift in counts per second, a contact ramp (counts per sample) from sample `contact_at`
static int run(double drift_per_s, double contact_per_sample, int contact_at, unsigned threshold, int &hit_at)
{
  long ref16 = 500L << 4;
  hit_at = -1;
  for (int k = 0; k < 600; k++) {   // 3 s
    double v = 500 + drift_per_s * k * 0.005 + (k >= contact_at ? contact_per_sample * (k - contact_at) : 0);
    unsigned raw = (unsigned)(v + 0.5);
    unsigned ref = fsr_track_reference(ref16);
    unsigned delta = raw > ref ? raw - ref : ref - raw;
    if (delta >= threshold) { hit_at = k; return 1; }
    fsr_track_update(ref16, raw);
  }
  return 0;
}

int main()
{
  int at;
  CHECK(!run(0, 0, 9999, 12, at), "a steady reading never fires");
  CHECK(!run(-60, 0, 9999, 12, at), "a bed relaxing by 60 counts/s does not fire");
  CHECK(!run(40, 0, 9999, 12, at), "a drift in the other direction does not fire either");
  CHECK(run(0, 2.0, 100, 12, at) && at <= 112, "a contact (2 counts per sample) fires after %d samples", at - 100);
  CHECK(run(-60, 2.0, 100, 12, at) && at <= 115, "a contact on top of a drift against it still fires (%d samples)", at - 100);
  CHECK(run(60, 2.0, 100, 12, at) && at <= 112, "a contact on top of a drift with it fires (%d samples)", at - 100);
  CHECK(run(0, 1.0, 100, 19, at) && at <= 130, "a slower contact at threshold 19 fires (%d samples)", at - 100);
  printf(fails ? "fsr track: %d FAILED\n" : "fsr track: all ok\n", fails);
  return fails != 0;
}
