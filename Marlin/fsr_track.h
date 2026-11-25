// FSR probe: the reference a stroke compares the reading with.
//
// A bed that was pressed relaxes for a while after the nozzle lifts. If the reference stayed where it was when
// the stroke started, that slow drift would work against the next contact (it needs more force, so the nozzle
// goes deeper and presses harder, which makes the next drift larger). The reference therefore follows the reading
// slowly (a low-pass filter of about 2^PROBE_TRACK_SHIFT samples, 5 ms each): a drift is followed, a contact,
// which is much quicker, is not.

#ifndef FSR_TRACK_H
#define FSR_TRACK_H

#include "Configuration_adv.h"

// ref16 is the reference times 16 (so the filter has a fraction to keep), raw the new 10 bit reading.
static inline void fsr_track_update(long &ref16, unsigned int raw)
{
  ref16 += (((long)raw << 4) - ref16) >> PROBE_TRACK_SHIFT;
}

static inline unsigned int fsr_track_reference(long ref16) { return (unsigned int)((ref16 + 8) >> 4); }

#endif
