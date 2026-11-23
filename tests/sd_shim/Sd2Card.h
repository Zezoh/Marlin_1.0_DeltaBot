#pragma once
#include "Marlin.h"
struct Sd2Card {
  uint8_t *img;
  uint32_t blocks;
  unsigned reads, writes;
  bool readBlock(uint32_t b, uint8_t *d)
  {
    if (b >= blocks) return false;
    memcpy(d, img + b * 512, 512);
    reads++;
    return true;
  }
  bool writeBlock(uint32_t b, const uint8_t *s)
  {
    if (b >= blocks) return false;
    memcpy(img + b * 512, s, 512);
    writes++;
    return true;
  }
};
