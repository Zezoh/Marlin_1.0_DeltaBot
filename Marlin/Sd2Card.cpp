/**
 * Sd2Card - raw block access to SD / SDHC cards over hardware SPI.
 *
 * Based on Arduino Sd2Card Library, Copyright (C) 2009 by William Greman
 * (GPL v3). Reduced to the features used by the Delta firmware.
 */
#include "Sd2Card.h"

#ifdef SDSUPPORT

// make sure SPCR rate is in expected bits
#if (SPR0 != 0 || SPR1 != 1)
#error unexpected SPCR bits
#endif

//------------------------------------------------------------------------------
// Hardware SPI helpers

/** Set SCK rate to F_CPU/pow(2, 1 + spiRate) for spiRate [0,6] */
static void spiInit(uint8_t spiRate) {
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

static inline void spiWait() {
  while (!(SPSR & (1 << SPIF))) { /* wait for transfer complete */ }
}

/** SPI receive a byte */
static uint8_t spiRec() {
  SPDR = 0XFF;
  spiWait();
  return SPDR;
}

/** SPI read data - only one call so force inline */
static inline __attribute__((always_inline))
void spiRead(uint8_t* buf, uint16_t nbyte) {
  if (nbyte-- == 0) return;
  SPDR = 0XFF;
  for (uint16_t i = 0; i < nbyte; i++) {
    spiWait();
    buf[i] = SPDR;
    SPDR = 0XFF;
  }
  spiWait();
  buf[nbyte] = SPDR;
}

/** SPI send a byte */
static void spiSend(uint8_t b) {
  SPDR = b;
  spiWait();
}

/** SPI send block - only one call so force inline */
static inline __attribute__((always_inline))
void spiSendBlock(uint8_t token, const uint8_t* buf) {
  SPDR = token;
  for (uint16_t i = 0; i < 512; i += 2) {
    spiWait();
    SPDR = buf[i];
    spiWait();
    SPDR = buf[i + 1];
  }
  spiWait();
}

//------------------------------------------------------------------------------
// Optional CRC checking of read data (enable SD_CHECK_AND_RETRY in Configuration.h)
#ifdef SD_CHECK_AND_RETRY
static const uint16_t crctab[] PROGMEM = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static uint16_t CRC_CCITT(const uint8_t* data, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc = pgm_read_word(&crctab[(crc >> 8 ^ data[i]) & 0XFF]) ^ (crc << 8);
  }
  return crc;
}
#endif  // SD_CHECK_AND_RETRY

//------------------------------------------------------------------------------
void Sd2Card::chipSelectHigh() {
  digitalWrite(chipSelectPin_, HIGH);
}

void Sd2Card::chipSelectLow() {
  spiInit(spiRate_);
  digitalWrite(chipSelectPin_, LOW);
}

// wait for card to go not busy
bool Sd2Card::waitNotBusy(uint16_t timeoutMillis) {
  uint16_t t0 = millis();
  while (spiRec() != 0XFF) {
    if (((uint16_t)millis() - t0) >= timeoutMillis) return false;
  }
  return true;
}

// send command and return the R1 response byte (zero for OK)
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg) {
  chipSelectLow();
  // wait up to 300 ms if busy
  waitNotBusy(300);
  spiSend(cmd | 0x40);
  for (int8_t s = 24; s >= 0; s -= 8) spiSend(arg >> s);
  // CRC is only checked for CMD0 and CMD8
  uint8_t crc = 0XFF;
  if (cmd == CMD0) crc = 0X95;  // correct crc for CMD0 with arg 0
  if (cmd == CMD8) crc = 0X87;  // correct crc for CMD8 with arg 0X1AA
  spiSend(crc);
  // skip stuff byte for stop read
  if (cmd == CMD12) spiRec();
  // wait for response
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++) { /* wait */ }
  return status_;
}

//------------------------------------------------------------------------------
/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector (SPI_FULL_SPEED or SPI_HALF_SPEED).
 * \param[in] chipSelectPin SD chip select pin number.
 * \return true for success or false for failure.
 */
bool Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
  errorCode_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;
  // 16-bit init start time allows over a minute
  uint16_t t0 = (uint16_t)millis();
  uint32_t arg;

  pinMode(chipSelectPin_, OUTPUT);
  chipSelectHigh();
  pinMode(MISO_PIN, INPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  // SS must be in output mode even if it is not the chip select,
  // and is kept high as it may be the chip select of another SPI device
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);

  // slow clock for initialization commands
  spiRate_ = SPI_SD_INIT_RATE;
  spiInit(spiRate_);

  // must supply min of 74 clock cycles with CS high
  for (uint8_t i = 0; i < 10; i++) spiSend(0XFF);

  // command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_CMD0);
      goto fail;
    }
  }

  // check SD version
  if ((cardCommand(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND)) {
    type_ = SD_CARD_TYPE_SD1;
  } else {
    // only need last byte of r7 response
    for (uint8_t i = 0; i < 4; i++) status_ = spiRec();
    if (status_ != 0XAA) {
      error(SD_CARD_ERROR_CMD8);
      goto fail;
    }
    type_ = SD_CARD_TYPE_SD2;
  }

  // initialize card and send host supports SDHC if SD2
  arg = type_ == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;
  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
      error(SD_CARD_ERROR_ACMD41);
      goto fail;
    }
  }

  // if SD2 read OCR register to check for SDHC card
  if (type_ == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto fail;
    }
    if ((spiRec() & 0XC0) == 0XC0) type_ = SD_CARD_TYPE_SDHC;
    // discard rest of ocr - contains allowed voltage range
    for (uint8_t i = 0; i < 3; i++) spiRec();
  }
  chipSelectHigh();
  return setSckRate(sckRateID);

 fail:
  chipSelectHigh();
  return false;
}

/**
 * Set the SPI clock rate to F_CPU/pow(2, 1 + sckRateID), sckRateID in [0, 6].
 * \return true for success, false for an invalid value of sckRateID.
 */
bool Sd2Card::setSckRate(uint8_t sckRateID) {
  if (sckRateID > 6) {
    error(SD_CARD_ERROR_SCK_RATE);
    return false;
  }
  spiRate_ = sckRateID;
  return true;
}

//------------------------------------------------------------------------------
// read the data part of a block (after a read command was accepted)
bool Sd2Card::readData(uint8_t* dst, uint16_t count) {
  // wait for start block token
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    error(SD_CARD_ERROR_READ);
    goto fail;
  }

  spiRead(dst, count);

#ifdef SD_CHECK_AND_RETRY
  {
    uint16_t calcCrc = CRC_CCITT(dst, count);
    uint16_t recvCrc = spiRec() << 8;
    recvCrc |= spiRec();
    if (calcCrc != recvCrc) {
      error(SD_CARD_ERROR_CRC);
      goto fail;
    }
  }
#else
  // discard CRC
  spiRec();
  spiRec();
#endif
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}

/**
 * Read a 512 byte block from an SD card.
 *
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 * \return true for success or false for failure.
 */
bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t* dst) {
  // use byte address if not SDHC card
  if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;

#ifdef SD_CHECK_AND_RETRY
  for (uint8_t tries = 3;;) {
    if (cardCommand(CMD17, blockNumber)) {
      error(SD_CARD_ERROR_CMD17);
    } else if (readData(dst, 512)) {
      return true;
    }
    if (--tries == 0) break;
    // try to stop any pending transfer, ignoring the result, then retry
    chipSelectHigh();
    cardCommand(CMD12, 0);
    errorCode_ = 0;
  }
  chipSelectHigh();
  return false;
#else
  if (cardCommand(CMD17, blockNumber)) {
    error(SD_CARD_ERROR_CMD17);
    chipSelectHigh();
    return false;
  }
  return readData(dst, 512);
#endif
}

//------------------------------------------------------------------------------
// send one block of data for a write block command
bool Sd2Card::writeData(uint8_t token, const uint8_t* src) {
  spiSendBlock(token, src);
  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc
  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    chipSelectHigh();
    return false;
  }
  return true;
}

/**
 * Write a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success or false for failure.
 */
bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src) {
  // use byte address if not SDHC card
  if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber)) {
    error(SD_CARD_ERROR_CMD24);
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) return false;  // CS already released

  // wait for flash programming to complete
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    error(SD_CARD_ERROR_WRITE_TIMEOUT);
    goto fail;
  }
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiRec()) {
    error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    goto fail;
  }
  chipSelectHigh();
  return true;

 fail:
  chipSelectHigh();
  return false;
}

#endif  // SDSUPPORT
