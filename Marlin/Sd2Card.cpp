/**
 * Sd2Card - raw block access to SD / SDHC cards over hardware SPI.
 *
 * Based on Arduino Sd2Card Library, Copyright (C) 2009 by William Greman
 * (GPL v3). Reduced to the features used by the Delta firmware.
 */
#include "Sd2Card.h"

// make sure SPCR rate is in expected bits
#if (SPR0 != 0 || SPR1 != 1)
#error unexpected SPCR bits
#endif

//------------------------------------------------------------------------------
// Hardware SPI helpers

/** Set SCK rate to F_CPU/pow(2, 1 + spiRate) for spiRate [0,6] */
static void spiInit(uint8_t spiRate)
{
  SPCR = (1 << SPE) | (1 << MSTR) | (spiRate >> 1);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : 1 << SPI2X;
}

static inline void spiWait()
{
  while (!(SPSR & (1 << SPIF))) { /* wait for transfer complete */
  }
}

/** SPI receive a byte */
static uint8_t spiRec()
{
  SPDR = 0XFF;
  spiWait();
  return SPDR;
}

/** SPI read data - only one call so force inline */
static inline __attribute__((always_inline)) void spiRead(uint8_t *buf, uint16_t nbyte)
{
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
static void spiSend(uint8_t b)
{
  SPDR = b;
  spiWait();
}

/** SPI send block - only one call so force inline */
static inline __attribute__((always_inline)) void spiSendBlock(uint8_t token, const uint8_t *buf)
{
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

//------------------------------------------------------------------------------
void Sd2Card::chipSelectHigh()
{
  digitalWrite(chipSelectPin_, HIGH);
}

void Sd2Card::chipSelectLow()
{
  spiInit(spiRate_);
  digitalWrite(chipSelectPin_, LOW);
}

// wait for card to go not busy
bool Sd2Card::waitNotBusy(uint16_t timeoutMillis)
{
  uint16_t t0 = millis();
  while (spiRec() != 0XFF) {
    if (((uint16_t)millis() - t0) >= timeoutMillis) return false;
  }
  return true;
}

// send command and return the R1 response byte (zero for OK)
uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg)
{
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
  for (uint8_t i = 0; ((status_ = spiRec()) & 0X80) && i != 0XFF; i++) { /* wait */
  }
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
bool Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin)
{
  type_ = 0;
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
      goto fail;
    }
    type_ = SD_CARD_TYPE_SD2;
  }

  // initialize card and send host supports SDHC if SD2
  arg = type_ == SD_CARD_TYPE_SD2 ? 0X40000000 : 0;
  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    if (((uint16_t)millis() - t0) > SD_INIT_TIMEOUT) {
      goto fail;
    }
  }

  // if SD2 read OCR register to check for SDHC card
  if (type_ == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
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
bool Sd2Card::setSckRate(uint8_t sckRateID)
{
  if (sckRateID > 6) {
    return false;
  }
  spiRate_ = sckRateID;
  return true;
}

//------------------------------------------------------------------------------
// read the data part of a block (after a read command was accepted)
bool Sd2Card::readData(uint8_t *dst, uint16_t count)
{
  // wait for start block token
  uint16_t t0 = millis();
  while ((status_ = spiRec()) == 0XFF) {
    if (((uint16_t)millis() - t0) > SD_READ_TIMEOUT) {
      goto fail;
    }
  }
  if (status_ != DATA_START_BLOCK) {
    goto fail;
  }

  spiRead(dst, count);

  // discard CRC
  spiRec();
  spiRec();
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
bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t *dst)
{
  // use byte address if not SDHC card
  if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;

  if (cardCommand(CMD17, blockNumber)) {
    chipSelectHigh();
    return false;
  }
  return readData(dst, 512);
}

//------------------------------------------------------------------------------
// send one block of data for a write block command
bool Sd2Card::writeData(uint8_t token, const uint8_t *src)
{
  spiSendBlock(token, src);
  spiSend(0xff);  // dummy crc
  spiSend(0xff);  // dummy crc
  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
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
bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t *src)
{
  // use byte address if not SDHC card
  if (type_ != SD_CARD_TYPE_SDHC) blockNumber <<= 9;
  if (cardCommand(CMD24, blockNumber)) {
    goto fail;
  }
  if (!writeData(DATA_START_BLOCK, src)) return false;  // CS already released

  // wait for flash programming to complete
  if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
    goto fail;
  }
  // response is r2 so get and check two bytes for nonzero
  if (cardCommand(CMD13, 0) || spiRec()) {
    goto fail;
  }
  chipSelectHigh();
  return true;

fail:
  chipSelectHigh();
  return false;
}
