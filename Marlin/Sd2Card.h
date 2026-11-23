/**
 * Sd2Card - raw block access to SD / SDHC cards over the hardware SPI port
 * of the ATmega1280/2560.
 *
 * Based on Arduino Sd2Card Library, Copyright (C) 2009 by William Greiman
 * (GPL v3). Reduced to the features used by the Delta firmware.
 */
#ifndef SD2CARD_H
#define SD2CARD_H

#include "Marlin.h"

#ifdef SDSUPPORT

//------------------------------------------------------------------------------
// The hardware SPI pins (SS_PIN, MOSI_PIN, MISO_PIN, SCK_PIN) are defined in pins.h.
/** The default chip select pin for the SD card is SS. */
uint8_t const SD_CHIP_SELECT_PIN = SS_PIN;

// SPI clock is F_CPU / 2^(1 + rate)
uint8_t const SPI_FULL_SPEED = 0;  // F_CPU/2
uint8_t const SPI_HALF_SPEED = 1;  // F_CPU/4
/** SPI rate used while the card is being initialised (F_CPU/64) */
uint8_t const SPI_SD_INIT_RATE = 5;

//------------------------------------------------------------------------------
// Timeouts in milliseconds
uint16_t const SD_INIT_TIMEOUT = 2000;
uint16_t const SD_READ_TIMEOUT = 300;
uint16_t const SD_WRITE_TIMEOUT = 600;

//------------------------------------------------------------------------------
// Card types
uint8_t const SD_CARD_TYPE_SD1 = 1;   // standard capacity V1
uint8_t const SD_CARD_TYPE_SD2 = 2;   // standard capacity V2
uint8_t const SD_CARD_TYPE_SDHC = 3;  // high capacity

//------------------------------------------------------------------------------
// SD commands and responses
uint8_t const CMD0 = 0X00;    // GO_IDLE_STATE
uint8_t const CMD8 = 0X08;    // SEND_IF_COND
uint8_t const CMD12 = 0X0C;   // STOP_TRANSMISSION
uint8_t const CMD13 = 0X0D;   // SEND_STATUS
uint8_t const CMD17 = 0X11;   // READ_SINGLE_BLOCK
uint8_t const CMD24 = 0X18;   // WRITE_BLOCK
uint8_t const CMD55 = 0X37;   // APP_CMD
uint8_t const CMD58 = 0X3A;   // READ_OCR
uint8_t const ACMD41 = 0X29;  // SD_SEND_OP_COMD

uint8_t const R1_READY_STATE = 0X00;
uint8_t const R1_IDLE_STATE = 0X01;
uint8_t const R1_ILLEGAL_COMMAND = 0X04;

uint8_t const DATA_START_BLOCK = 0XFE;   // start token for read or write block
uint8_t const DATA_RES_MASK = 0X1F;      // write data response mask
uint8_t const DATA_RES_ACCEPTED = 0X05;  // write data accepted

//------------------------------------------------------------------------------
class Sd2Card
{
public:
  Sd2Card() : type_(0) {}

  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
            uint8_t chipSelectPin = SD_CHIP_SELECT_PIN);
  bool readBlock(uint32_t block, uint8_t *dst);
  bool writeBlock(uint32_t block, const uint8_t *src);

private:
  uint8_t chipSelectPin_;
  uint8_t spiRate_;
  uint8_t status_;
  uint8_t type_;

  bool setSckRate(uint8_t sckRateID);
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg)
  {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  bool readData(uint8_t *dst, uint16_t count);
  bool writeData(uint8_t token, const uint8_t *src);
  bool waitNotBusy(uint16_t timeoutMillis);
  void chipSelectHigh();
  void chipSelectLow();
};

#endif  // SDSUPPORT
#endif  // SD2CARD_H
