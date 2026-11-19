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
uint16_t const SD_INIT_TIMEOUT  = 2000;
uint16_t const SD_READ_TIMEOUT  = 300;
uint16_t const SD_WRITE_TIMEOUT = 600;

//------------------------------------------------------------------------------
// SD card errors
uint8_t const SD_CARD_ERROR_CMD0              = 0X01;  // CMD0 timeout (go idle)
uint8_t const SD_CARD_ERROR_CMD8              = 0X02;  // CMD8 rejected
uint8_t const SD_CARD_ERROR_CMD17             = 0X04;  // read block failed
uint8_t const SD_CARD_ERROR_CMD24             = 0X06;  // write block failed
uint8_t const SD_CARD_ERROR_CMD58             = 0X08;  // read OCR failed
uint8_t const SD_CARD_ERROR_ACMD41            = 0X0A;  // init timeout
uint8_t const SD_CARD_ERROR_READ              = 0X0F;  // error token on read
uint8_t const SD_CARD_ERROR_READ_TIMEOUT      = 0X11;  // timeout waiting for data
uint8_t const SD_CARD_ERROR_WRITE             = 0X13;  // error token on write
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X16;  // CMD13 failed after write
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT     = 0X17;  // programming timeout
uint8_t const SD_CARD_ERROR_SCK_RATE          = 0X18;  // invalid SPI rate
uint8_t const SD_CARD_ERROR_INIT_NOT_CALLED   = 0X19;  // init() not called
uint8_t const SD_CARD_ERROR_CRC               = 0X20;  // CRC mismatch

// Card types
uint8_t const SD_CARD_TYPE_SD1  = 1;  // standard capacity V1
uint8_t const SD_CARD_TYPE_SD2  = 2;  // standard capacity V2
uint8_t const SD_CARD_TYPE_SDHC = 3;  // high capacity

//------------------------------------------------------------------------------
// SD commands and responses
uint8_t const CMD0   = 0X00;  // GO_IDLE_STATE
uint8_t const CMD8   = 0X08;  // SEND_IF_COND
uint8_t const CMD12  = 0X0C;  // STOP_TRANSMISSION
uint8_t const CMD13  = 0X0D;  // SEND_STATUS
uint8_t const CMD17  = 0X11;  // READ_SINGLE_BLOCK
uint8_t const CMD24  = 0X18;  // WRITE_BLOCK
uint8_t const CMD55  = 0X37;  // APP_CMD
uint8_t const CMD58  = 0X3A;  // READ_OCR
uint8_t const ACMD41 = 0X29;  // SD_SEND_OP_COMD

uint8_t const R1_READY_STATE     = 0X00;
uint8_t const R1_IDLE_STATE      = 0X01;
uint8_t const R1_ILLEGAL_COMMAND = 0X04;

uint8_t const DATA_START_BLOCK   = 0XFE;  // start token for read or write block
uint8_t const DATA_RES_MASK      = 0X1F;  // write data response mask
uint8_t const DATA_RES_ACCEPTED  = 0X05;  // write data accepted

//------------------------------------------------------------------------------
class Sd2Card {
 public:
  Sd2Card() : errorCode_(SD_CARD_ERROR_INIT_NOT_CALLED), type_(0) {}

  bool init(uint8_t sckRateID = SPI_FULL_SPEED,
            uint8_t chipSelectPin = SD_CHIP_SELECT_PIN);
  bool readBlock(uint32_t block, uint8_t* dst);
  bool writeBlock(uint32_t block, const uint8_t* src);

 private:
  uint8_t chipSelectPin_;
  uint8_t errorCode_;
  uint8_t spiRate_;
  uint8_t status_;
  uint8_t type_;

  void error(uint8_t code) {errorCode_ = code;}
  bool setSckRate(uint8_t sckRateID);
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg) {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  bool readData(uint8_t* dst, uint16_t count);
  bool writeData(uint8_t token, const uint8_t* src);
  bool waitNotBusy(uint16_t timeoutMillis);
  void chipSelectHigh();
  void chipSelectLow();
};

#endif  // SDSUPPORT
#endif  // SD2CARD_H
