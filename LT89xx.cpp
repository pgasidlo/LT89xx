/*
 *  Copyright (C) 2020 Piotr Gasidlo, <quaker@barbara.eu.org>
 *
 *  Based on https://github.com/MINI-Qiang/LT8910
 *  Copyright (C) 2015 Rob van der Veer, <rob.c.veer@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 */

#include <SPI.h>
#include "LT89xx.h"

#define REGISTER_BITS(n) (1 << (n + 1) - 1)

#define REGISTER_7                          7
#define REGISTER_7_TX_EN                    (1 << 8)
#define REGISTER_7_RX_EN                    (1 << 7)
#define REGISTER_7_CHANNEL_MASK             REGISTER_BITS(7)

#define REGISTER_9                          9
#define REGISTER_9_POWER_MASK               REGISTER_BITS(4)
#define REGISTER_9_POWER_SHIFT              12
#define REGISTER_9_GAIN_MASK                REGISTER_BITS(4)
#define REGISTER_9_GAIN_SHIFT               7

#define REGISTER_35                         35
#define REGISTER_35_POWER_DOWN              (1 << 15)
#define REGISTER_35_SLEEP_MODE              (1 << 14)
#define REGISTER_35_BRCLK_ON_SLEEP          (1 << 12)
#define REGISTER_35_RETRANSMIT_TIMES_MASK   REGISTER_BITS(3)
#define REGISTER_35_RETRANSMIT_TIMES_SHIFT  8
#define REGISTER_35_MISO_TRI_OPT            (1 << 7)
#define REGISTER_35_SCRAMBLE_DATA_MASK      REGISTER_BITS(6)

#define REGISTER_41                         41
#define REGISTER_41_CRC_ON                  (1 << 15)
#define REGISTER_41_SCRAMBLE_ON             (1 << 14)
#define REGISTER_41_PACK_LEN_EN             (1 << 13)
#define REGISTER_41_FW_TERM_TX              (1 << 12)
#define REGISTER_41_AUTO_ACK                (1 << 11)
#define REGISTER_41_PKT_FIFO_POLARITY       (1 << 10)
#define REGISTER_41_CRC_INITIAL_DATA_MASK   REGISTER_BITS(8)

#define REGISTER_43
#define REGISTER_43_SCAN_RSSI_EN            (1 << 15)
#define REGISTER_43_SCAN_RSSI_EN            (1 << 15)

#define REGISTER_44                         44
#define REGISTER_44_DATARATE_MASK           REGISTER_BITS(8)
#define REGISTER_44_DATARATE_SHIFT          8
#define REGISTER_44_1MBPS                   (1 << 8)
#define REGISTER_44_250KBPS                 (1 << 10)
#define REGISTER_44_125KBPS                 (1 << 11)
#define REGISTER_44_62KBPS                  (1 << 12)

#define REGISTER_45                         45
#define REGISTER_45_1MBPS                   0x0152 /* or 0x0080 */
#define REGISTER_45_250KBPS                 0x0552
#define REGISTER_45_125KBPS                 0x0552
#define REGISTER_45_62KBPS                  0x0552

#define REGISTER_48                         48
#define REGISTER_48_CRC_ERROR               (1 << 15)
#define REGISTER_48_FEC23_ERROR             (1 << 14)
#define REGISTER_48_FRAMER_ST_MASK          REGISTER_BITS(6)
#define REGISTER_48_FRAMER_ST_SHIFT         8
#define REGISTER_48_SYNCWORD_RECV           (1 << 7)
#define REGISTER_48_PKT_FLAG                (1 << 6)
#define REGISTER_48_FIFO_FLAG               (1 << 5)

#define REGISTER_50                         50

#define REGISTER_52                         52
#define REGISTER_52_CLR_W_PTR               (1 << 15)
#define REGISTER_52_FIFO_WR_PTR_MASK        REGISTER_BITS(6)
#define REGISTER_52_FIFO_WR_PTR_SHIFT       8
#define REGISTER_52_CLR_R_PTR               (1 << 7)
#define REGISTER_52_FIFO_RD_PTR_MASK        REGISTER_BITS(6)

#define debug(input)   { if (_serial) _serial->print(input);   }
#define debugln(input) { if (_serial) _serial->println(input); }

bool LT89xx::init(const uint64_t syncWord, const uint8_t csnPin, const uint8_t pktPin, const uint8_t rstPin, BitRate bitRate, uint8_t channel)
{
  _csPin = csnPin;
  _pktPin = pktPin;
  _rstPin = rstPin;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  if (_rstPin) {
    pinMode(_rstPin, OUTPUT);
    pinMode(_rstPin, HIGH);
  }
  pinMode(_pktPin, INPUT);

  _init();
}

bool LT89xx::_init(BitRate bitRate, uint8_t channel)
{
  /* Reset, if RESET pin connected */
  if(_rstPin > 0) {
    digitalWrite(_rstPin, LOW);
    delay(200);
    digitalWrite(_rstPin, HIGH);
    delay(200);
  }

  /* Register 0 */
  writeRegister(0, 0x6fe0);

  /* Register 1 */
  writeRegister(1, 0x5681);

  /* Register 2 */
  writeRegister(2, 0x6617);

  /* Register 4 */
  writeRegister(4, 0x9cc9);

  /* Register 5 */
  writeRegister(5, 0x6637);

  /* Register 7 */
  writeRegister(7, 0x0300);

  /* Register 8 */
  writeRegister(8, 0x6c90);

  /* Register 9 */
  setCurrentControl(4, 0);

  /* Register 10 */
  writeRegister(10, 0x7ffd);

  /* Register 11 */
  writeRegister(11, 0x0000);

  /* Register 12 */
  writeRegister(12, 0x0000);

  /* Register 13 */
  writeRegister(13, 0x48bd);

  /* Register 22 */
  writeRegister(22, 0x00ff);

  /* Register 23 */
  writeRegister(23, 0x8005);

  /* Register 24 */
  writeRegister(24, 0x0067);

  /* Register 25 */
  writeRegister(25, 0x1659);

  /* Register 26 */
  writeRegister(26, 0x19e0);

  /* Register 27 */
  writeRegister(27, 0x1300);

  /* Register 28 */
  writeRegister(28, 0x1800);

  /* Register 29: RO: Stores p/n, version information */
  /* Register 30: RO: Stores p/n, version information */
  /* Register 31: RO: Stores p/n, version information */

  /* Register 32 */
  writeRegister(32, 0x5000);

  /* Register 33 */
  writeRegister(33, 0x3fc7);

  /* Register 34 */
  writeRegister(34, 0x2000);

  /* Register 35 */
  writeRegister(35, 0x0300);

  /* Register 36 */
  writeRegister(36, 0xdead);

  /* Register 37 */
  writeRegister(36, 0xbeaf);

  /* Register 38 */
  writeRegister(36, 0xdead);

  /* Register 39 */
  writeRegister(36, 0xbead);

  /* Register 40 */
  writeRegister(40, 0x4401);

  /* Register 41 */
  writeRegister(REGISTER_41, REGISTER_41_CRC_ON | REGISTER_41_PACK_LEN_ENABLE | REGISTER_41_FW_TERM_TX);

  /* Register 42 */
  writeRegister(42, 0xfdb0);

  /* Register 43 */
  writeRegister(43, 0x000f);

  /* Register 44 */
  writeRegister(44, 0x1000);

  /* Register 45 */
  _hardwareType = LT8900;
  if (_setBitRate(BITRATE_62KBPS) == BITRATE_62KBPS) {
    _hardwareType = LT8910;
  }
  if (_setBitRate(BITRATE_1MBPS) != BITRATE_1MBPS) {
    return false;
  }

  /* Register 50 */
  writeRegister(REGISTER_50, 0x0000);

  /* Register 52 */
  writeRegister(REGISTER_52, 0x8080);

  delay(200);

  setChannel(channel);
  _startSend();
}

void LT89xx::setCurrentControl(uint8_t power, uint8_t gain)
{
  writeRegister(
   REGISTER_9, 
   ((uint16_t)(power & REGISTER_9_POWER_MASK) << REGISTER_9_POWER_SHIFT) |
   ((uint16_t)(gain & REGISTER_9_GAIN_MASK) << REGISTER_9_GAIN_SHIFT)
  );
}

void LT89xx::setScramble(uint8_t seed)
{
  if (seed) {
    writeRegister(REGISTER_41, readRegister(REGISTER_41) | REGISTER_41_SCRAMBLE_ON);
    writeRegister(REGISTER_35, (readRegister(REGISTER_35) & ~REGISTER_35_SCRAMBLE_DATA_MASK) | ((uint16_t)( seed )));
  } else {
    writeRegister(REGISTER_41, (readRegister(REGISTER_41) & ~REGISTER_41_SCRAMBLE_ON));
  }
}

void LT89xx::setCrc(bool status)
{
  if (status) {
    writeRegister(REGISTER_41, (readRegister(REGISTER_41) | REGISTER_41_CRC_ON));
  } else {
    writeRegister(REGISTER_41, (readRegister(REGISTER_41) & ~REGISTER_41_CRC_ON));
  }
}

void LT89xx::setPacketFormat(PreambleLen preambleLen, TrailerLen trailerLen, PacketType packetType, FecType fecType)
{
}

uint16_t LT89xx::readRegister(uint8_t reg)
{
}

void LT89xx::writeRegister(uint8_t reg, uint16_t data)
{
}

void LT89xx::writeRegister(uint8_t reg, uint8_t high, uint8_t low)
{
}

void LT89xx::sleep()
{
}

/* Transmiter */
uint8_t LT89xx::send(void *data, uint8_t length)
{
}

/* Receiver */
bool LT89xx::available()
{
  return false;
}

uint8_t receive(void *data, uint8_t maxLength)
{
}

/* ISR */
bool LT89xx::availableISR()
{
}

bool LT89xx::startReceive()
{
  writeRegister(REGISTER_7, (readRegister(REGISTER_7) & ~(REGISTER_7_TX | REGISTER_7_RX)) | REGISTER_7_RX);
}

void LT89xx::_startSend(void *data, uint8_t length)
{
  writeRegister(REGISTER_7, (readRegister(REGISTER_7) & ~(REGISTER_7_TX | REGISTER_7_RX)) | REGISTER_7_TX);
}

void LT89xx::setChannel(uint8_t channel)
{
  writeRegister(REGISTER_7, (uint16_t)(_channel & REGISTER_7_TXRX_MASK));
}

uint16_t LT89xx::getChannel()
{
  return (uint8_t)(readRegister(REGISTER_7) & REGISTER_7_CHANNEL_MASK);
}


void LT89xx::whatHappend(uint8_t &txDone, uint8_t &rxReady)
{
}

BitRate _setBitRate(BitRate bitRate)
{
  uint16_t value = 0;

  /* Set */
  switch (bitRate)
  {
    case BITRATE_1MBPS:
      value = DATARATE_1MBPS;
      break;
    case BITRATE_250KBPS:
      value = DATARATE_250KBPS;
      break;
    case BITRATE_125KBPS:
      value = DATARATE_125KBPS;
      break;
    case BITRATE_62KBPS:
      value = DATARATE_62KBPS;
      break;
    default:
      return BITRATE_UNKNOWN;
  }

  writeRegister(REGISTER_44, (uint16_t)((readRegister(REGISTER_44) & ~(REGISTER_44_DATARATE_MASK << REGISTER_44_DATARATE_SHIFT)) | value));

  /* Verify if successfully set */
  if ((readRegister(REGISTER_44) & REGISTER_44_DATARATE_MASK) >> REGISTER_44_DATARATE_SHIFT == value) {
    return bitRate;
  }

  return _getBitRate();
}

BitRate _getBitRate()
{
  uint16_t value = (readRegister(REGISTER_44) & REGISTER_44_DATARATE_MASK) >> REGISTER_44_DATARATE_SHIFT;
  switch (value)
  {
    case REGISTER_44_1MBPS:
      return BITRATE_1MBPS;
    case REGISTER_44_250KBPS:
      return BITRATE_250KBPS;
    case REGISTER_44_125KBPS:
      return BITRATE_125KBPS;
    case REGISTER_44_62KBPS:
      return BITRATE_62KBPS;
  }
  return BITRATE_UNKNOWN;
}

void LT89xx::printRegisters()
{
  for (uint8_t reg = 0; reg <= 50; reg++) {
    LT89xx::printRegister(reg);
  }
}

void LT89xx::printRegister(uint8_t reg)
{
  uint16_t value = readRegister(reg);
  char sbuf[32];
  sprintf_P(sbuf, PSTR("%d: %02x%02x"), reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xff));
  debugln(sbuf);
}

void LT89xx::printStatus()
{
}


