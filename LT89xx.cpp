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

#define REGISTER_MASK(n) (1 << (n + 1) - 1)
#define REGISTER_MASK_RANGE(u,l) (1 << (u - l + 2) - 1)

#define REGISTER_READ  (1 << 7)
#define REGISTER_WRITE 0
#define REGISTER_MASK  REGISTER_MASK(7)

#define REGISTER_3                              3
#define REGISTER_3_RF_SYNTH_LOCK                (1 << 12)

#define REGISTER_6                              6
#define REGISTER_6_RAW_RSSI_MASK                REGISTER_MASK_RANGE(15,10)
#define REGISTER_6_RAW_RSSI_SHIFT               10

#define REGISTER_7                              7
#define REGISTER_7_TX_EN                        (1 << 8)
#define REGISTER_7_RX_EN                        (1 << 7)
#define REGISTER_7_RF_PLL_CH_NO_MASK            REGISTER_MASK_RANGE(6,0)
#define REGISTER_7_RF_PLL_CH_NO_SHIFT           0

#define REGISTER_9                              9
#define REGISTER_9_PA_PWCTR_MASK                REGISTER_MASK_RANGE(15,12)
#define REGISTER_9_PA_PWCTR_SHIFT               12
#define REGISTER_9_PA_GN_MASK                   REGISTER_MASK_RANGE(10,7)
#define REGISTER_9_PA_GN_SHIFT                  7

#define REGISTER_10                             10
#define REGISTER_10_XTAL_OSC_EN                 (1 << 0)

#define REGISTER_11                             1
#define REGISTER_11_RSII_PDN                    (1 << 8)

#define REGISTER_23                             23
#define REGISTER_23_TXRX_VCA_CAL_EN             (1 << 2)

#define REGISTER_27                             27
#define REGISTER_27_XI_TRIM_MASK                REGISTER_MASK_RANGE(5,0)
#define REGISTER_27_XI_TRIM_SHIFT               0

#define REGISTER_29                             29
#define REGISTER_29_RF_VER_ID_MASK              REGISTER_MASK_RANGE(7,4)
#define REGISTER_29_RF_VER_ID_SHIFT             4
#define REGISTER_20_DIGITAL_VERSION             REGISTER_MASK_RANGE(2,0)

#define REGISTER_30                             30
#define REGISTER_30_ID_CODE_L_MASK              REGISTER_MASK_RANGE(15,0)
#define REGISTER_MASK_RANGE_L_SHIFT             0

#define REGISTER_31                             31
#define REGISTER_31_RF_CODE_ID_MASK             REGISTER_MASK_RANGE(15,12)
#define REGISTER_31_RF_CODE_ID_SHIFT            12
#define REGISTER_31_ID_CODE_M_MASK              REGISTER_MASK_RANGE(11,0)
#define REGISTER_31_ID_CODE_M_SHIFT             0

#define REGISTER_32                             32
#define REGISTER_32_PREAMBLE_LEN_MASK           REGISTER_MASK_RANGE(15,13)
#define REGISTER_32_PREAMBLE_LEN_SHIFT          13
#define REGISTER_32_SYNCWORD_LEN_MASK           REGISTER_MASK_RANGE(12,11)
#define REGISTER_32_SYNCWORD_LEN_SHIFT          11
#define REGISTER_32_TRAILER_LEN_MASK            REGISTER_MASK_RANGE(10,8)
#define REGISTER_32_TRAILER_LEN_SHIFT           8
#define REGISTER_32_DATA_PACKET_TYPE_MASK       REGISTER_MASK_RANGE(7,6)
#define REGISTER_32_DATA_PACKET_TYPE_SHIFT      6
#define REGISTER_32_FEC_TYPE_MASK               REGISTER_MASK_RANGE(5,4)
#define REGISTER_32_FEC_TYPE_SHIFT              4
#define REGISTER_32_BRCLK_SEL_MASK              REGISTER_MASK_RANGE(3,1)
#define REGISTER_32_BRCLK_SEL_SHIFT             1

#define REGISTER_33                             33
#define REGISTER_33_VCO_ON_DELAY_CNT_MASK       REGISTER_MASK_RANGE(15,8)
#define REGISTER_33_VCO_ON_DELAY_CNT_SHIFT      8
#define REGISTER_33_TX_PA_OFF_DELAY_MASK        REGISTER_MASK_RANGE(7,6)
#define REGISTER_33_TX_PA_OFF_DELAY_SHIFT       6
#define REGISTER_33_TX_PA_ON_DELAY_MASK         REGISTER_MASK_RANGE(5,0)
#define REGISTER_33_TX_PA_ON_DELAY_SHIFT        0

#define REGISTER_34                             34
#define REGISTER_34_BPKTCTL_DIRECT              (1 << 15)
#define REGISTER_34_TX_CW_DLY_MASK              REGISTER_MASK_RANGE(14,8)
#define REGISTER_34_TX_CW_DLY_SHIFT             8
#define REGISTER_34_TX_SW_ON_DELAY_MASK         REGISTER_MASK_RANGE(5,0)
#define REGISTER_34_TX_SW_ON_DELAY_SHIFT        0

#define REGISTER_35                             35
#define REGISTER_35_POWER_DOWN                  (1 << 15)
#define REGISTER_35_SLEEP_MODE                  (1 << 14)
#define REGISTER_35_BRCLK_ON_SLEEP              (1 << 12)
#define REGISTER_35_RETRANSMIT_TIMES_MASK       REGISTER_MASK_RANGE(11,8)
#define REGISTER_35_RETRANSMIT_TIMES_SHIFT      8
#define REGISTER_35_MISO_TRI_OPT                (1 << 7)
#define REGISTER_35_SCRAMBLE_DATA_MASK          REGISTER_MASK_RANGE(6,0)
#define REGISTER_35_SCRAMBLE_DATA_SHIFT         0

/* SYNC_WORD[15:0] */
#define REGISTER_36                             36
/* SYNC_WORD[31:16] */
#define REGISTER_37                             37
/* SYNC_WORD[47:32] */
#define REGISTER_38                             38
/* SYNC_WORD[63:48] */
#define REGISTER_39                             39

#define REGISTER_40                             40
#define REGISTER_40_FIFO_EMPTY_THRESHOLD_MASK   REGISTER_MASK_RANGE(15,11)
#define REGISTER_40_FIFO_EMPTY_THRESHOLD_SHIFT  11
#define REGISTER_40_FIFO_FULL_THRESHOLD_MASK    REGISTER_MASK_RANGE(10,6)
#define REGISTER_40_FIFO_FULL_THRESHOLD_SHIFT   6
#define REGISTER_40_SYNCWORD_THRESHOLD_MASK     REGISTER_MASK_RANGE(5,0)
#define REGISTER_40_SYNCWORD_THRESHOLD_SHIFT    0

#define REGISTER_41                             41
/*
 * 1 - CRC: on
 * 0 - CRC: off
 */
#define REGISTER_41_CRC_ON                      (1 << 15)
/*
 * Removes long patterns ofcontinius 0 or 1 in transmit data.
 * Automatically restores original unscrambled data on receive.
 *
 * 1 - scramble: on
 * 0 - scramble: off
 */
#define REGISTER_41_SCRAMBLE_ON                 (1 << 14)
/*
 * 1 - LT8910 regards first byte of payload as packet length descriptor byte
 */
#define REGISTER_41_PACK_LEN_EN                 (1 << 13)
/*
 * 1 - When FIFO write point equals read point, LT8910 will terminate TX when FW handle packet length.
 * 0 - FW (MCU) handles length and terminates TX.
 */
#define REGISTER_41_FW_TERM_TX                  (1 << 12)
/*
 * 1 - After receiving data, automatically send ACK/NACK
 * 0 - After receive, do not send ACK or NACK, just go to IDLE
 */
#define REGISTER_41_AUTO_ACK                    (1 << 11)
/*
 * 1 - PKT flag, FIFO flag: active low
 * 0 - PKT flag, FIFO flag: active hight
 */
#define REGISTER_41_PKT_FIFO_POLARITY           (1 << 10)
/*
 * Initialization constraint for CRC calculation
 */
#define REGISTER_41_CRC_INITIAL_DATA_MASK       REGISTER_MASK_RANGE(7,0)
#define REGISTER_41_CRC_INITIAL_DATA_SHIFT      0

/*
 * Number of consecutive channells to scan for RSSI value.
 * RSSI result of each channel is return in FIFO registers.
 */
#define REGISTER_42                             42
#define REGISTER_42_SCAN_RSSI_CH_NO_MASK        REGISTER_MASK_RANGE(15,10)
#define REGISTER_42_SCAN_RSSI_CH_NO_SHIFT       10
/*
 * Wait RX_ACK ID timer setting.
 * 1 represents 1us, 2 represents 2us, etc.
 */
#define REGISTER_42_RX_ACK_TIME_MASK            REGISTER_MASK_RANGE(7,0)
#define REGISTER_42_RX_ACK_TIME_SHIFT           0

#define REGISTER_43                             43
/* Start scan RSSI process */
#define REGISTER_43_SCAN_RSSI_EN                (1 << 15)
/*
 * Normally an RSSI scan would start at 2402 Mhz (channel 0).
 * This field introduces a starting offset.
 * For example, if offset = +10 than starting channel will be 2412 Mhz (channel 10).
 */
#define REGISTER_43_SCAN_STRT_CH_OFFST_MASK     REGISTER_MASK_RANGE(14,8)
#define REGISTER_43_SCAN_STRT_CH_OFFST_SHIFT    8
/*
 * Set VCO & SYNC setting time when scan diferent channel.
 */
#define REGISTER_43_WAIT_RSSI_SCAN_TIM_MASK     REGISTER_MASK_RANGE(7,0)
#define REGISTER_43_WAIT_RSSI_SCAN_TIM_SHIFT    0

#define REGISTER_44                             44
#define REGISTER_44_DATARATE_MASK               REGISTER_MASK_RANGE(15,8)
#define REGISTER_44_DATARATE_SHIFT              8
#define REGISTER_44_1MBPS                       (1 << 8)
#define REGISTER_44_250KBPS                     (1 << 10)
#define REGISTER_44_125KBPS                     (1 << 11)
#define REGISTER_44_62KBPS                      (1 << 12)

#define REGISTER_45                             45
#define REGISTER_45_1MBPS                       0x0152 /* or 0x0080 */
#define REGISTER_45_250KBPS                     0x0552
#define REGISTER_45_125KBPS                     0x0552
#define REGISTER_45_62KBPS                      0x0552

#define REGISTER_48                             48
/* Received CRC error */
#define REGISTER_48_CRC_ERROR                   (1 << 15)
/* Indicate FEC32 error */
#define REGISTER_48_FEC23_ERROR                 (1 << 14)
/* Framer status */
#define REGISTER_48_FRAMER_ST_MASK              REGISTER_MASK_RANGE(13,8)
#define REGISTER_48_FRAMER_ST_SHIFT             8
/* 
 * 1: syncword received, it is just available in recive status.
 * After out receive status always keep 0.
 */
#define REGISTER_48_SYNCWORD_RECV               (1 << 7)
/* PKT flag indicator */
#define REGISTER_48_PKT_FLAG                    (1 << 6)
/* FIFO flag indicator */
#define REGISTER_48_FIFO_FLAG                   (1 << 5)

#define REGISTER_50                             50

#define REGISTER_52                             52
#define REGISTER_52_CLR_W_PTR                   (1 << 15)
#define REGISTER_52_FIFO_WR_PTR_MASK            REGISTER_MASK_RANGE(13,8)
#define REGISTER_52_FIFO_WR_PTR_SHIFT           8
#define REGISTER_52_CLR_R_PTR                   (1 << 7)
#define REGISTER_52_FIFO_RD_PTR_MASK            REGISTER_MASK_RANGE(5,0)
#define REGISTER_52_FIFO_RD_PTR_SHIFT           0

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

  writeRegister(0, 0x6fe0);
  writeRegister(1, 0x5681);
  writeRegister(2, 0x6617);
  writeRegister(4, 0x9cc9);
  writeRegister(5, 0x6637);
  writeRegister(7, 0x0300);
  writeRegister(8, 0x6c90);
  writeRegister(9, 0x4800);
  writeRegister(10, 0x7ffd);
  writeRegister(11, 0x0000);
  writeRegister(12, 0x0000);
  writeRegister(13, 0x48bd);
  writeRegister(22, 0x00ff);
  writeRegister(23, 0x8005);
  writeRegister(24, 0x0067);
  writeRegister(25, 0x1659);
  writeRegister(26, 0x19e0);
  writeRegister(27, 0x1300);
  writeRegister(28, 0x1800);
  writeRegister(32, 0x5000);
  writeRegister(33, 0x3fc7);
  writeRegister(34, 0x2000);
  writeRegister(35, 0x0300);
  writeRegister(36, 0xdead);
  writeRegister(36, 0xbeaf);
  writeRegister(36, 0xdead);
  writeRegister(36, 0xbead);
  writeRegister(40, 0x4401);
  writeRegister(41, 0xb000);
  writeRegister(42, 0xfdb0);
  writeRegister(43, 0x000f);
  writeRegister(44, 0x1000);
  writeRegister(45, 0x0080);
  writeRegister(50, 0x0000);
  writeRegister(52, 0x8080);

  writeRegister(REGISTER_41, REGISTER_41_CRC_ON | REGISTER_41_PACK_LEN_ENABLE | REGISTER_41_FW_TERM_TX);

  _hardwareType = LT8900;
  if (_setBitRate(BITRATE_62KBPS) == BITRATE_62KBPS) {
    _hardwareType = LT8910;
  }
  if (_setBitRate(BITRATE_1MBPS) != BITRATE_1MBPS) {
    return false;
  }

  setChannel(channel);
  setCurrentControl(4, 0);
  _startSend();
}

void LT89xx::setCurrentControl(uint8_t power, uint8_t gain)
{
  writeRegister(
   REGISTER_9,
   ((uint16_t)(power & REGISTER_9_PA_PWCTR_MASK) << REGISTER_9_PA_PWCTR_SHIFT) |
   ((uint16_t)(gain & REGISTER_9_PA_GN_MASK) << REGISTER_9_PA_GN_SHIFT)
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
  writeRegister(REGISTER_32,
      (
       (preambleLen & REGISTER_32_PREAMBLE_LEN_MASK) << REGISTER_32_PREAMBLE_LEN_SHIFT |
       (trailerLen & REGISTER_32_TRAILER_LEN_MASK) << REGISTER_32_TRAILER_LEN_SHIFT |
       (packetType & REGISTER_32_DATA_PACKET_TYPE_MASK) << REGISTER_32_DATA_PACKET_TYPE_SHIFT |
       (fecType & REGISTER_32_FEC_TYPE_MASK) << REGISTER_32_FEC_TYPE_SHIFT
      )
  );
}

uint16_t LT89xx::readRegister(uint8_t reg)
{
  digitalWrite(_csPin, LOW);
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE1));
  SPI.transfer(REGISTER_READ | (reg & REGISTER_MASK));
  uint8_t high = SPI.transfer(0x00);
  uint8_t low = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(_csPin, HIGH);
  return (uint16_t)(high << 8 | low);
}

void LT89xx::writeRegister(uint8_t reg, uint16_t data)
{
  writeRegister(reg, (uint8_t)(data >> 8), (uint8_t)(data & 0xff));
}

void LT89xx::writeRegister(uint8_t reg, uint8_t high, uint8_t low)
{
  digitalWrite(_csPin, LOW);
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE1));
  uint8_t result = SPI.transfer(REGISTER_WRITE | (reg & REGISTER_MASK));
  SPI.transfer(high);
  SPI.transfer(low);
  SPI.endTransaction();
  digitalWrite(_csPin, HIGH);
  return result;
}

void LT89xx::sleep()
{
  writeRegister(REGISTER_35, (readRegister(REGISTER_35) | REGISTER_35_SLEEP_MODE));
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
  writeRegister(REGISTER_7, (readRegister(REGISTER_7) & ~(REGISTER_7_TX_EN | REGISTER_7_RX_EN)) | REGISTER_7_RX_EN);
}

void LT89xx::_startSend(void *data, uint8_t length)
{
  writeRegister(REGISTER_7, (readRegister(REGISTER_7) & ~(REGISTER_7_TX_EN | REGISTER_7_RX_EN)) | REGISTER_7_TX_EN);
}

void LT89xx::setChannel(uint8_t channel)
{
  writeRegister(REGISTER_7, (uint16_t)(_channel & REGISTER_7_RF_PLL_CH_NO_MASK) << REGISTER_7_RF_PLL_CH_NO_MASK);
}

uint16_t LT89xx::getChannel()
{
  return (uint8_t)((readRegister(REGISTER_7) & REGISTER_7_RF_PLL_CH_NO_MASK) >> REGISTER_7_RF_PLL_CH_NO_SHIFT);
}


void LT89xx::whatHappend(uint8_t &txDone, uint8_t &rxReady)
{
}

BitRate _setBitRate(BitRate bitRate)
{
  uint16_t value1 = 0;

  /* Set */
  switch (bitRate)
  {
    case BITRATE_1MBPS:
      value1 = DATARATE_1MBPS;
      value2 = REGISTER_45_1MBPS;
      break;
    case BITRATE_250KBPS:
      value1 = DATARATE_250KBPS;
      value2 = REGISTER_45_250KBPS;
      break;
    case BITRATE_125KBPS:
      value1 = DATARATE_125KBPS;
      value2 = REGISTER_45_125KBPS;
      break;
    case BITRATE_62KBPS:
      value1 = DATARATE_62KBPS;
      value2 = REGISTER_45_62KBPS;
      break;
    default:
      return BITRATE_UNKNOWN;
  }

  writeRegister(REGISTER_44, readRegister(REGISTER_44) & ~(REGISTER_44_DATARATE_MASK << REGISTER_44_DATARATE_SHIFT) | value1);

  /* Verify if successfully set */
  if ((readRegister(REGISTER_44) & REGISTER_44_DATARATE_MASK) >> REGISTER_44_DATARATE_SHIFT == value1) {
    writeRegister(REGISTER_45, value2);
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
  char sbuf[32];
  uint16_t value = readRegister(reg);
  sprintf_P(sbuf, PSTR("%d: %02x%02x"), reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xff));
  debugln(sbuf);
}

void LT89xx::printStatus()
{
  char sbuf[32];
  uint8_t value = readRegister(REGISTER_48);
  sprintf_P(sbuf, PSTR("CRC_ERROR: %d"), (uint8_t)(value & REGISTER_48_CRC_ERROR));
  debugln(sbuf);
  sprintf_P(sbuf, PSTR("FEC32_ERROR: %d"), (uint8_t)(value & REGISTER_48_FEC32_ERROR));
  debugln(sbuf);
  sprintf_P(sbuf, PSTR("FRAMER_ST: %02d"), (uint8_t)((value >> REGISTER_48_FRAMER_ST_SHIFT) & REGISTER_48_FRAMER_ST_MASK));
  debugln(sbuf);
  sprintf_P(sbuf, PSTR("SYNCWORD_RECV: %d"), (uint8_t)(value & REGISTER_48_SYNCWORD_RECV));
  debugln(sbuf);
  sprintf_P(sbuf, PSTR("PKT_FLAG: %d"), (uint8_t)(value & REGISTER_48_PKT_FLAG));
  debugln(sbuf);
  sprintf_P(sbuf, PSTR("FIFO_FLAG: %d"), (uint8_t)(value & REGISTER_48_FIFO_FLAG));
  debugln(sbuf);
}


