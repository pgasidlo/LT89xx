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

#ifndef LT89xx_H
#define LT89xx_H

#include <Arduino.h>

class LT89xx
{
  public:
    enum BitRate
    {
      BITRATE_UNKNOWN = 0,
      BITRATE_1MBPS,
      BITRATE_250KBPS,
      BITRATE_125KBPS,
      BITRATE_62KBPS
    };

    enum SyncWordLen
    {
      SYNCWORD_LEN_16 = 0,
      SYNCWORD_LEN_32 = 1,
      SYNCWORD_LEN_48 = 2,
      SYNCWORD_LEN_64 = 3
    };

    enum PreambleLen
    {
      PREAMBLE_LEN_1 = 1,
      PREAMBLE_LEN_2 = 2,
      PREAMBLE_LEN_3 = 3,
      PREAMBLE_LEN_4 = 4,
      PREAMBLE_LEN_5 = 5,
      PREAMBLE_LEN_6 = 6,
      PREAMBLE_LEN_7 = 7,
      PREAMBLE_LEN_8 = 8,
    };

    enum TrailerLen
    {
      TRAILER_LEN_4 = 0,
      TRAILER_LEN_6 = 1,
      TRAILER_LEN_8 = 2,
      TRAILER_LEN_10 = 3,
      TRAILER_LEN_12 = 4,
      TRAILER_LEN_14 = 5,
      TRAILER_LEN_16 = 6,
      TRAILER_LEN_18 = 7
    };

    enum PacketType
    {
      PACKET_TYPE_NRZ = 0,
      PACKET_TYPE_MANCHESTER = 1,
      PACKET_TYPE_8_10 = 2,
      PACKET_TYPE_INTERLEAVE = 3
    };

    enum FecType
    {
      FEC_NONE = 0,
      FEC_13 = 1,
      FEC_23 = 2
    };

    enum HardwareType
    {
      LT8900 = 0,
      LT8910 = 1,
      LT8920 = 2
    };

    enum State {
      STATE_IDLE = 0,
      STATE_TX = 1,
      STATE_RX = 2,
      STATE_SLEEP = 3,
    };

  private:
    uint8_t _csPin;
    uint8_t _pktPin;
    uint8_t _rstPin;
    uint8_t _channel;
    uint8_t _statusHigh;
    State _state;
    Stream* _debugStream;
    HardwareType _hardwareType;

  public:
    LT89xx() {}
    LT89xx(Stream &debugStream) : _debugStream(&debugStream) {};

    bool init(const uint8_t csPin, const uint8_t pktPin, const uint8_t rstPin, uint8_t channel = 0);

    void printRegisters();
    void printRegister(uint8_t reg);
    void printStatus();

    void setCurrentControl(uint8_t power, uint8_t gain);
    void setChannel(uint8_t channel);
    uint8_t getChannel();
    void setScramble(uint8_t seed);
    void setCrc(bool status, uint8_t init = 0);
    void setSyncWord(const uint64_t syncWord, SyncWordLen syncWordLen = SYNCWORD_LEN_64, uint8_t syncWordThreshold = 0);
    void setPacketFormat(PreambleLen preambleLen, TrailerLen trailerLen, PacketType packetType, FecType fecType);

    uint16_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint16_t data);
    void writeRegister(uint8_t reg, uint8_t high, uint8_t low);

    void sleep();

    /* Transmiter */
    int8_t send(void *data, int8_t length);

    /* Receiver */
    bool available();
    int8_t receive(void *data, int8_t maxLength);

    void startReceive();
    int8_t startSend(void *data, int8_t length);
    void whatHappend(uint8_t &txDone, uint8_t &rxReady);

    void idle();

  private:
    bool _init();
    void _startTX();
    void _startRX();
    BitRate _setBitRate(BitRate bitRate);
    BitRate _getBitRate();
};

#endif
