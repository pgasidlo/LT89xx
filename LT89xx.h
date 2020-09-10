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

class LT89xx
{
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
      SYNCWORD_LEN_32,
      SYNCWORD_LEN_48,
      SYNCWORD_LEN_64
    };

    enum PreambleLen
    {
      PREAMBLE_LEN_1 = 1,
      PREAMBLE_LEN_2,
      PREAMBLE_LEN_3,
      PREAMBLE_LEN_4,
      PREAMBLE_LEN_5,
      PREAMBLE_LEN_6,
      PREAMBLE_LEN_7,
      PREAMBLE_LEN_8,
    };

    enum TrailerLen
    {
      TRAILER_LEN_4 = 0,
      TRAILER_LEN_6,
      TRAILER_LEN_8,
      TRAILER_LEN_10,
      TRAILER_LEN_12,
      TRAILER_LEN_14,
      TRAILER_LEN_16,
      TRAILER_LEN_18
    };

    enum PacketType
    {
      PACKET_TYPE_NRZ = 0,
      PACKET_TYPE_MANCHESTER,
      PACKET_TYPE_8_10,
      PACKET_TYPE_INTERLEAVE
    };

    enum FecType
    {
      FEC_NONE = 0,
      FEC_13,
      FEC_23
    };

    enum HardwareType
    {
      LT8900 = 0,
      LT8910 = 1,
      LT8920 = 2
    }

  private:
    uint8_t _csPin;
    uint8_t _pktPin;
    uint8_t _rstPin;
    uint8_t _channel;
    Serial* _serial;
    HardwareType _hardwareType;

  public:
    LT89xx() {}
    LT89xx(Stream &stream) : _serial(&serial) {};

    bool init(const uint64_t syncWord, const uint8_t pinChipSelect, const uint8_t pinPacketFlag, const uint8_t pinReset = 0, BitRate bitRate, uint8_t channel);

    void printRegisters();
    void printStatus();

    void setCurrentControl(uint8_t power, uint8_t gain);
    void setChannel(uint8_t channel);
    uint8_t getChannel();
    void setScramble(uint8_t seed);
    void setCrc(bool status);
    void setPacketFormat(PreambleLen preambleLen, TrailerLen trailerLen, PacketType packetType, FecType fecType);

    uint16_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint16_t data);
    void writeRegister(uint8_t reg, uint8_t high, uint8_t low);

    void sleep();

    /* Transmiter */
    uint8_t send(void *data, uint8_t length);

    /* Receiver */
    bool available();
    uint8_t receive(void *data, uint8_t maxLength);

    /* ISR */
    bool availableISR();
    bool startReceive();
    void startSend(void *data, uint8_t length);
    void whatHappend(uint8_t &txDone, uint8_t &rxReady);

  private:
    bool _init();
    BitRate _setBitRate(BitRate bitRate);
    BitRate _getBitRate();
    void _startSend();
}

#endif
