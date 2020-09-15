#include "LT89xx.h"
#include <SPI.h>

#define LT_CS_PIN 10
#define LT_PKT_PIN 2
#define LT_RST_PIN A1

LT89xx lt;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  lt.init(LT_CS_PIN, LT_PKT_PIN, LT_RST_PIN);  
  //lt.setPacketFormat(LT89xx::PREAMBLE_LEN_2, LT89xx::TRAILER_LEN_4, LT89xx::PACKET_TYPE_8_10, LT89xx::FEC_13);
  //lt.setScramble(2);
  //lt.setSyncWord(0xdeadbeafdeadbeaf);
  Serial.println(F("R - start receive, S - goto sleep")); 
}

bool isRunning = false;
volatile bool isPacket = false;

void ltPacket() {
  isPacket = true;
}

void printPacket(uint8_t *data, uint8_t length) {
  Serial.print(F("PACKET = {"));
  char sbuf[32];
  for (int i = 0; i < length; i++) {
    sprintf_P(sbuf, PSTR("%02x"), data[i]);
    if (i > 0) {
      Serial.print(F(", "));
    }
    Serial.print(sbuf);
  }
  Serial.println(F("}"));
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'R') {
      isRunning = true;
      attachInterrupt(digitalPinToInterrupt(LT_PKT_PIN), ltPacket, RISING);      
      lt.startReceive();
    } else if (ch == 'S') {      
      detachInterrupt(digitalPinToInterrupt(LT_PKT_PIN));
      lt.sleep();
      isRunning = false;
    }
  }

  if (isRunning) {
    if (isPacket) {
      isPacket = false;
      uint8_t packet[60];
      int8_t result = lt.receive(packet, sizeof(packet));
      if (result > 0) {
        printPacket(packet, result);
      } else if (result < 0) {
        Serial.print(F("ERROR"));
      }      
      lt.startReceive();
    }
  }
  delay(10);
}
