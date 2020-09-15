#include "LT89xx.h"
#include <SPI.h>

#define LT_CS_PIN 10
#define LT_PKT_PIN 2
#define LT_RST_PIN A1

//LT89xx lt(Serial);
LT89xx lt;

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP: START");

  SPI.begin();
  lt.init(LT_CS_PIN, LT_PKT_PIN, LT_RST_PIN);  
  //lt.setPacketFormat(LT89xx::PREAMBLE_LEN_2, LT89xx::TRAILER_LEN_4, LT89xx::PACKET_TYPE_8_10, LT89xx::FEC_13);
  //lt.setScramble(2);
  //lt.printRegisters();
  //lt.printStatus();
  Serial.println("SETUP: END");
  Serial.println("SEND: T - to start transmit, R - to start receive, S - status");  
}

enum {
  MODE_NONE = 0,
  MODE_RECEIVER = 1,
  MODE_SENDER = 2,
};

uint8_t mode = MODE_NONE;

void printPacket(uint8_t *data, uint8_t length) {
  Serial.print("{");
  char sbuf[32];
  for (int i = 0; i < length; i++) {
    sprintf_P(sbuf, PSTR("%02x"), data[i]);
    if (i > 0) {
      Serial.print(", ");
    }
    Serial.print(sbuf);
  }
  Serial.println("}");
}

bool useInterrupts = false;

volatile bool _pktFlag = false;
void ltInterrupt()
{
  _pktFlag = true;
}

void loop() {
  static uint8_t counter = 0;

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'T') {
      attachInterrupt(digitalPinToInterrupt(LT_PKT_PIN), ltInterrupt, RISING);
      Serial.println("MODE: SENDER");       
      lt.idle();
      mode = MODE_SENDER;      
    } else if (ch == 'R') {
      Serial.println("MODE: RECEIVER");            
      mode = MODE_RECEIVER;
      lt.startReceive();
      attachInterrupt(digitalPinToInterrupt(LT_PKT_PIN), ltInterrupt, RISING);
    } else if (ch == 'S') {      
      Serial.println("MODE: SLEEP");            
      detachInterrupt(digitalPinToInterrupt(LT_PKT_PIN));
      lt.sleep();
      mode = MODE_NONE;            
    } else if (ch == '?') {
      lt.printStatus();   
      lt.printRegisters();        
    }
  }

  if (mode == MODE_SENDER) { /* sender */
    if (_pktFlag) {
      _pktFlag = false;
      counter++;    
      uint8_t packet[] = { counter, 0x01, counter, 0x02, counter, 0x03, counter };    
      Serial.print("SENDING: ");
      printPacket(packet, sizeof(packet));        

//      int8_t result = lt.send(packet, sizeof(packet));
//      if (result != sizeof(packet)) {
//        Serial.println("SEND: ERROR");
//      }    
      
      lt.startSend(packet, sizeof(packet));      
    }
    delay(1);
  } else if (mode == MODE_RECEIVER) { /* receiver */    
    if (
      _pktFlag ||
      lt.available() ||
      false
    ) {
      _pktFlag = false;
      uint8_t packet[60];
      Serial.print("RECEIVED: ");
      int8_t result = lt.receive(packet, sizeof(packet));
      if (result > 0) {
        printPacket(packet, result);
      } else if (result < 0) {
        Serial.print("ERROR");
      }      
      lt.startReceive();
    }    
    delay(1);
  } else {
    delay(500);
  }    
}
