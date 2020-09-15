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
  Serial.println(F("T - start transmit, S - goto sleep"));
}

bool isRunning = false;

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
  static uint8_t counter = 0;

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'T') {
      lt.idle();
      isRunning = true;
    } else if (ch == 'S') {      
      lt.sleep();
      isRunning = false;
    }
  }

  if (isRunning) {  
    counter++;    
    uint8_t packet[] = { counter, 0x01, counter, 0x02, counter, 0x03, counter };    
    printPacket(packet, sizeof(packet));        
    int8_t result = lt.send(packet, sizeof(packet));
    if (result != sizeof(packet)) {
      Serial.println(F("ERROR"));   
    }
  } 
  delay(1000);    
}
