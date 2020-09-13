#include "LT89xx.h"

#define LT_CS_PIN 0
#define LT_PKT_PIN 0
#define LT_RST_PIN 0

LT89xx lt(Serial);

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP: START");
  lt.init(LT_CS_PIN, LT_PKT_PIN, LT_RST_PIN);
  lt.printRegisters();
  lt.printStatus();
  Serial.println("SETUP: END");
  Serial.println("SEND: T - to start transmit, R - to start receive");
  Serial.println("MODE: RECEIVER");
}

bool sender = false;

void printPacket(uint8_t *data, uint8_t length) {
  Serial.print("PACKET = {");
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

void loop() {
  static uint8_t counter = 0;
  
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'T') {
      sender = true;
      lt.idle();
      Serial.println("MODE: SENDER");
    } else {
      sender = false;
      lt.startReceive();
      Serial.println("MODE: RECEIVER");
    }
  }
  if (sender) {
    counter++;
    uint8_t packet[] = { counter, 0x01, counter, 0x02, counter, 0x03, counter };
    printPacket(packet, sizeof(packet));
    uint8_t result = lt.send(packet, sizeof(packet));
    if (result == sizeof(packet)) {
      Serial.println("SEND: OK");
    } else{
      Serial.println("SEND: ERROR");
    }
    delay(1000);  
  } else { /* receiver */
    if (lt.available()) {
      uint8_t packet[60];
      uint8_t result = lt.receive(packet, sizeof(packet));
      if (result > 0) {
        printPacket(packet, result);
      } else if (result < 0) {
        Serial.print("PACKET: ERROR");
      }
    }
    delay(200);
  }
}
