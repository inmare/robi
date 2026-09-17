#include <Arduino.h>
#include <SoftwareSerial.h>

// CNC 쉴드 Resume=A2(RX), Hold=A1(TX). NodeMCU D5→A2, D6←A1(분압).
SoftwareSerial esp(A2, A1);

void setup() {
  Serial.begin(115200);
  esp.begin(9600);
  Serial.println("esp8266 uart test");
  Serial.println("PC에서 문자 치면 esp로 보냄. esp 응답은 여기로 찍힘");
  Serial.println("p = ping 보내기");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      return;
    }
    if (c == 'p' || c == 'P') {
      esp.println("PING");
      Serial.println("-> PING");
      return;
    }
    esp.write(c);
    Serial.print("-> ");
    Serial.println(c);
  }

  while (esp.available()) {
    char c = esp.read();
    Serial.write(c);
  }
}
