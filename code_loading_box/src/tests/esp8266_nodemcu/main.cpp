#include <Arduino.h>
#include <SoftwareSerial.h>

// D6=RX(우노 Hold), D5=TX(우노 Resume)
SoftwareSerial uno(D6, D5);

void setup() {
  Serial.begin(115200);
  uno.begin(9600);
  Serial.println("nodemcu ready");
}

void loop() {
  while (uno.available()) {
    String line = uno.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }
    Serial.print("from uno: ");
    Serial.println(line);

    if (line == "PING") {
      uno.println("PONG");
      Serial.println("-> PONG");
    } else {
      uno.print("ECHO ");
      uno.println(line);
    }
  }
}
