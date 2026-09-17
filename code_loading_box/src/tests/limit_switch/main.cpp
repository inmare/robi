#include <Arduino.h>
#include "common/limit_switch.h"

static void printOne(const char *name, bool pressed) {
  Serial.print(name);
  Serial.print(pressed ? "눌림" : "열림");
}

void setup() {
  Serial.begin(115200);
  limitSwitchInit();
  Serial.println("limit switch ready (눌리면 LOW)");
}

void loop() {
  printOne("하단=", limitLiftBottomPressed());
  Serial.print("  ");
  printOne("상단=", limitLiftTopPressed());
  Serial.print("  ");
  printOne("후진=", limitPusherBackPressed());
  Serial.print("  ");
  printOne("전진=", limitPusherFrontPressed());
  Serial.println();
  delay(100);
}
