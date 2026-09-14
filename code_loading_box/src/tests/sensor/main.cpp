#include <Arduino.h>
#include "common/sensor.h"

void setup() {
  Serial.begin(115200);
  if (!sensorInit()) {
    Serial.println("센서 초기화 실패!");
    while (1) {}
  }
  Serial.println("VL53L0X ready");
}

void loop() {
  uint16_t mm = sensorReadDistanceMm();
  Serial.print(mm);
  Serial.print(" mm");
  if (sensorTimeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();
  delay(100);
}