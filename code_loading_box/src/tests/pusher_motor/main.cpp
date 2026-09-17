#include "common/pusher_motor.h"
#include <AccelStepper.h>
#include <Arduino.h>

// 푸셔 = 쉴드 Z축. STEP D4, DIR D7
AccelStepper pusher(AccelStepper::DRIVER, 4, 7);

void setup() {
  Serial.begin(115200);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  pusher.setMaxSpeed(800);        // steps/s
  pusher.setAcceleration(400);    // steps/s^2
  // 거리는 양수. 방향은 PUSHER_DIR. 중간에 둔 상태에서 조금만 움직임.
  pusher.moveTo(400 * PUSHER_DIR);

  Serial.println("푸셔 짧은 이동 시작");
}

void loop() {
  pusher.run();
}
