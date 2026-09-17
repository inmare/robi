#include "common/limit_switch.h"
#include "common/pusher_motor.h"
#include <AccelStepper.h>
#include <Arduino.h>

// X·Z 바꾼 상태. 푸셔 4핀은 X 칸. STEP D2, DIR D5
AccelStepper pusher(AccelStepper::DRIVER, PUSHER_STEP_PIN, PUSHER_DIR_PIN);

const int JOG_STEPS = 400;

static uint8_t lastLimitMask = 0;

static void haltNow() {
  pusher.setCurrentPosition(pusher.currentPosition());
}

static uint8_t limitMask() {
  uint8_t m = 0;
  if (limitLiftBottomPressed()) {
    m |= 1;
  }
  if (limitLiftTopPressed()) {
    m |= 2;
  }
  if (limitPusherBackPressed()) {
    m |= 4;
  }
  if (limitPusherFrontPressed()) {
    m |= 8;
  }
  return m;
}

static void printPressedLimits(uint8_t mask) {
  // 명칭은 test_limit_switch 와 같음: 하단 / 상단 / 후진 / 전진
  if (mask & 1) {
    Serial.print("하단 눌림 D");
    Serial.println(PIN_LIFT_BOTTOM);
  }
  if (mask & 2) {
    Serial.print("상단 눌림 D");
    Serial.println(PIN_LIFT_TOP);
  }
  if (mask & 4) {
    Serial.print("후진 눌림 D");
    Serial.println(PIN_PUSHER_BACK);
  }
  if (mask & 8) {
    Serial.println("전진 눌림 A0");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  limitSwitchInit();

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  pusher.setMaxSpeed(380);
  pusher.setAcceleration(150);
  lastLimitMask = limitMask();

  Serial.println("ready");
  Serial.println("limit switch 명칭 = 핀 (test_limit_switch 와 같음)");
  Serial.print("하단 D");
  Serial.println(PIN_LIFT_BOTTOM);
  Serial.print("상단 D");
  Serial.println(PIN_LIFT_TOP);
  Serial.print("후진 D");
  Serial.println(PIN_PUSHER_BACK);
  Serial.println("전진 A0");
  Serial.println("f 앞 / b 뒤, 조금씩");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') {
      return;
    }
    if (c == 'f' || c == 'F') {
      Serial.println("f, 앞");
      if (limitPusherFrontPressed()) {
        Serial.println("이미 전진 리미트 A0, 이동 안 함");
        return;
      }
      lastLimitMask = limitMask();
      pusher.move(JOG_STEPS * PUSHER_FORWARD);
    } else if (c == 'b' || c == 'B') {
      Serial.println("b, 뒤");
      if (limitPusherBackPressed()) {
        Serial.print("이미 후진 리미트 D");
        Serial.print(PIN_PUSHER_BACK);
        Serial.println(", 이동 안 함");
        return;
      }
      lastLimitMask = limitMask();
      pusher.move(JOG_STEPS * PUSHER_BACK);
    } else {
      Serial.print("무시: ");
      Serial.println(c);
      Serial.println("f 앞 / b 뒤");
    }
  }

  if (pusher.distanceToGo() != 0) {
    uint8_t mask = limitMask();
    uint8_t newly = mask & ~lastLimitMask;
    lastLimitMask = mask;
    if (newly) {
      haltNow();
      printPressedLimits(newly);
      Serial.println("정지");
      return;
    }
  }

  pusher.run();
}
