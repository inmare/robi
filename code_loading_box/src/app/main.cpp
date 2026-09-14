#include "common/sensor.h"
#include <AccelStepper.h>
#include <Arduino.h>

// ===== PIN MAP 상수들 =====
// // 리프트(X축) 모터
// const int SCREW_DIR_PIN = 2;
// const int SCREW_STEP_PIN = 3;

// // 푸셔(Y축) 모터
// const int PUSHER_DIR_PIN = 4;
// const int PUSHER_STEP_PIN = 5;
// ==========================

const int TARGET_DISTANCE_MM = 100;
const int MOTOR_STEP = 10000;

AccelStepper lift(AccelStepper::DRIVER, 2, 5);

unsigned long lastPrintMs = 0;

// SDA=A4, SCL=A5 (Uno 고정)
void setup() {
  Serial.begin(115200);
  if (!sensorInit(500)) {
    Serial.println("센서 초기화 실패!");
    while (1) {
    }
  }
  Serial.println("VL53L0X ready");

  // temp lift motor init code
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  // lift setting
  lift.setMaxSpeed(400);
  lift.setAcceleration(200);
  lift.moveTo(MOTOR_STEP);
}

void loop() {
  uint16_t mm = sensorReadDistanceMm();
  // 1초마다 센서 읽기 결과 출력
  if (millis() - lastPrintMs > 1000) {
    lastPrintMs = millis();
    Serial.print(mm);
    Serial.print(" mm");
    if (sensorTimeoutOccurred()) {
      Serial.print("TIMEOUT");
      lift.stop();
    }
    Serial.println();
  }
  // temp lift motor loop code
  if (mm <= TARGET_DISTANCE_MM) {
    lift.stop();
    Serial.println("Lift motor stopped");
  } else {
    lift.run();
    if (lift.distanceToGo() == 0) {
      lift.move(MOTOR_STEP);
    }
  }
}
