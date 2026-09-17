#include "common/lift_motor.h"
#include "common/limit_switch.h"
#include "common/sensor.h"
#include <AccelStepper.h>
#include <Arduino.h>

// 리프트 = 쉴드 X축. STEP D2, DIR D5
AccelStepper lift(AccelStepper::DRIVER, 2, 5);

const int EN_PIN = 8;

const char CMD_LIFT_UP = 'u';
const char CMD_LIFT_DOWN = 'd';

// test_lift_motor는 800 / 400. 살짝 빠르게.
const float LIFT_MAX_SPEED = 650;  // steps/s
const float LIFT_ACCEL = 320;      // steps/s^2
// 센서가 안 되면 너무 멀리 가지 않게.
const int MAX_LIFT_STEPS = 12000;

enum State { WAIT_CMD, LIFTING_UP, LIFTING_DOWN };
State state = WAIT_CMD;

unsigned long lastPrintMs = 0;
uint16_t lastMm = 0;

static void haltNow() {
  lift.setCurrentPosition(lift.currentPosition());
}

static bool distanceOk(uint16_t mm) {
  return !sensorTimeoutOccurred() && mm > 0 && mm < 8000;
}

static void startLiftUp() {
  uint16_t mm = sensorReadDistanceMm();
  if (distanceOk(mm) && mm <= TARGET_DISTANCE_MM) {
    Serial.print(mm);
    Serial.println(" mm, 이미 목표");
    return;
  }

  lift.move(MAX_LIFT_STEPS * LIFT_UP);
  state = LIFTING_UP;
  lastPrintMs = 0;
  Serial.print("상승 시작 (u), ");
  Serial.print(TARGET_DISTANCE_MM);
  Serial.println(" mm까지");
}

static void startLiftDown() {
  if (limitLiftBottomPressed()) {
    Serial.println("이미 하단 리미트, 이동 안 함");
    return;
  }

  lift.move(MAX_LIFT_STEPS * LIFT_DOWN);
  state = LIFTING_DOWN;
  lastPrintMs = 0;
  Serial.println("하강 시작 (d), 하단 리미트에서 정지");
}

void setup() {
  Serial.begin(115200);
  limitSwitchInit();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  lift.setMaxSpeed(LIFT_MAX_SPEED);
  lift.setAcceleration(LIFT_ACCEL);

  if (!sensorInit(500)) {
    Serial.println("센서 초기화 실패!");
    while (1) {
    }
  }

  Serial.print("u = 상승 ");
  Serial.print(TARGET_DISTANCE_MM);
  Serial.println(" mm까지");
  Serial.println("d = 하강, 하단 리미트에서 즉시 정지");
}

void loop() {
  if (state == WAIT_CMD) {
    if (!Serial.available()) {
      return;
    }
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') {
      return;
    }
    if (c == CMD_LIFT_UP || c == 'U') {
      startLiftUp();
    } else if (c == CMD_LIFT_DOWN || c == 'D') {
      startLiftDown();
    } else {
      Serial.println("u 상승 / d 하강");
    }
    return;
  }

  if (state == LIFTING_DOWN && limitLiftBottomPressed()) {
    haltNow();
    state = WAIT_CMD;
    Serial.println("하단 리미트, 즉시 정지");
    return;
  }

  if (state == LIFTING_UP && limitLiftTopPressed()) {
    haltNow();
    state = WAIT_CMD;
    Serial.println("상단 리미트, 즉시 정지");
    return;
  }

  if (state == LIFTING_UP && sensorRangeReady()) {
    uint16_t mm = sensorReadDistanceMm();
    lastMm = mm;
    if (distanceOk(mm) && mm <= TARGET_DISTANCE_MM) {
      haltNow();
      state = WAIT_CMD;
      Serial.print(mm);
      Serial.println(" mm, 목표 도달 정지");
      return;
    }
  }

  lift.run();

  if (state == LIFTING_UP && millis() - lastPrintMs > 300) {
    lastPrintMs = millis();
    Serial.print(lastMm);
    Serial.println(" mm");
  }

  if (lift.distanceToGo() == 0) {
    haltNow();
    state = WAIT_CMD;
    Serial.println("최대 스텝, 정지");
  }
}
