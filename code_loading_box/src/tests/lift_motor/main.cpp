#include "common/lift_motor.h"
#include "common/limit_switch.h"
#include "common/sensor.h"
#include <AccelStepper.h>
#include <Arduino.h>

// X·Z 드라이버 바꿔 꽂은 상태. 리프트 4핀은 Z 칸. STEP D4, DIR D7
AccelStepper lift(AccelStepper::DRIVER, 4, 7);

static bool sensorOk = false;

static bool distanceOk(uint16_t mm) {
    return !sensorTimeoutOccurred() && mm > 0 && mm < 8000;
}

static bool liftAlreadyAtUpStop() {
    if (limitLiftTopPressed()) {
        Serial.println("이미 상단 리미트, 이동 안 함");
        return true;
    }
    if (!sensorOk) {
        return false;
    }
    uint16_t mm = sensorReadDistanceMm();
    if (distanceOk(mm) && mm <= TARGET_DISTANCE_MM) {
        Serial.print(mm);
        Serial.println(" mm, 이미 목표, 이동 안 함");
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    limitSwitchInit();

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    lift.setMaxSpeed(800); // steps/s
    lift.setAcceleration(400); // steps/s^2

    sensorOk = sensorInit(500);
    if (!sensorOk) {
        Serial.println("센서 초기화 실패, ToF 없이 u/d 만 동작");
    }

    Serial.println("ready");
    Serial.println("u 상승 / d 하강");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n' || c == ' ') {
            return;
        }
        if (c == 'u' || c == 'U') {
            Serial.println("u, 상승");
            if (liftAlreadyAtUpStop()) {
                return;
            }
            lift.setMaxSpeed(550);
            lift.setAcceleration(220);
            lift.move(12000 * LIFT_UP);
        } else if (c == 'd' || c == 'D') {
            Serial.println("d, 하강");
            if (limitLiftBottomPressed()) {
                Serial.println("이미 하단 리미트, 이동 안 함");
                return;
            }
            lift.setMaxSpeed(800);
            lift.setAcceleration(400);
            lift.move(12000 * LIFT_DOWN);
        } else {
            Serial.print("무시: ");
            Serial.println(c);
            Serial.println("u 상승 / d 하강");
        }
    }

    // 상승: 메인과 같은 ToF 목표 거리, 또는 상단 리미트
    if (lift.distanceToGo() * LIFT_UP > 0) {
        if (limitLiftTopPressed()) {
            lift.setCurrentPosition(lift.currentPosition());
        } else if (sensorOk && sensorRangeReady()) {
            uint16_t mm = sensorReadDistanceMm();
            if (!sensorTimeoutOccurred() && mm > 0 && mm < 8000 && mm <= TARGET_DISTANCE_MM) {
                lift.setCurrentPosition(lift.currentPosition());
            }
        }
    }

    // 하강: 메인과 같은 하단 리미트
    if (lift.distanceToGo() * LIFT_DOWN > 0 && limitLiftBottomPressed()) {
        lift.setCurrentPosition(lift.currentPosition());
    }

    lift.run();
}
