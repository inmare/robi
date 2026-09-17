#include "common/lift_motor.h"
#include "common/limit_switch.h"
#include "common/sensor.h"
#include <AccelStepper.h>
#include <Arduino.h>

AccelStepper lift(AccelStepper::DRIVER, 2, 5);

void setup() {
    Serial.begin(115200);
    limitSwitchInit();

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    lift.setMaxSpeed(800); // steps/s
    lift.setAcceleration(400); // steps/s^2

    if (!sensorInit(500)) {
        Serial.println("센서 초기화 실패!");
        while (1) {
        }
    }

    Serial.println("u 상승 / d 하강");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'u' || c == 'U') {
            lift.setMaxSpeed(550);
            lift.setAcceleration(220);
            lift.move(12000 * LIFT_UP);
        } else if (c == 'd' || c == 'D') {
            lift.setMaxSpeed(800);
            lift.setAcceleration(400);
            lift.move(12000 * LIFT_DOWN);
        }
    }

    // 상승: 메인과 같은 ToF 목표 거리, 또는 상단 리미트
    if (lift.distanceToGo() * LIFT_UP > 0) {
        if (limitLiftTopPressed()) {
            lift.setCurrentPosition(lift.currentPosition());
        } else if (sensorRangeReady()) {
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
