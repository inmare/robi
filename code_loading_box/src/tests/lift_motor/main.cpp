#include <Arduino.h>
#include <AccelStepper.h>

AccelStepper lift(AccelStepper::DRIVER, 2, 5);


void setup() {
    Serial.begin(115200);

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    // 최대 속도 상한
    lift.setMaxSpeed(800); // steps/s
    // 가감속 기울기
    lift.setAcceleration(400); // steps/s^2
    // 목표 절대 위치
    lift.moveTo(-1000);

}

void loop() {
    lift.run();
    // lift.stop(); // 정지
    // lift.setCurrentPosition(lift.currentPosition()); // 현재 위치에 멈추기
    // lift.move(); // 상대 이동
    // lift.distanceToGo(); // 목표 위치까지의 거리
}