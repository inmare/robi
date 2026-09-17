#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H

// 이 기계에서 양수 스텝 = 아래, 음수 스텝 = 위.
const int LIFT_DOWN = 1;
const int LIFT_UP = -1;

// 여기만 바꾸면 됨. LIFT_DOWN 또는 LIFT_UP
const int LIFT_DIR = LIFT_DOWN;

// app / test_lift_motor 공통. ToF가 이 거리(mm) 이하면 상승 정지.
const int TARGET_DISTANCE_MM = 70;

#endif
