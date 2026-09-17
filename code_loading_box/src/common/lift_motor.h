#ifndef LIFT_MOTOR_H
#define LIFT_MOTOR_H

// 이 기계에서 양수 스텝 = 아래, 음수 스텝 = 위.
const int LIFT_DOWN = 1;
const int LIFT_UP = -1;

// 여기만 바꾸면 됨. LIFT_DOWN 또는 LIFT_UP
const int LIFT_DIR = LIFT_DOWN;

// app / test_lift_motor 공통. ToF 목표 40mm.
// 값이 ±5mm 안에서 HOLD_COUNT번 연속이면 상단 리미트 없이 도착으로 본다.
const int TARGET_DISTANCE_MM = 40;
const int TARGET_BAND_MM = 5;
const int TARGET_HOLD_COUNT = 6;

inline bool tofInBand(int mm) {
  return mm >= TARGET_DISTANCE_MM - TARGET_BAND_MM &&
         mm <= TARGET_DISTANCE_MM + TARGET_BAND_MM;
}

inline bool tofPastTarget(int mm) {
  return mm > 0 && mm < TARGET_DISTANCE_MM - TARGET_BAND_MM;
}

inline bool tofReached(int mm) {
  return tofInBand(mm) || tofPastTarget(mm);
}

// X·Z 바꿔 꽂은 상태. 리프트 4핀 = 쉴드 Z. test_lift_motor 와 같음.
const int LIFT_STEP_PIN = 4;  // D4
const int LIFT_DIR_PIN = 7;   // D7

#endif
