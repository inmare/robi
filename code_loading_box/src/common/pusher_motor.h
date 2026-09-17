#ifndef PUSHER_MOTOR_H
#define PUSHER_MOTOR_H

// 이 기계에서 음수 스텝 = 책 밀기(앞), 양수 스텝 = 후진(뒤).
const int PUSHER_FORWARD = -1;
const int PUSHER_BACK = 1;

// 푸셔 4핀 = 쉴드 X. test_pusher_motor 와 같음.
const int PUSHER_STEP_PIN = 2;  // D2
const int PUSHER_DIR_PIN = 5;   // D5

#endif
