#ifndef PUSHER_MOTOR_H
#define PUSHER_MOTOR_H

// 부호는 배선에 따라 전진/후진이 반대일 수 있음. 반대로 가면 PUSHER_DIR만 바꿈.
const int PUSHER_FORWARD = 1;
const int PUSHER_BACK = -1;

// 여기만 바꾸면 됨. PUSHER_FORWARD 또는 PUSHER_BACK
const int PUSHER_DIR = PUSHER_FORWARD;

#endif
