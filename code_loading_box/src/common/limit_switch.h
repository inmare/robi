#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

// CNC 쉴드 END STOPS / Abort. 내부 풀업, 눌리면 LOW.
const int PIN_LIFT_BOTTOM = 9;    // END STOPS X-, 리프트 하단
const int PIN_LIFT_TOP = 10;      // END STOPS Y-, 리프트 상단
const int PIN_PUSHER_BACK = 11;   // END STOPS Z-, 푸셔 후진
const int PIN_PUSHER_FRONT = A0;  // Abort, 푸셔 전진

void limitSwitchInit();

bool limitLiftBottomPressed();
bool limitLiftTopPressed();
bool limitPusherBackPressed();
bool limitPusherFrontPressed();

#endif
