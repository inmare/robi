#include "limit_switch.h"

void limitSwitchInit() {
  pinMode(PIN_LIFT_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_LIFT_TOP, INPUT_PULLUP);
  pinMode(PIN_PUSHER_BACK, INPUT_PULLUP);
  pinMode(PIN_PUSHER_FRONT, INPUT_PULLUP);
}

bool limitLiftBottomPressed() {
  return digitalRead(PIN_LIFT_BOTTOM) == LOW;
}

bool limitLiftTopPressed() {
  return digitalRead(PIN_LIFT_TOP) == LOW;
}

bool limitPusherBackPressed() {
  return digitalRead(PIN_PUSHER_BACK) == LOW;
}

bool limitPusherFrontPressed() {
  return digitalRead(PIN_PUSHER_FRONT) == LOW;
}
