"""핀·기구 상수. 번호는 learn/pin map.md, 홀·PWM은 tests/hall_motor_test.py 와 같게."""

import math

# 홀 DO
LEFT_DO = 5
RIGHT_DO = 16

# 왼쪽 GPIO18/19/27/21, 오른쪽 GPIO12/13/17/4
LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN = 18, 19, 27, 21
RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN = 12, 13, 17, 4

LEFT_INVERT = False
RIGHT_INVERT = False
PWM_HZ = 1000

MAGNETS = 4
WHEEL_D = 0.066
PULSE_M = math.pi * WHEEL_D / MAGNETS

# 좌우 바퀴 접지 중심 사이. 자로 재서 맞출 것
TRACK_M = 0.25

TELEOP_SPEED = 0.62
NAV_SPEED = 0.58
SPIN_SPEED = 0.5
FRONT_STOP_M = 0.35
FRONT_STOP_DEG = 25.0
ARRIVE_M = 0.15
INFLATE_M = 0.16
