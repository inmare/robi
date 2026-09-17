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
LOOKAHEAD_M = 0.45  # 경로 선 앞을 보고 조향. 옆에서 시작해도 다음 점으로 돌지 않게.
ROUTE_LATERAL_M = 0.55  # 경로 옆에서 시작할 때 스캔 맞춤 폭
REJOIN_OFF_M = 0.28  # 이보다 옆이면 A*로 선에 다시 붙음
INFLATE_M = 0.16
STALL_S = 3.0
STALL_MOVE_M = 0.10  # 경로 남은 거리가 이만큼 줄어야 움직임. 제자리 회전·SLAM 흔들림은 무시.

# 바닥 장애물에 라이다가 걸리면 스캔이 끊긴다. 좁은 실내: 짧게 기억하고 더 빼서 돈다.
LIDAR_STALL_S = 1.2
RECOVER_BACK_M = 0.50
RECOVER_SIDE_M = 0.22
RECOVER_TURN_RAD = 0.42
RECOVER_CLEAR_M = 0.28
RECOVER_SECTOR_DEG = 35.0
RECOVER_MAX = 4  # 웨이포인트 하나당. 점을 지나면 횟수 리셋
VIRTUAL_BLOCK_M = 0.22
HIT_AHEAD_M = 0.14
HIT_DEPTH_M = 0.45
RECOVER_REVERSE_S = 6.0
RECOVER_WAIT_S = 2.5
RECOVER_TURN_S = 4.0
RECOVER_HOP_S = 3.0
SURVEY_RAD = 0.70  # 막힌 뒤 좌우로 돌아 스캔을 넓힘
SURVEY_TURN_S = 3.5
SCAN_DISK_MAX_M = 1.20
SCAN_DISK_R = 0.16
SCAN_DISK_FRONT_DEG = 55.0

# 테스트 주행 때 경로를 시간마다 찍는다. 시연은 저장 JSON을 재생.
RECORD_DT = 1.0
RECORD_MIN_M = 0.25
RECORD_MIN_YAW = 0.40

# X4 원시 각은 시계 방향. 오도메트리는 반시계(+yaw).
# SDK Inverted에 의존하지 않고 read()에서 곱한다. 좌우가 뒤집히면 +1.
LIDAR_ANGLE_SIGN = -1
