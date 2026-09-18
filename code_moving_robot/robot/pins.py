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
# w 전진만. 후진(s)은 1.00 유지. 왼쪽으로 휘면 오른쪽을 줄임. 0.02 단위.
LEFT_FWD_SCALE = 1.00
RIGHT_FWD_SCALE = 0.88
PWM_HZ = 1000

MAGNETS = 4
WHEEL_D = 0.066
PULSE_M = math.pi * WHEEL_D / MAGNETS

# 좌우 바퀴 접지 중심 사이. 자로 재서 맞출 것
TRACK_M = 0.25

TELEOP_SPEED = 1.00
NAV_SPEED = 1.00
SPIN_SPEED = 0.85
FRONT_STOP_M = 0.35
FRONT_STOP_DEG = 25.0
ARRIVE_M = 0.15
LOOKAHEAD_M = 0.45  # 경로 선 앞을 보고 조향. 옆에서 시작해도 다음 점으로 돌지 않게.
ROUTE_LATERAL_M = 0.55  # 경로 옆에서 시작할 때 스캔 맞춤 폭
REJOIN_OFF_M = 0.45  # 선에서 이보다 옆일 때만 A*. 다음 점까지 거리와 헷갈리지 말 것.
INFLATE_M = 0.16
STALL_S = 3.0
STALL_MOVE_M = 0.10  # 경로 남은 거리가 이만큼 줄어야 움직임. 제자리 회전·SLAM 흔들림은 무시.

# 바닥 장애물에 라이다가 걸리면 스캔이 끊긴다. 좁은 실내: 짧게 기억하고 더 빼서 돈다.
LIDAR_STALL_S = 1.2
RECOVER_BACK_M = 0.22  # 너무 빼면 뒤에 걸려 회전만 반복한다
RECOVER_SIDE_M = 0.28
RECOVER_TURN_RAD = 0.95  # 좌우 스캔 부채꼴(약 50~130°) 안으로 실제 이동 방향도 맞춘다
RECOVER_CLEAR_M = 0.22
RECOVER_SECTOR_DEG = 40.0
RECOVER_MAX = 4  # 웨이포인트 하나당. 점을 지나면 횟수 리셋
VIRTUAL_BLOCK_M = 0.22
HIT_AHEAD_M = 0.14
HIT_DEPTH_M = 0.45
RECOVER_REVERSE_S = 3.5
RECOVER_WAIT_S = 2.0
RECOVER_TURN_S = 2.5
RECOVER_HOP_S = 2.5
SURVEY_RAD = 0.45
SURVEY_TURN_S = 2.2
YAW_STALL_S = 0.85  # 이 동안 yaw가 거의 안 변하면 회전에 걸린 것
YAW_STALL_RAD = 0.07
WHEEL_SKEW_S = 0.80  # 양쪽 직진 명령인데 한쪽 홀만 뛰면 그 바퀴가 걸린 것
WHEEL_SKEW_PULSES = 3
WHEEL_SKEW_RATIO = 0.20
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
