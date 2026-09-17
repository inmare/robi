# 이동로봇 코드 스택

하드웨어 본체는 `docs/moving_robot.md`. 핀은 `learn/pin map.md`.

이 프로젝트는 **ROS를 쓰지 않는다.** 스캔·지도·경로·회피·시연은 Python + (예정) MQTT.

관련 문서:

- `docs/lidar.md` — X4 Pro SDK, DTR 모터, 체크섬 로그, SSH로 지도 보기
- `docs/route.md` — 슬롯 TUI, 왕복 재생, 중간 출발, 장애 수색 후 재연결
- `docs/moving_robot.md` — 기구·전원
- `code_moving_robot/readme.md` — 짧은 색인만

## 폴더

```
code_moving_robot/
  robot/
    pins.py        GPIO·바퀴 상수. TRACK_M 은 자로 잰 좌우 접지 중심 거리
    drive.py       좌우 PWM
    odometry.py    홀 + 명령 부호로 x,y,yaw
    lidar.py       X4 Pro 시작·정지. 각 부호는 LIDAR_ANGLE_SIGN
    grid.py        occupancy grid
    localize.py    스캔-맵 상관 맞춤
    slam.py        스캔-투-맵 SLAM. 홀은 예측, 라이다가 포즈
    planner.py     A*
    follow.py      경로 선 look-ahead 추종
    path.py        왕복·선 투영
    recover.py     후진·수색·A* 재연결
    slots.py       슬롯 4개 이름·시각
    tui.py         메뉴·키
    session.py     주행 루프
  tests/
    motor_test.py, hall_test.py, hall_motor_test.py
    lidar_test.py, lidar_stop.py, map_scan_test.py
    path_test.py, recover_test.py, slots_test.py
  route/
    route_run.py   옛 단일 경로. 새 시연은 main.py
  main.py          슬롯 TUI + 왕복 주행
  maps/            pgm·json·slots/. git에 안 올림
  lidar_build_script.sh
  .venv/
```

실행은 항상 `code_moving_robot/.venv`.

```bash
cd code_moving_robot
uv pip install --python .venv/bin/python gpiozero
.venv/bin/python main.py
```

`gpiozero`가 venv에 없으면 홀·모터가 안 열린다. 라이다만 쓸 때는 SDK만 있어도 된다.

## 목표

1. 라이다 스캔. 안 쓸 때는 모터(DTR) 정지
2. 직접 몰면서 occupancy grid. 포즈는 홀 예측 + 스캔 SLAM
3. 시작·경유·도착을 그 자리에서 찍고, 왕복으로 따라감
4. 주행 중 장애물은 **지금 스캔** 전방 거리로 정지. 저장 지도만 믿지 않음
5. 시연은 MQTT + 웹 (아직 없음)
6. 중앙 센터 MQTT 상태기계는 이후

기울여 찍은 2D 지도는 윤곽 확인용. 수평 고정 후 다시 그린다.

구현 순서 (`docs/readme.md`와 같음): 모터 → 차동 → 홀 → 라이다 → 장애물 정지 → 적재함 접근.

## 쓰지 않는 것

ROS 1/2, Nav2, slam_toolbox, RViz, `ydlidar_ros2_driver`.  
시연용으로 파이 HDMI에 matplotlib를 띄우지 않는다.

## 스택 표

| 역할 | 선택 |
|---|---|
| 보드 | 라즈베리 파이 4, Raspberry Pi OS |
| 언어 | Python 3 |
| 라이다 | YDLIDAR X4 Pro `/dev/ttyUSB0` |
| SDK | YDLidar-SDK → `import ydlidar` |
| 모터·홀 | `gpiozero` |
| 격자·A* | 표준 라이브러리 (`robot/grid.py`, `planner.py`) |
| 시연 | MQTT `paho-mqtt` (아직 없음), 브로커 Mosquitto |

## 라이브러리

| 패키지 | 용도 |
|---|---|
| `ydlidar` | SDK에서 빌드. pip 이름 아님 |
| `gpiozero` | 모터·홀. `main.py`에 필요 |
| `numpy` / `matplotlib` | 아직 안 씀. 지도는 PGM + ASCII |
| `paho-mqtt` | 아직 없음 |

## MQTT (예정)

- `robot/pose` 자주, 작게
- `robot/map` 가끔
- `robot/scan` 줄여서
- `robot/status`

## 다른 AI

- 사용자가 파일 작성을 명시하기 전에는 코드 대신 역할만 설명한다
- 로직은 `robot/`, 경로 작업 엔트리는 `route/`, 시험은 `tests/`
- 라이다는 `finally`/`with`로 `close()`
- 핀은 `learn/pin map.md`와 `robot/pins.py`. 추측하지 않음
- 기울인 라이다 지도를 최종 경로 맵으로 쓰지 않음
