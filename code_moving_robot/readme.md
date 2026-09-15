# 이동로봇 (`code_moving_robot`)

라즈베리 파이 4에서 차동구동 이동로봇을 제어한다.  
적재함·중앙 센터와 맞춰 볼 문서: `docs/moving_robot.md`, `docs/readme.md`, `learn/pin map.md`.

이 폴더는 **ROS를 쓰지 않는다.** 스캔·지도·경로·회피·시연 화면은 Python + MQTT로 간다.

## 폴더 구조

```
code_moving_robot/
  robot/                 실제 모듈. 시험 파일에 로직을 이어 붙이지 않는다
    lidar.py             X4 Pro 시작·정지·한 바퀴 읽기
    grid.py              occupancy grid, PGM 저장
  tests/                 수동 시험만 모은다
    motor_test.py
    hall_test.py
    hall_motor_test.py
    lidar_test.py        전방 거리. 종료 시 모터 OFF
    lidar_stop.py        스캔 없이 DTR을 올려 모터 정지 시도
    map_scan_test.py     제자리 스캔 → 터미널 ASCII + maps/last_scan.pgm
  maps/                  저장한 격자. git에 올리지 않음
  lidar_build_script.sh  이 폴더에 SDK 클론 + .venv 에 바인딩
  .venv/                 uv 환경. 홈의 ydlidar-venv 를 쓰지 않음
  YDLidar-SDK/           빌드 스크립트가 클론. git에 올리지 않음
```

아직 없는 모듈: 모터·홀 패키지화, A*, MQTT, 메인 상태기계.  
모터·홀은 당분간 `tests/` 안 시험 스크립트만 쓴다.

실행은 항상 프로젝트 루트의 venv로 한다.

```bash
cd code_moving_robot
.venv/bin/python tests/lidar_test.py
.venv/bin/python tests/map_scan_test.py
.venv/bin/python tests/lidar_stop.py
```

## 라이다 모터

X4 Pro 모터는 USB 전원이 아니라 시리얼 **DTR → M_CTR**으로 속도를 바꾼다.  
포트를 닫을 때 DTR이 0V로 떨어지면 데이터시트상 **최고속**이 된다.

X4 Pro 모터핀 `M_CTR`은 전압이 **낮을수록 빠르고, 0V가 최고속**이다.  
그래서 DTR을 내리면 정지가 아니라 최고속이 된다. 예전 `stop`/`lidar_stop.py`가 그렇게 동작했다.

지금은 `LidarPropSupportMotorDtrCtrl = False` 로 `turnOff()`가 DTR을 **올린다**.  
포트를 닫을 때 리눅스 `HUPCL`이 DTR을 다시 내리면 또 최고속이 되므로, 닫기 전에 `HUPCL`을 끈다.

| 메서드 | 동작 |
|---|---|
| `start()` | DTR low, 모터 회전 |
| `stop()` | `turnOff()`가 DTR high |
| `close()` | 정지 후 포트 해제. HUPCL 끄고 DTR을 올린 채 닫음 |
| `force_motor_off()` | SDK 없이 DTR high + HUPCL off |
| `with Lidar() as lidar:` | 들어가면 start, 나오면 close |

이미 돌아가고 있으면:

```bash
.venv/bin/python tests/lidar_stop.py
```

그래도 안 멈추면 USB를 뺐다 꽂는다.

## SSH에서 지도 보기

SSH 셸만으로는 PGM/PNG 창이 안 뜬다. X11 포워딩도 윈도우에서 거의 안 쓴다.

`tests/map_scan_test.py`는 끝나면 터미널에 ASCII 지도를 찍는다. `#` 벽, `.` 빈 공간, `R` 로봇. SSH에서 보는 그림은 이것이다.

`maps/last_scan.pgm`은 나중에 PC로 가져가 이미지로 연다.

```bash
scp pi@라즈베리주소:code_moving_robot/maps/last_scan.pgm .
```

Cursor/VS Code Remote SSH로 파이 폴더를 열고 있으면 그 PGM을 에디터에서 열 수도 있다. 순수 셸만 있으면 ASCII가 맞다.

## 목표 (현재 계획)

1. 라이다로 스캔한다. 안 쓸 때는 모터를 끈다.
2. occupancy grid 지도를 만든다. 지금은 제자리 스캔(`tests/map_scan_test.py`). 다음은 홀 오도메트리와 합치기.
3. 격자 위에서 A*로 원하는 칸까지 경로를 찾는다.
4. 주행 중 장애물은 **지금 스캔**의 전방 거리·좌우 여유로 감속·정지·회피한다. 저장된 지도만 믿지 않는다.
5. 주행 테스트를 사람에게 보여 준다.
   - 우선: MQTT로 실시간 위치 + 격자(또는 줄인 스캔)
   - 실시간 부하가 크면: 같은 형식으로 파일을 저장해 두고 웹에서 재생
6. 중앙 센터와는 MQTT로 상태·명령을 주고받는다. (`주차 완료 → 적재 요청 → 적재 완료 → 출발 허가` 는 이후)

라이다 고정 브래킷이 나오기 전, 센서가 앞쪽이 솟게 비스듬해도 **윤곽 확인용 테스트 지도**는 된다.  
2D 라이다는 스캔면이 바닥과 평행하다고 가정하므로, 그 지도는 최종 주행용으로 쓰지 말고 수평 고정 후 다시 그린다. 흔들림이 기울기보다 더 나쁘다.

구현 순서는 `docs/readme.md`와 같다. 한 파일에 모터+라이다+MQTT를 한꺼번에 넣지 않는다.

1. 좌우 모터 전진·후진·정지
2. 차동구동 직진·회전
3. 홀 카운트
4. 라이다 읽기
5. 장애물 발견 시 정지
6. (이후) 적재함 탐색·저속 접근·리미트 정렬·주차 메시지

## 쓰지 않는 것

- ROS 1 / ROS 2, Nav2, slam_toolbox, RViz, `ydlidar_ros2_driver`
- 파이 HDMI에 matplotlib를 띄워 시연하는 방식 (디버그 전용. 시연은 MQTT + 웹)

ROS는 나중에 기성 Nav2를 붙일 때만 검토한다. 그때는 Ubuntu 22.04 + Humble이 필요하고, 지금의 `gpiozero` 모터/홀 코드를 ROS 노드로 다시 감싸야 한다.

## 스택

| 역할 | 선택 |
|---|---|
| 보드 | 라즈베리 파이 4, Raspberry Pi OS |
| 언어 | Python 3 |
| 라이다 | YDLIDAR X4 Pro, USB 시리얼 `/dev/ttyUSB0` |
| 라이다 SDK | [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) C++ 설치 후 Python 바인딩 `import ydlidar` |
| 모터·홀 GPIO | `gpiozero` (`PWMOutputDevice`, `DigitalOutputDevice`, `DigitalInputDevice`) |
| 격자 | `robot/grid.py` 리스트 격자. numpy는 이후 A*·시연에서 |
| 파이 로컬 디버그 그림 | `maps/*.pgm` 을 이미지 뷰어로. matplotlib는 선택 |
| 로봇 → 중앙 | MQTT, 클라이언트 `paho-mqtt` (아직 없음) |
| 브로커 | 노트북/중앙 PC의 Mosquitto |
| 사람에게 보여 주기 | 브라우저. 브로커 WebSocket + 캔버스에 격자·로봇 위치 |
| 패키지 설치 | `uv` 가상환경 `code_moving_robot/.venv` |

핀·전원은 `learn/pin map.md`만 본다. 라이다는 GPIO가 아니라 파이 USB다.

모터는 N7960 + JGA25-370 좌우 독립, 뒤는 볼캐스터. 홀(A3144E)로 바퀴 펄스를 센다. 직진 보정은 홀로 하고, 절대 위치는 라이다(+ 이후 주차 센서)가 담당한다.

## 라이브러리 (Python)

전부 `code_moving_robot/.venv` 에 넣는다. `lidar_build_script.sh`가 SDK 바인딩까지 설치한다.

| 패키지 | 용도 | 지금 코드에서 |
|---|---|---|
| `ydlidar` | SDK Python 바인딩. pip 패키지가 아니라 SDK 소스에서 빌드 | `robot/lidar.py`, `tests/lidar_test.py`, `tests/map_scan_test.py` |
| `gpiozero` | 모터 PWM/EN, 홀 입력 | `tests/motor_test.py`, `tests/hall_test.py`, `tests/hall_motor_test.py` |
| `numpy` | 이후 A*·배열 연산 | 아직 없음. grid는 표준 리스트 |
| `matplotlib` | 한 바퀴 스캔·격자 확인 | 아직 없음. 지금은 PGM |
| `paho-mqtt` | 포즈·지도·상태 publish | 아직 없음 |

`gpiozero`는 파이 OS에 이미 있는 경우가 많다. venv에서 쓸 때는 `uv pip install gpiozero` 하고, 필요하면 시스템 패키지 `python3-gpiozero`, `python3-lgpio` 또는 `python3-rpi.gpio`를 맞춘다.

## MQTT 토픽 (예정)

실시간과 파일 재생이 같은 JSON을 쓰게 한다. 스캔 전체를 7Hz JSON으로 보내지 않는다.

- `robot/pose` — x, y, yaw, 홀 카운트. 자주, 작게
- `robot/map` — occupancy grid. 2~5초에 한 번이거나 바뀐 칸만. PNG/바이트 배열 가능
- `robot/scan` — 선택. 1도 빈 또는 초당 2번 이하
- `robot/status` — 정지·직진·장애물 등. 이후 주차/출발 허가와 같은 브로커

X4 Pro 점구름을 매 스캔 전부 실으면 파이 Wi-Fi가 버거울 수 있다. 시연은 위치 + 줄인 격자면 충분하다.

## 라이다 설정 (`robot/lidar.py`)

바꾸기 전에 이 값을 기본으로 둔다.

- 포트 `/dev/ttyUSB0`
- baud `128000`
- `LidarPropLidarType` = `TYPE_TRIANGLE`
- `LidarPropDeviceType` = `YDLIDAR_TYPE_SERIAL`
- 스캔 주파수 `7.0`
- 샘플레이트 `5`
- `LidarPropSingleChannel` = `True`

## YDLidar-SDK 다시 빌드

공식 저장소: https://github.com/YDLIDAR/YDLidar-SDK  
파이에서 이 폴더의 `lidar_build_script.sh`를 실행한다. 스크립트가 있는 폴더로 이동한 뒤 `YDLidar-SDK`를 클론하고 `.venv`에 바인딩을 넣는다.

### 1. 장치

```bash
lsusb
ls /dev/ttyUSB*
```

`/dev/ttyUSB0`이 있어야 한다. 없으면 케이블·전원·칩셋 드라이버부터 본다.

```bash
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

`dialout` 추가는 로그아웃/재로그인 후에 적용된다.

### 2. apt 빌드 의존성

```bash
sudo apt update
sudo apt install -y cmake make build-essential swig python3-dev python3-pip git
```

| 패키지 | 이유 |
|---|---|
| `cmake` `make` `build-essential` | C++ SDK 컴파일 |
| `swig` `python3-dev` | Python 바인딩 |
| `python3-pip` `git` | pip 설치, 클론 |

### 3. 스크립트

```bash
cd code_moving_robot
chmod +x lidar_build_script.sh
./lidar_build_script.sh
```

홈의 `~/ydlidar-venv` 는 쓰지 않는다. 설치는 `lidar_build_script.sh`만 쓴다.

지도·MQTT를 같은 프로세스에서 쓰려면 `.venv`에 이어서:

```bash
uv pip install --python .venv/bin/python numpy matplotlib paho-mqtt gpiozero
```

### 4. 실행

```bash
.venv/bin/python tests/lidar_test.py
.venv/bin/python tests/map_scan_test.py
```

`초기화 실패`면 포트·baud·`dialout`·다른 프로세스가 `/dev/ttyUSB0`을 잡고 있는지 본다.  
`import ydlidar` 실패면 빌드 스크립트를 다시 하고, 실행이 시스템 `python3`가 아니라 `.venv/bin/python`인지 확인한다.

`maps/last_scan.pgm` 은 PC로 복사해 이미지 뷰어로 연다. SSH만 있으면 `map_scan_test.py`가 찍는 ASCII(`#` 벽, `.` 빈 공간)를 본다. 기울여 찍은 결과는 윤곽 확인용이다.

## 다른 AI를 위한 작업 규칙

- 코드를 직접 고치지 말고, 작성 순서와 역할을 설명하는 것이 이 저장소의 기본이다. 사용자가 파일 작성을 명시하면 그때만 쓴다.
- 새 기능은 `tests/`에 이어 붙이지 말고 `robot/`에 모듈을 두고, 시험만 `tests/`에 둔다. 라이다 / 모터·홀 / MQTT를 분리한다.
- 라이다를 켠 코드는 반드시 `finally` 또는 `with`로 `close()` 한다.
- 핀 번호를 추측하지 말고 `learn/pin map.md`를 연다. `tests/hall_motor_test.py`는 좌우 GPIO를 실제 배선에 맞게 둔 상태다.
- 라이다 장착이 기울어진 채로 만든 격자 지도를 경로 추종용 최종 맵으로 쓰지 않는다.
- 시연 데이터는 MQTT 페이로드와 저장 파일 스키마를 같게 둔다.
