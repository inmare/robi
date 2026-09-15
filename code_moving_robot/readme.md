# 이동로봇 (`code_moving_robot`)

라즈베리 파이 4에서 차동구동 이동로봇을 제어한다.  
적재함·중앙 센터와 맞춰 볼 문서: `docs/moving_robot.md`, `docs/readme.md`, `learn/pin map.md`.

이 폴더는 **ROS를 쓰지 않는다.** 스캔·지도·경로·회피·시연 화면은 Python + MQTT로 간다.

## 목표 (현재 계획)

1. 라이다로 스캔한다.
2. occupancy grid 지도를 만든다. (홀 오도메트리 + 스캔)
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
| 격자·경로 계산 | `numpy` |
| 파이 로컬 디버그 그림 | `matplotlib` (시연 화면 아님) |
| 로봇 → 중앙 | MQTT, 클라이언트 `paho-mqtt` |
| 브로커 | 노트북/중앙 PC의 Mosquitto |
| 사람에게 보여 주기 | 브라우저. 브로커 WebSocket + 캔버스에 격자·로봇 위치 |
| 패키지 설치 | `uv` 가상환경. 라이다는 `~/ydlidar-venv` |

핀·전원은 `learn/pin map.md`만 본다. 라이다는 GPIO가 아니라 파이 USB다.

모터는 N7960 + JGA25-370 좌우 독립, 뒤는 볼캐스터. 홀(A3144E)로 바퀴 펄스를 센다. 직진 보정은 홀로 하고, 절대 위치는 라이다(+ 이후 주차 센서)가 담당한다.

## 라이브러리 (Python)

라이다 venv (`~/ydlidar-venv`)와 GPIO 코드가 나중에는 같은 환경에 있어야 한다. 지금은 시험 파일이 분리돼 있다.

| 패키지 | 용도 | 지금 코드에서 |
|---|---|---|
| `ydlidar` | SDK Python 바인딩. pip 패키지가 아니라 SDK 소스에서 빌드 | `lidar_test.py` |
| `gpiozero` | 모터 PWM/EN, 홀 입력 | `motor_test.py`, `hall_test.py`, `hall_motor_test.py` |
| `numpy` | 스캔 → x,y, occupancy grid, A* | 아직 없음. 지도 단계에서 추가 |
| `matplotlib` | 한 바퀴 스캔·격자 확인 | 아직 없음. 디버그용 |
| `paho-mqtt` | 포즈·지도·상태 publish | 아직 없음. 시연·중앙 통신 단계 |

apt로 넣는 것(SDK 빌드용)은 아래 “YDLidar-SDK 다시 빌드”를 따른다.  
`gpiozero`는 파이 OS에 이미 있는 경우가 많다. venv에서 쓸 때는 `uv pip install gpiozero` 하고, 필요하면 시스템 패키지 `python3-gpiozero`, `python3-lgpio` 또는 `python3-rpi.gpio`를 맞춘다.

## MQTT 토픽 (예정)

실시간과 파일 재생이 같은 JSON을 쓰게 한다. 스캔 전체를 7Hz JSON으로 보내지 않는다.

- `robot/pose` — x, y, yaw, 홀 카운트. 자주, 작게
- `robot/map` — occupancy grid. 2~5초에 한 번이거나 바뀐 칸만. PNG/바이트 배열 가능
- `robot/scan` — 선택. 1도 빈 또는 초당 2번 이하
- `robot/status` — 정지·직진·장애물 등. 이후 주차/출발 허가와 같은 브로커

X4 Pro 점구름을 매 스캔 전부 실으면 파이 Wi-Fi가 버거울 수 있다. 시연은 위치 + 줄인 격자면 충분하다.

## 이 폴더의 파일

| 파일 | 역할 |
|---|---|
| `motor_test.py` | 모터만. 전진·후진·정지 |
| `hall_test.py` | 홀만. 모터 안 돌림 |
| `hall_motor_test.py` | 홀 카운트로 1m 직진, 좌우 펄스 보정 |
| `lidar_test.py` | X4 Pro 스캔. 전방 ±5° 거리만 출력 |
| `install_script.sh` | 라이다 포트 확인 + SDK/uv 설치 메모. 경로 플레이스홀더 있음 |
| `readme.md` | 이 문서 |

아직 없는 것: occupancy grid, A*, MQTT publish, 시연 저장/재생, 메인 상태기계.

`lidar_test.py` 설정 (바꾸기 전에 이 값을 기본으로):

- 포트 `/dev/ttyUSB0`
- baud `128000`
- `LidarPropLidarType` = `TYPE_TRIANGLE`
- `LidarPropDeviceType` = `YDLIDAR_TYPE_SERIAL`
- 스캔 주파수 `7.0`
- 샘플레이트 `5`
- `LidarPropSingleChannel` = `True`

## YDLidar-SDK 다시 빌드

공식 저장소: https://github.com/YDLIDAR/YDLidar-SDK  
파이에서 USB로 X4 Pro가 잡혀 있어야 한다.

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

이미 클론·설치한 적이 있어도, SDK를 다시 받을 때나 바인딩이 `import ydlidar` 실패일 때 이 패키지는 그대로 다시 두면 된다.

### 3. C++ SDK

기존 빌드가 깨졌으면 `build` 폴더를 지우고 다시 한다.

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
rm -rf build
mkdir -p build && cd build
cmake ..
make
sudo make install
cd ..
```

이미 클론된 디렉터리가 있으면 `clone` 대신 그 폴더로 가서 `git pull` 후 `build`부터 반복한다.  
`sudo make install`이 `/usr/local`에 라이브러리를 넣는다.

### 4. Python 바인딩 (`uv`)

`uv`가 없으면:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

SDK **루트**(안에 `setup.py` 또는 `pyproject.toml`이 있는 `YDLidar-SDK` 폴더)에서:

```bash
source $HOME/.local/bin/env
cd /경로/YDLidar-SDK
uv venv ~/ydlidar-venv
source ~/ydlidar-venv/bin/activate
uv pip install .
```

`cd  경로/YDLidar-SDK` 같이 공백 있는 플레이스홀더는 쓰지 않는다. 실제 클론 경로로 바꾼다.

지도·MQTT를 같은 프로세스에서 쓰려면 이 venv에 이어서:

```bash
uv pip install numpy matplotlib paho-mqtt gpiozero
```

### 5. 실행

```bash
source $HOME/.local/bin/env
~/ydlidar-venv/bin/python /경로/code_moving_robot/lidar_test.py
```

`초기화 실패`면 포트·baud·`dialout`·다른 프로세스가 `/dev/ttyUSB0`을 잡고 있는지 본다.  
`import ydlidar` 실패면 3~4를 다시 하고, 실행이 시스템 `python3`가 아니라 `~/ydlidar-venv/bin/python`인지 확인한다.

## 다른 AI를 위한 작업 규칙

- 코드를 직접 고치지 말고, 작성 순서와 역할을 설명하는 것이 이 저장소의 기본이다. 사용자가 파일 작성을 명시하면 그때만 쓴다.
- 새 기능은 기존 시험 파일에 이어 붙이지 말고, 라이다 / 모터·홀 / MQTT를 분리한다.
- 핀 번호를 추측하지 말고 `learn/pin map.md`를 연다. `hall_motor_test.py`는 좌우 GPIO를 실제 배선에 맞게 바꾼 상태다.
- 라이다 장착이 기울어진 채로 만든 격자 지도를 경로 추종용 최종 맵으로 쓰지 않는다.
- 시연 데이터는 MQTT 페이로드와 저장 파일 스키마를 같게 둔다.
