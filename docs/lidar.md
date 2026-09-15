# 라이다 (X4 Pro)

코드: `code_moving_robot/robot/lidar.py`  
빌드: `code_moving_robot/lidar_build_script.sh`

## 설정

- 포트 `/dev/ttyUSB0`
- baud `128000`
- `TYPE_TRIANGLE`, 시리얼, 스캔 7Hz, 샘플레이트 5
- `SingleChannel = True`
- `SupportMotorDtrCtrl = False` (정지 때 DTR을 올림)
- `Inverted = True` (X4 기본 각은 시계 방향. 오도메트리 yaw는 반시계라 반드시 뒤집는다)
- `Reversion = False` (라이다 0°가 로봇 앞. 케이블 쪽이 앞이면 True로)

## 모터가 안 꺼질 때

X4 Pro `M_CTR`은 전압이 낮을수록 빠르고 **0V가 최고속**이다.  
DTR을 내리면 정지가 아니라 최고속이 된다. 포트를 닫을 때 리눅스 `HUPCL`이 DTR을 내리면 프로그램이 끝나도 계속 돈다.

지금은 `turnOff()`가 DTR을 올리고, 닫기 전에 `HUPCL`을 끈다.

```bash
cd code_moving_robot
.venv/bin/python tests/lidar_stop.py
```

그래도 최고속이면 어댑터에 DTR이 모터에 안 이어진 것이다. USB를 뽑는다.

| 메서드 | 동작 |
|---|---|
| `start()` | DTR low, 스캔 모터 회전 |
| `stop()` / `close()` | DTR high + HUPCL off |
| `force_motor_off()` | SDK 없이 같은 정지 |

## 시작할 때 빨간 로그

`Fail to get baseplate device information`, `Checksum error`, intensity 16→8→0bit 는 X4 Pro 단방향이라 SDK가 G시리즈처럼 물어보다가 실패하는 줄이다.  
`health status good` 다음에 점 개수가 수백이면 스캔은 된 것이다. 켤 때마다 나온다.

점 수가 0이거나 `스캔 실패`만 반복되면 그때 케이블·전원을 본다.

## SDK 다시 빌드

저장소: https://github.com/YDLIDAR/YDLidar-SDK

```bash
lsusb
ls /dev/ttyUSB*
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
sudo apt update
sudo apt install -y cmake make build-essential swig python3-dev python3-pip git
cd code_moving_robot
chmod +x lidar_build_script.sh
./lidar_build_script.sh
```

`dialout`은 재로그인 후. venv는 `code_moving_robot/.venv` 만 쓴다. `~/ydlidar-venv` 는 쓰지 않는다.

```bash
.venv/bin/python tests/lidar_test.py
.venv/bin/python tests/map_scan_test.py
```

## SSH에서 지도

셸만으로는 PGM 창이 안 뜬다. `map_scan_test.py` / `route_run.py`의 `m` 키가 ASCII를 찍는다. `#` 벽, `.` 빈 공간, `R` 로봇.

PGM은 PC로 복사한다.

```bash
scp pi@주소:code_moving_robot/maps/last_map.pgm .
```
