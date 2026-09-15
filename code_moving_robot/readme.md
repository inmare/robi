# 이동로봇 코드

라즈베리 파이 4, **ROS 없음**. Python.

긴 설명은 `docs/` 에만 둔다.

| 문서 | 내용 |
|---|---|
| [docs/moving_stack.md](../docs/moving_stack.md) | 폴더, 라이브러리, 계획, 다른 AI 규칙 |
| [docs/lidar.md](../docs/lidar.md) | SDK 빌드, DTR 모터, 체크섬, SSH 지도 |
| [docs/route.md](../docs/route.md) | **수동 지도 + 시작/경유/도착 사용법** |
| [docs/moving_robot.md](../docs/moving_robot.md) | 기구·전원 |
| [learn/pin map.md](../learn/pin%20map.md) | 핀 |

## 자주 쓰는 명령

```bash
cd code_moving_robot
./lidar_build_script.sh
.venv/bin/python tests/lidar_test.py
.venv/bin/python tests/lidar_stop.py
.venv/bin/python tests/map_scan_test.py
.venv/bin/python route/route_run.py
```

`route_run.py` 키와 순서는 `docs/route.md` 만 본다.
