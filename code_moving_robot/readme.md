# 이동로봇 코드

라즈베리 파이 4, **ROS 없음**. Python.

메인: 슬롯 4개 TUI에서 이름을 붙인 경로를 저장하고, **갔다가 같은 길로 시작점**까지 왕복한다. 경로 중간·옆에서 시작해도 스캔으로 붙고, 장애물은 좌우를 수색한 뒤 A*로 원래 선에 다시 잇는다.

긴 설명은 `docs/` 에만 둔다.

| 문서 | 내용 |
|---|---|
| [docs/moving_stack.md](../docs/moving_stack.md) | 폴더, 라이브러리, 계획, 다른 AI 규칙 |
| [docs/lidar.md](../docs/lidar.md) | SDK 빌드, DTR 모터, 체크섬, SSH 지도 |
| [docs/route.md](../docs/route.md) | **슬롯 TUI · 왕복 · 중간 출발 · 장애 우회** |
| [docs/moving_robot.md](../docs/moving_robot.md) | 기구·전원 |
| [learn/pin map.md](../learn/pin%20map.md) | 핀 |

## 자주 쓰는 명령

```bash
cd code_moving_robot
./lidar_build_script.sh
.venv/bin/python tests/lidar_test.py
.venv/bin/python tests/lidar_stop.py
.venv/bin/python tests/map_scan_test.py
.venv/bin/python tests/path_test.py
.venv/bin/python tests/recover_test.py
.venv/bin/python main.py
```

`main.py` 슬롯·키·왕복은 `docs/route.md` 만 본다. 옛 엔트리 `route/route_run.py` 는 슬롯 없는 단일 경로용이다.
