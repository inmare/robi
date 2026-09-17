# 적재함 중앙 컨트롤

엔트리: `code_center_control/main.py`  
상자 펌웨어: `code_loading_box` 의 `app` + `bridge_nodemcu`

상자는 한 번에 한 동작만 한다. 순서·반복은 여기서 명령 id를 붙여 보낸다.

```bash
cd code_center_control
uv sync
uv run python main.py --mock
```

| 실행 | 용도 |
|---|---|
| `uv run python main.py --mock` | 보드 없이 명령·레시피 시험 |
| `uv run python main.py --serial COM5 --no-tcp` | 우노 USB |
| `uv run python main.py` | ESP 브리지가 붙을 TCP `:9000` |
| `uv run python main.py --mock --web-only` | 가짜 상자 + 브라우저 `http://127.0.0.1:8080/` |
| `uv run python main.py --mqtt 127.0.0.1` | 같은 명령을 Mosquitto에도 올림. 모니터에 MQTT 줄이 추가로 찍힘 |

입력 예: `lift.up`, `상승`, `status`, `discharge`, `return_book --step` 은 실행 인자 `--step`.
