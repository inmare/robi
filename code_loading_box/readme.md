# 빌드 명령어
- `pio run -e app -t upload`: 실제 앱 (중앙 명령 수신)
- `pio run -e bridge_nodemcu -t upload`: NodeMCU UART↔TCP 브리지 (USB를 NodeMCU에)
- `pio run -e test_sensor -t upload`: VL53L0X 거리 테스트
- `pio run -e test_lift_motor -t upload`: 리프트 모터 테스트
- `pio run -e test_pusher_motor -t upload`: 푸셔 모터 짧은 이동 테스트
- `pio run -e test_limit_switch -t upload`: 리미트 스위치 4개 테스트
- `pio run -e test_esp8266_nodemcu -t upload`: NodeMCU 쪽 UART 테스트 (USB를 NodeMCU에)
- `pio run -e test_esp8266 -t upload`: 우노 쪽 UART 테스트 (USB를 우노에)

이거 하고 `pio device monitor -b 115200` 하면 됨

ESP8266 통신 테스트는 NodeMCU를 먼저 올리고, USB를 우노로 옮긴 뒤 우노를 올린다. 우노 모니터에서 `p`를 치면 `PONG`이 와야 한다.

중앙 컨트롤 브리지는 `src/bridge/wifi_secrets.example.h`를 `wifi_secrets.h`로 복사해 SSID·비번·PC IP를 넣은 다음 `bridge_nodemcu`를 올린다.
