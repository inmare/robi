# 빌드 명령어
- `pio run -e app -t upload`: 실제 앱
- `pio run -e test_sensor -t upload`: VL53L0X 거리 테스트
- `pio run -e test_lift_motor -t upload`: 리프트 모터 테스트
- `pio run -e test_pusher_motor -t upload`: 푸셔 모터 짧은 이동 테스트
- `pio run -e test_limit_switch -t upload`: 리미트 스위치 4개 테스트

이거 하고 `pio device monitor -b 115200` 하면 됨