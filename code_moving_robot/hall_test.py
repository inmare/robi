"""A3144E 홀 센서만 수동 시험. 모터는 안 돌림. 핀은 learn/pin map.md."""

import time
from gpiozero import DigitalInputDevice

# 왼쪽 DO = GPIO16 (물리 36), 오른쪽 DO = GPIO5 (물리 29)
LEFT_DO = 16
RIGHT_DO = 5
MAGNETS = 4

# 자석이 가까우면 DO가 LOW. pull_up 이라 is_active 가 True 면 자석 있음
left = DigitalInputDevice(LEFT_DO, pull_up=True, bounce_time=0.002)
right = DigitalInputDevice(RIGHT_DO, pull_up=True, bounce_time=0.002)

left_count = 0
right_count = 0


def on_left():
    global left_count
    left_count += 1
    print(f"왼  펄스 {left_count:4d}  ({left_count / MAGNETS:.2f} 바퀴)")


def on_right():
    global right_count
    right_count += 1
    print(f"오른 펄스 {right_count:4d}  ({right_count / MAGNETS:.2f} 바퀴)")


def main():
    left.when_activated = on_left
    right.when_activated = on_right

    print("모터는 안 돕니다. 자석을 센서에 붙였다 떼거나 바퀴를 손으로 돌리세요.")
    print("자석이 오면 LOW, 펄스가 올라가야 정상. Ctrl+C 종료")
    print(
        f"지금 왼={'자석' if left.is_active else '없음'}, "
        f"오른={'자석' if right.is_active else '없음'}"
    )

    last_l = left.is_active
    last_r = right.is_active
    try:
        while True:
            if left.is_active != last_l:
                last_l = left.is_active
                print(f"왼  {'감지' if last_l else '해제'}")
            if right.is_active != last_r:
                last_r = right.is_active
                print(f"오른 {'감지' if last_r else '해제'}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    print(f"종료. 왼 {left_count}회, 오른 {right_count}회")


if __name__ == "__main__":
    main()
