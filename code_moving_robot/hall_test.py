"""한쪽 바퀴를 느리게 돌리며 A3144E 홀 센서를 확인. 핀은 learn/pin map.md."""

import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice

# 왼쪽 DO = GPIO16 (물리 36), 오른쪽 DO = GPIO5 (물리 29)
LEFT_DO = 16
RIGHT_DO = 5
MAGNETS = 8

LEFT_INVERT = False
RIGHT_INVERT = False
SPEED = 0.15
PWM_HZ = 1000

# 실제 배선이 좌우 반대라, 문서 원래 계획과 GPIO를 바꿈
LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN = 18, 19, 27, 21
RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN = 12, 13, 17, 4


class Wheel:
    def __init__(self, rpwm, lpwm, ren, len_pin, invert=False):
        self.rpwm = PWMOutputDevice(rpwm, frequency=PWM_HZ, initial_value=0)
        self.lpwm = PWMOutputDevice(lpwm, frequency=PWM_HZ, initial_value=0)
        self.ren = DigitalOutputDevice(ren, initial_value=False)
        self.len = DigitalOutputDevice(len_pin, initial_value=False)
        self.invert = invert

    def enable(self):
        self.ren.on()
        self.len.on()

    def disable(self):
        self.rpwm.value = 0
        self.lpwm.value = 0
        self.ren.off()
        self.len.off()

    def drive(self, speed):
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        if self.invert:
            speed = -speed
        if speed > 0:
            self.lpwm.value = 0
            self.rpwm.value = speed
        elif speed < 0:
            self.rpwm.value = 0
            self.lpwm.value = -speed
        else:
            self.rpwm.value = 0
            self.lpwm.value = 0


left_hall = DigitalInputDevice(LEFT_DO, pull_up=True, bounce_time=0.002)
right_hall = DigitalInputDevice(RIGHT_DO, pull_up=True, bounce_time=0.002)

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
    left_wheel = Wheel(LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN, LEFT_INVERT)
    right_wheel = Wheel(RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN, RIGHT_INVERT)

    left_hall.when_activated = on_left
    right_hall.when_activated = on_right

    print("로봇을 들어 두거나 바퀴가 헛돌게 하세요.")
    print("모터 6V 스위치는 이 프로그램이 뜬 뒤에 켜세요.")
    print("한쪽만 느리게 돌리고, 그 바퀴 홀 센서 펄스가 올라가야 정상입니다.")
    side = input("어느 쪽? l 왼쪽 / r 오른쪽 [l]: ").strip().lower()
    if side != "r":
        side = "l"
    side_name = "왼쪽" if side == "l" else "오른쪽"
    input(f"{side_name}만 속도 {SPEED:.2f} 로 돕니다. 준비되면 Enter...")

    moving = left_wheel if side == "l" else right_wheel
    idle = right_wheel if side == "l" else left_wheel
    moving.enable()
    idle.disable()
    moving.drive(SPEED)
    print(
        f"{side_name} 회전 시작. Ctrl+C 종료. "
        f"지금 왼={'자석' if left_hall.is_active else '없음'}, "
        f"오른={'자석' if right_hall.is_active else '없음'}"
    )

    last_l = left_hall.is_active
    last_r = right_hall.is_active
    try:
        while True:
            if left_hall.is_active != last_l:
                last_l = left_hall.is_active
                print(f"왼  {'감지' if last_l else '해제'}")
            if right_hall.is_active != last_r:
                last_r = right_hall.is_active
                print(f"오른 {'감지' if last_r else '해제'}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        moving.disable()
        idle.disable()
        print(f"EN OFF. 왼 {left_count}회, 오른 {right_count}회")


if __name__ == "__main__":
    main()
