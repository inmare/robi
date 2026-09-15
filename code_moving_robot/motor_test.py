"""N7960 바퀴 수동 시험. 핀 번호는 learn/pin map.md 와 같음."""

import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# 바퀴가 반대로 돌면 True 로 바꿈
LEFT_INVERT = False
RIGHT_INVERT = False

SPEED = 0.3  # 0.0 ~ 1.0. 처음엔 낮게
PWM_HZ = 1000

# 핀은 learn/pin map.md. 왼쪽 GPIO18/19/27/21, 오른쪽 GPIO12/13/17/4
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

    def stop(self):
        self.rpwm.value = 0
        self.lpwm.value = 0

    def drive(self, speed):
        """speed > 0 전진, < 0 후진, 0 정지. 절댓값은 0~1."""
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
            self.stop()


def apply(left, right, mode, speed):
    if mode == "f":
        left.drive(speed)
        right.drive(speed)
    elif mode == "b":
        left.drive(-speed)
        right.drive(-speed)
    elif mode == "l":
        left.drive(speed)
        right.stop()
    elif mode == "r":
        left.stop()
        right.drive(speed)
    else:
        left.stop()
        right.stop()
    print(
        f"모드 {mode}, 속도 {speed:.2f}, "
        f"L pwm {left.rpwm.value:.2f}/{left.lpwm.value:.2f}, "
        f"R pwm {right.rpwm.value:.2f}/{right.lpwm.value:.2f}"
    )


def main():
    left = Wheel(LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN, LEFT_INVERT)
    right = Wheel(RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN, RIGHT_INVERT)
    speed = SPEED
    mode = "s"

    print("로봇을 들어 두거나 바퀴가 헛돌게 하세요.")
    print("모터 6V 스위치는 이 프로그램이 뜬 뒤에 켜세요.")
    print("명령: f 전진 / b 후진 / l 왼쪽만 / r 오른쪽만 / s 정지 / + - 속도 / q 종료")
    input("준비되면 Enter...")

    left.enable()
    right.enable()
    print(f"EN ON, 속도 {speed:.2f}")

    try:
        while True:
            cmd = input("명령: ").strip().lower()
            if cmd == "q":
                break
            if cmd in ("f", "b", "l", "r", "s"):
                mode = cmd
                apply(left, right, mode, speed)
            elif cmd == "+":
                speed = min(1.0, speed + 0.1)
                apply(left, right, mode, speed)
            elif cmd == "-":
                speed = max(0.1, speed - 0.1)
                apply(left, right, mode, speed)
            else:
                print("f b l r s + - q")
            time.sleep(0.05)
    finally:
        left.disable()
        right.disable()
        print("EN OFF, 종료")


if __name__ == "__main__":
    main()
