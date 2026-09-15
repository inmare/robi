"""홀 카운트로 1m 직진 시험. 핀은 learn/pin map.md."""

import math
import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice

# 왼쪽 DO = GPIO5 (물리 29), 오른쪽 DO = GPIO16 (물리 36)
LEFT_DO = 5
RIGHT_DO = 16
MAGNETS = 4
WHEEL_D = 0.066
# 자석은 타이어 바깥에서 5~7mm 안쪽. 홀이 세는 건 바퀴 회전 횟수라
# 1m 이동은 자석 원 지름이 아니라 바닥과 닿는 타이어 지름으로 계산함
MAGNET_INSET = 0.006
MAGNET_CIRCLE_D = WHEEL_D - 2 * MAGNET_INSET
TARGET_M = 1.0
PULSE_M = math.pi * WHEEL_D / MAGNETS
TARGET_PULSES = TARGET_M / PULSE_M

LEFT_INVERT = False
RIGHT_INVERT = False
BASE_SPEED = 0.25
KP = 0.06
MIN_SPEED = 0.12
MAX_SPEED = 0.4
PWM_HZ = 1000
MAX_PULSE_DIFF = MAGNETS * 2

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


def on_right():
    global right_count
    right_count += 1


def clamp(speed):
    if speed < MIN_SPEED:
        return MIN_SPEED
    if speed > MAX_SPEED:
        return MAX_SPEED
    return speed


def meters(count):
    return count * PULSE_M


def main():
    left_wheel = Wheel(LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN, LEFT_INVERT)
    right_wheel = Wheel(RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN, RIGHT_INVERT)

    left_hall.when_activated = on_left
    right_hall.when_activated = on_right

    print(
        f"자석 {MAGNETS}개, 타이어 {WHEEL_D * 1000:.0f}mm, "
        f"자석은 테두리에서 {MAGNET_INSET * 1000:.0f}mm 안쪽 "
        f"(자석 원 {MAGNET_CIRCLE_D * 1000:.0f}mm)"
    )
    print(f"목표 {TARGET_M:.2f}m = 펄스 {TARGET_PULSES:.1f}개 (타이어 둘레 기준)")
    print("바닥에 두고 1m 직선 공간이 있는지 보세요.")
    print("모터 6V는 이 프로그램이 뜬 뒤에 켜세요. Ctrl+C 즉시 정지")
    input("준비되면 Enter...")

    left_wheel.enable()
    right_wheel.enable()
    left_wheel.drive(BASE_SPEED)
    right_wheel.drive(BASE_SPEED)
    print("직진 시작")

    last_print = 0.0
    reason = "중지"
    try:
        while True:
            error = left_count - right_count
            if abs(error) > MAX_PULSE_DIFF:
                reason = "좌우 펄스 차이가 커서 정지"
                break

            avg = (left_count + right_count) / 2
            if avg >= TARGET_PULSES:
                reason = "1m 도달"
                break

            left_speed = clamp(BASE_SPEED - KP * error)
            right_speed = clamp(BASE_SPEED + KP * error)
            left_wheel.drive(left_speed)
            right_wheel.drive(right_speed)

            now = time.monotonic()
            if now - last_print >= 0.3:
                last_print = now
                print(
                    f"왼 {left_count:3d} {meters(left_count):.2f}m {left_speed:.2f} | "
                    f"오른 {right_count:3d} {meters(right_count):.2f}m {right_speed:.2f}"
                )
            time.sleep(0.02)
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        left_wheel.disable()
        right_wheel.disable()
        print(
            f"EN OFF. {reason}. "
            f"왼 {left_count}회 {meters(left_count):.2f}m, "
            f"오른 {right_count}회 {meters(right_count):.2f}m"
        )


if __name__ == "__main__":
    main()
