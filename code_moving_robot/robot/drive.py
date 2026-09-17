"""좌우 바퀴 PWM. 홀 부호는 명령을 odometry에 넘긴다."""

from gpiozero import DigitalOutputDevice, PWMOutputDevice

from robot.pins import (
    LEFT_INVERT,
    LEFT_LEN,
    LEFT_LPWM,
    LEFT_REN,
    LEFT_RPWM,
    PWM_HZ,
    RIGHT_INVERT,
    RIGHT_LEN,
    RIGHT_LPWM,
    RIGHT_REN,
    RIGHT_RPWM,
)


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

    def close(self):
        self.disable()
        for dev in (self.rpwm, self.lpwm, self.ren, self.len):
            try:
                dev.close()
            except Exception:
                pass

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


def _sign(speed):
    if speed > 0.04:
        return 1
    if speed < -0.04:
        return -1
    return 0


class Drive:
    def __init__(self):
        self.left = Wheel(LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN, LEFT_INVERT)
        self.right = Wheel(RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN, RIGHT_INVERT)

    def enable(self):
        self.left.enable()
        self.right.enable()

    def disable(self):
        self.left.disable()
        self.right.disable()

    def close(self):
        self.disable()
        self.left.close()
        self.right.close()

    def set_speeds(self, left, right, odo=None):
        self.left.drive(left)
        self.right.drive(right)
        if odo is not None:
            odo.left_sign = _sign(left)
            odo.right_sign = _sign(right)

    def stop(self, odo=None):
        self.set_speeds(0.0, 0.0, odo)
