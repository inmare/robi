"""홀 펄스 + 명령 부호로 차동구동 포즈를 적분한다. 미끄러지면 어긋난다."""

import math

from gpiozero import DigitalInputDevice

from robot.pins import LEFT_DO, PULSE_M, RIGHT_DO, TRACK_M


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Odometry:
    def __init__(self, track_m=TRACK_M, pulse_m=PULSE_M):
        self.track_m = track_m
        self.pulse_m = pulse_m
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.left_sign = 0
        self.right_sign = 0
        self.left_count = 0
        self.right_count = 0
        self.left_hall = DigitalInputDevice(LEFT_DO, pull_up=True, bounce_time=0.002)
        self.right_hall = DigitalInputDevice(RIGHT_DO, pull_up=True, bounce_time=0.002)
        self.left_hall.when_activated = self._on_left
        self.right_hall.when_activated = self._on_right

    def _on_left(self):
        s = self.left_sign
        if s == 0:
            s = 1
        self.left_count += s
        self._step(s * self.pulse_m, 0.0)

    def _on_right(self):
        s = self.right_sign
        if s == 0:
            s = 1
        self.right_count += s
        self._step(0.0, s * self.pulse_m)

    def _step(self, dl, dr):
        dc = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.track_m
        self.x += dc * math.cos(self.yaw)
        self.y += dc * math.sin(self.yaw)
        self.yaw = wrap_angle(self.yaw + dyaw)

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.left_count = 0
        self.right_count = 0
