"""경로 점을 홀 포즈로 따라간다. 전방 장애물은 호출쪽에서 본다."""

import math

from robot.odometry import wrap_angle
from robot.pins import ARRIVE_M, NAV_SPEED, SPIN_SPEED


class Follower:
    def __init__(self, points):
        self.points = list(points)
        self.i = 0

    def done(self):
        return self.i >= len(self.points)

    def current(self):
        if self.done():
            return None
        return self.points[self.i]

    def step(self, drive, odo):
        if self.done():
            drive.stop(odo)
            return True
        tx, ty = self.points[self.i][0], self.points[self.i][1]
        dx = tx - odo.x
        dy = ty - odo.y
        dist = math.hypot(dx, dy)
        if dist < ARRIVE_M:
            self.i += 1
            if self.done():
                drive.stop(odo)
                return True
            return False
        want = math.atan2(dy, dx)
        err = wrap_angle(want - odo.yaw)
        if abs(err) > 0.45:
            spin = SPIN_SPEED
            if err > 0:
                drive.set_speeds(-spin, spin, odo)
            else:
                drive.set_speeds(spin, -spin, odo)
        else:
            base = NAV_SPEED
            turn = max(-0.12, min(0.12, 0.2 * err))
            drive.set_speeds(base - turn, base + turn, odo)
        return False
