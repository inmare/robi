"""경로 선을 따라간다. 다음 점이 아니라 look-ahead 점을 본다."""

import math

from robot.path import point_along, wrap_angle
from robot.pins import ARRIVE_M, LOOKAHEAD_M, NAV_SPEED, SPIN_SPEED


class Follower:
    def __init__(self, points, lookahead_m=LOOKAHEAD_M):
        self.points = [(float(p[0]), float(p[1])) for p in points]
        self.i = 0
        self.lookahead_m = lookahead_m

    def done(self):
        return self.i >= len(self.points)

    def current(self):
        if self.done():
            return None
        return self.points[self.i]

    def remaining_points(self):
        return self.points[self.i :]

    def remaining_m(self, odo):
        if self.done():
            return 0.0
        acc = math.hypot(self.points[self.i][0] - odo.x, self.points[self.i][1] - odo.y)
        for j in range(self.i, len(self.points) - 1):
            x0, y0 = self.points[j][0], self.points[j][1]
            x1, y1 = self.points[j + 1][0], self.points[j + 1][1]
            acc += math.hypot(x1 - x0, y1 - y0)
        return acc

    def heading_err(self, odo):
        tgt = self._lookahead(odo)
        if tgt is None:
            return 0.0
        want = math.atan2(tgt[1] - odo.y, tgt[0] - odo.x)
        return wrap_angle(want - odo.yaw)

    def _advance(self, odo):
        while self.i < len(self.points) - 1:
            px, py = self.points[self.i]
            if math.hypot(px - odo.x, py - odo.y) < ARRIVE_M:
                self.i += 1
                continue
            nx, ny = self.points[self.i + 1]
            vx, vy = nx - px, ny - py
            l2 = vx * vx + vy * vy
            if l2 < 1e-12:
                self.i += 1
                continue
            t = ((odo.x - px) * vx + (odo.y - py) * vy) / l2
            cross = abs((odo.x - px) * vy - (odo.y - py) * vx) / math.sqrt(l2)
            if t > 0.92 and cross < 0.40:
                self.i += 1
                continue
            break
        if self.i == len(self.points) - 1:
            px, py = self.points[self.i]
            if math.hypot(px - odo.x, py - odo.y) < ARRIVE_M:
                self.i += 1

    def _lookahead(self, odo):
        if self.done():
            return None
        hit = point_along(self.points, self.i, self.lookahead_m)
        if hit is None:
            return self.points[self.i][0], self.points[self.i][1], 0.0
        return hit

    def step(self, drive, odo, speed=None):
        if self.done():
            drive.stop(odo)
            return True
        self._advance(odo)
        if self.done():
            drive.stop(odo)
            return True
        tx, ty, _heading = self._lookahead(odo)
        dx = tx - odo.x
        dy = ty - odo.y
        dist = math.hypot(dx, dy)
        if dist < ARRIVE_M * 0.6 and self.i >= len(self.points) - 1:
            self.i = len(self.points)
            drive.stop(odo)
            return True
        want = math.atan2(dy, dx)
        err = wrap_angle(want - odo.yaw)
        cur = self.points[self.i]
        cross = math.hypot(cur[0] - odo.x, cur[1] - odo.y)
        spin_lim = 0.85 if cross > 0.22 else 0.50
        if abs(err) > spin_lim:
            spin = min(SPIN_SPEED, speed if speed is not None else SPIN_SPEED)
            if err > 0:
                drive.set_speeds(-spin, spin, odo)
            else:
                drive.set_speeds(spin, -spin, odo)
        else:
            base = NAV_SPEED if speed is None else speed
            gain = 0.28 if cross > 0.22 else 0.20
            cap = 0.18 if cross > 0.22 else 0.12
            turn = max(-cap, min(cap, gain * err))
            drive.set_speeds(base - turn, base + turn, odo)
        return False
