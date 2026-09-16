"""스캔-투-맵 SLAM. 홀은 예측만, 벽은 라이다가 붙인다.

BreezySLAM/CoreSLAM과 같은 계열이다. C 확장 없이 occupancy grid 위에서
상관 스캔 매칭을 한다. 맞추지 못한 스캔은 지도에 넣지 않아 벽이 돌지 않게 한다.
"""

import math

from robot.localize import MIN_OCC, match_scan, match_yaw, wrap_angle

MIN_SLAM_SCORE = 0.12


class Slam:
    def __init__(self, grid, update_map=True):
        self.grid = grid
        self.update_map = update_map
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.misses = 0
        self.last_score = None

    def seed(self, odo):
        self.last_x = odo.x
        self.last_y = odo.y
        self.last_yaw = odo.yaw
        self.misses = 0

    def process(self, points, odo):
        if not points:
            return
        if self.last_x is None:
            self.seed(odo)

        if self.grid.occ_n < MIN_OCC:
            if self.update_map:
                self.grid.add_scan(odo.x, odo.y, odo.yaw, points)
            self.seed(odo)
            self.last_score = None
            return

        dist = math.hypot(odo.x - self.last_x, odo.y - self.last_y)
        dth = abs(wrap_angle(odo.yaw - self.last_yaw))
        grow = 1.0 + 0.55 * self.misses
        xy_m = min(0.45, (0.08 + 0.7 * dist + 0.06 * dth) * grow)
        yaw_rad = min(math.pi, (0.28 + 2.2 * dth + 0.5 * dist) * grow)
        if self.misses >= 2:
            yaw_rad = math.pi

        found = match_scan(
            self.grid, odo.x, odo.y, odo.yaw, points, xy_m=xy_m, yaw_rad=yaw_rad
        )
        if found is None or found.score < MIN_SLAM_SCORE:
            found = match_yaw(self.grid, odo.x, odo.y, points)

        ok = (
            found is not None
            and found.score >= MIN_SLAM_SCORE
            and not found.ambiguous
        )
        if ok:
            jump = abs(wrap_angle(found.yaw - odo.yaw))
            if jump > math.radians(70) and self.misses < 2:
                ok = False
        if ok:
            odo.set_pose(found.x, found.y, found.yaw)
            if self.update_map:
                self.grid.add_scan(odo.x, odo.y, odo.yaw, points)
            self.last_score = found.score
            self.seed(odo)
            return

        self.misses += 1
        self.last_score = None if found is None else found.score
        self.last_x = odo.x
        self.last_y = odo.y
        self.last_yaw = odo.yaw
