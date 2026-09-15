"""로봇 기준 occupancy grid. 홀 포즈가 생기면 그 좌표로 add_scan 한다."""

import math
from pathlib import Path

# 로그오즈. 0이면 미지, 양수면 occupied, 음수면 free
L_OCC = math.log(0.7 / 0.3)
L_FREE = math.log(0.3 / 0.7)
L_MIN = -4.0
L_MAX = 4.0
RANGE_MIN = 0.12
RANGE_MAX = 8.0


class OccupancyGrid:
    def __init__(self, size_m=12.0, resolution=0.05):
        self.size_m = size_m
        self.resolution = resolution
        n = int(round(size_m / resolution))
        self.n = n
        self.origin_x = -size_m / 2
        self.origin_y = -size_m / 2
        self.log_odds = [[0.0] * n for _ in range(n)]

    def world_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.n and 0 <= col < self.n:
            return row, col
        return None

    def _add(self, row, col, delta):
        v = self.log_odds[row][col] + delta
        if v < L_MIN:
            v = L_MIN
        elif v > L_MAX:
            v = L_MAX
        self.log_odds[row][col] = v

    def _walk(self, r0, c0, r1, c1):
        """끝 칸 직전 자유공간, 끝 칸은 호출쪽에서 occupied."""
        dr = r1 - r0
        dc = c1 - c0
        steps = max(abs(dr), abs(dc))
        if steps == 0:
            return
        cells = []
        for i in range(steps):
            t = i / steps
            row = int(round(r0 + dr * t))
            col = int(round(c0 + dc * t))
            if 0 <= row < self.n and 0 <= col < self.n:
                cells.append((row, col))
        if not cells:
            return
        last = (r1, c1)
        for row, col in cells:
            if (row, col) == last:
                continue
            self._add(row, col, L_FREE)

    def add_scan(self, x, y, yaw, points):
        """points는 Lidar.read()와 같은 [(angle_rad, range_m), ...]."""
        origin = self.world_to_cell(x, y)
        if origin is None:
            return
        r0, c0 = origin
        for angle, rng in points:
            if rng < RANGE_MIN or rng > RANGE_MAX:
                continue
            wx = x + rng * math.cos(yaw + angle)
            wy = y + rng * math.sin(yaw + angle)
            hit = self.world_to_cell(wx, wy)
            if hit is None:
                continue
            r1, c1 = hit
            self._walk(r0, c0, r1, c1)
            self._add(r1, c1, L_OCC)

    def save_pgm(self, path):
        """회색 PGM. 검정=벽, 회색=미지, 흰=빈 공간. 이미지 뷰어로 연다."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        n = self.n
        header = f"P5\n{n} {n}\n255\n".encode("ascii")
        body = bytearray(n * n)
        for row in range(n):
            # PGM은 위가 y+. 그리드 row 0은 y- 쪽이므로 뒤집는다
            src = n - 1 - row
            for col in range(n):
                lo = self.log_odds[src][col]
                if lo > 0.5:
                    pix = 0
                elif lo < -0.5:
                    pix = 255
                else:
                    pix = 160
                body[row * n + col] = pix
        path.write_bytes(header + body)
        return path
