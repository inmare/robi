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


def _read_p5(path):
    raw = Path(path).read_bytes()
    if not raw.startswith(b"P5"):
        raise ValueError("P5 PGM이 아닙니다")
    idx = 2

    def token():
        nonlocal idx
        while idx < len(raw) and raw[idx] in b" \t\r\n":
            idx += 1
        if idx < len(raw) and raw[idx] == ord("#"):
            while idx < len(raw) and raw[idx] not in b"\n":
                idx += 1
            return token()
        start = idx
        while idx < len(raw) and raw[idx] not in b" \t\r\n":
            idx += 1
        return raw[start:idx]

    width = int(token())
    height = int(token())
    maxval = int(token())
    if maxval != 255:
        raise ValueError("maxval 255만 됩니다")
    if idx < len(raw) and raw[idx] in b"\r\n":
        idx += 1
        if raw[idx - 1] == 13 and idx < len(raw) and raw[idx] == 10:
            idx += 1
    body = raw[idx:]
    if len(body) < width * height:
        raise ValueError("PGM이 잘렸습니다")
    return width, height, body


class OccupancyGrid:
    def __init__(self, size_m=12.0, resolution=0.05):
        self.size_m = size_m
        self.resolution = resolution
        n = int(round(size_m / resolution))
        self.n = n
        self.origin_x = -size_m / 2
        self.origin_y = -size_m / 2
        self.log_odds = [[0.0] * n for _ in range(n)]
        self.occ_n = 0

    def world_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.n and 0 <= col < self.n:
            return row, col
        return None

    def cell_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y


    def _add(self, row, col, delta):
        old = self.log_odds[row][col]
        v = old + delta
        if v < L_MIN:
            v = L_MIN
        elif v > L_MAX:
            v = L_MAX
        was = old > 0.5
        now = v > 0.5
        if now and not was:
            self.occ_n += 1
        elif was and not now:
            self.occ_n -= 1
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

    @classmethod
    def from_pgm(cls, path, size_m=None, resolution=0.05):
        """save_pgm으로 쓴 P5를 다시 occupancy로 읽는다."""
        width, height, body = _read_p5(path)
        if width != height:
            raise ValueError(f"정사각 PGM만 됩니다 ({width}x{height})")
        if size_m is None:
            size_m = width * resolution
        grid = cls(size_m=size_m, resolution=resolution)
        if grid.n != width:
            grid = cls(size_m=width * resolution, resolution=resolution)
        if grid.n != width:
            raise ValueError("격자 크기와 PGM이 안 맞습니다")
        n = grid.n
        occ = 0
        for row in range(n):
            src = n - 1 - row
            for col in range(n):
                pix = body[row * n + col]
                if pix < 80:
                    lo = 2.0
                    occ += 1
                elif pix > 200:
                    lo = -2.0
                else:
                    lo = 0.0
                grid.log_odds[src][col] = lo
        grid.occ_n = occ
        return grid

    def render_ascii(self, cols=72, rows=36, pose=None, marks=None):
        """SSH 터미널용. #=벽  .=빈 공간  공백=미지  R=로봇. marks는 (x,y,글자)."""
        n = self.n
        overlays = []
        if marks:
            overlays.extend(marks)
        if pose is not None:
            overlays.append((pose[0], pose[1], "R"))
        else:
            overlays.append((0.0, 0.0, "R"))
        lines = []
        for out_r in range(rows):
            src_r0 = int((rows - 1 - out_r) * n / rows)
            src_r1 = int((rows - out_r) * n / rows)
            row_chars = []
            for out_c in range(cols):
                src_c0 = int(out_c * n / cols)
                src_c1 = int((out_c + 1) * n / cols)
                occupied = False
                free = False
                for sr in range(src_r0, max(src_r1, src_r0 + 1)):
                    for sc in range(src_c0, max(src_c1, src_c0 + 1)):
                        if sr >= n or sc >= n:
                            continue
                        lo = self.log_odds[sr][sc]
                        if lo > 0.5:
                            occupied = True
                        elif lo < -0.5:
                            free = True
                mark = None
                for mx, my, ch in overlays:
                    cell = self.world_to_cell(mx, my)
                    if cell is None:
                        continue
                    orow, ocol = cell
                    if src_r0 <= orow < src_r1 and src_c0 <= ocol < src_c1:
                        mark = ch
                if mark:
                    row_chars.append(mark)
                elif occupied:
                    row_chars.append("#")
                elif free:
                    row_chars.append(".")
                else:
                    row_chars.append(" ")
            lines.append("".join(row_chars))
        return "\n".join(lines)
