"""occupancy grid 위 A*. 벽은 로봇 반지름만큼 부풀린다."""

import heapq
import math

from robot.pins import INFLATE_M


def _add_disk(blocked, grid, x, y, radius_m):
    cell = grid.world_to_cell(x, y)
    if cell is None:
        return
    r = max(1, int(math.ceil(radius_m / grid.resolution)))
    row0, col0 = cell
    r2 = r * r
    n = grid.n
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if di * di + dj * dj > r2:
                continue
            rr, cc = row0 + di, col0 + dj
            if 0 <= rr < n and 0 <= cc < n:
                blocked.add((rr, cc))


def _inflate(grid, inflate_m, extra_disks=None):
    r = max(1, int(math.ceil(inflate_m / grid.resolution)))
    n = grid.n
    blocked = set()
    for row in range(n):
        for col in range(n):
            if grid.log_odds[row][col] <= 0.5:
                continue
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if di * di + dj * dj > r * r:
                        continue
                    rr, cc = row + di, col + dj
                    if 0 <= rr < n and 0 <= cc < n:
                        blocked.add((rr, cc))
    if extra_disks:
        pad = max(inflate_m, 0.0)
        for disk in extra_disks:
            _add_disk(blocked, grid, disk[0], disk[1], disk[2] + pad)
    return blocked


def _clear_around(blocked, cell, n, radius_cells=6):
    if cell is None:
        return
    row, col = cell
    for di in range(-radius_cells, radius_cells + 1):
        for dj in range(-radius_cells, radius_cells + 1):
            rr, cc = row + di, col + dj
            if 0 <= rr < n and 0 <= cc < n:
                blocked.discard((rr, cc))


def astar(grid, start_xy, goal_xy, inflate_m=INFLATE_M, free_only=False, extra_disks=None):
    """성공하면 월드 좌표 [(x,y), ...]. 실패하면 빈 리스트."""
    start = grid.world_to_cell(start_xy[0], start_xy[1])
    goal = grid.world_to_cell(goal_xy[0], goal_xy[1])
    if start is None or goal is None:
        return []
    blocked = _inflate(grid, inflate_m, extra_disks=extra_disks)
    _clear_around(blocked, start, grid.n)
    _clear_around(blocked, goal, grid.n)
    if goal in blocked:
        return []

    nbrs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    openh = [(h(start, goal), 0.0, start)]
    came = {start: None}
    gscore = {start: 0.0}

    while openh:
        _, g, cur = heapq.heappop(openh)
        if cur == goal:
            cells = []
            while cur is not None:
                cells.append(cur)
                cur = came[cur]
            cells.reverse()
            return [grid.cell_to_world(r, c) for r, c in cells]
        if g > gscore.get(cur, 1e9) + 1e-9:
            continue
        for dr, dc, step in nbrs:
            nxt = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nxt[0] < grid.n and 0 <= nxt[1] < grid.n):
                continue
            if nxt in blocked:
                continue
            lo = grid.log_odds[nxt[0]][nxt[1]]
            if free_only and lo > -0.2 and nxt != start and nxt != goal:
                continue
            extra = 1.4 if lo > -0.2 else 1.0
            ng = g + step * extra
            if ng < gscore.get(nxt, 1e9):
                gscore[nxt] = ng
                came[nxt] = cur
                heapq.heappush(openh, (ng + h(nxt, goal), ng, nxt))
    return []


def plan(grid, start_xy, goal_xy, extra_disks=None):
    """부풀리기를 줄여가며 길을 찾는다. 미지 칸은 벽으로 보지 않는다.

    extra_disks: 스캔에 안 잡힌 바닥 장애물 [(x, y, r), ...].
    SLAM이 그 칸을 비워도 A*는 계속 피한다.
    """
    for inflate_m in (INFLATE_M, 0.08, 0.0):
        path = astar(
            grid,
            start_xy,
            goal_xy,
            inflate_m=inflate_m,
            free_only=False,
            extra_disks=extra_disks,
        )
        if len(path) >= 2:
            return path
    return []
