"""스캔을 occupancy grid에 맞춰 포즈를 보정한다.

홀은 바퀴당 자석 4개라 제자리 회전 yaw가 약 12°씩 뛴다.
저장 맵이 있으면 시작 칸에서 yaw를 전 방향으로 찾는다.
"""

import math

from robot.grid import RANGE_MAX, RANGE_MIN


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

MIN_OCC = 40
MIN_HITS = 16
MIN_LOCAL_SCORE = 0.08
MIN_HEADING_SCORE = 0.18


class Match:
    def __init__(self, x, y, yaw, score):
        self.x = x
        self.y = y
        self.yaw = wrap_angle(yaw)
        self.score = score
        self.ambiguous = False


def _frange(a, b, step):
    out = []
    x = a
    if step <= 0:
        return out
    while x <= b + 1e-9:
        out.append(x)
        x += step
    return out


def downsample_scan(points, buckets=72):
    slots = [None] * buckets
    for angle, rng in points:
        if rng < RANGE_MIN or rng > RANGE_MAX:
            continue
        a = wrap_angle(angle)
        i = int((a + math.pi) / (2 * math.pi) * buckets) % buckets
        prev = slots[i]
        if prev is None or rng < prev[1]:
            slots[i] = (angle, rng)
    return [p for p in slots if p is not None]


def score_pose(grid, x, y, yaw, points):
    used = 0
    score = 0.0
    here = grid.world_to_cell(x, y)
    if here is not None and grid.log_odds[here[0]][here[1]] > 0.5:
        score -= 0.8
    for angle, rng in points:
        wx = x + rng * math.cos(yaw + angle)
        wy = y + rng * math.sin(yaw + angle)
        cell = grid.world_to_cell(wx, wy)
        used += 1
        if cell is None:
            score -= 0.25
            continue
        lo = grid.log_odds[cell[0]][cell[1]]
        if lo > 0.5:
            score += 1.0
        elif lo < -0.5:
            score -= 0.4
        else:
            score -= 0.05
    if used < MIN_HITS:
        return None
    return score / used


def match_scan(grid, x, y, yaw, points, xy_m=0.12, yaw_rad=0.4):
    """오도메트리 예측 근처에서 스캔을 맵에 맞춘다. 창은 호출쪽에서 키운다."""
    if grid.occ_n < MIN_OCC:
        return None
    ds = downsample_scan(points, buckets=48)
    if len(ds) < MIN_HITS:
        return None
    xy_step = max(0.05, xy_m / 3.0)
    yaw_step = math.radians(6)
    if yaw_rad > 1e-6:
        yaw_step = min(max(math.radians(6), yaw_rad / 6.0), math.radians(10))
    best = (x, y, yaw)
    best_s = score_pose(grid, x, y, yaw, ds)
    if best_s is None:
        best_s = -1e9
    for dx in _frange(-xy_m, xy_m, xy_step):
        for dy in _frange(-xy_m, xy_m, xy_step):
            for dth in _frange(-yaw_rad, yaw_rad, yaw_step):
                s = score_pose(grid, x + dx, y + dy, yaw + dth, ds)
                if s is not None and s > best_s:
                    best_s = s
                    best = (x + dx, y + dy, yaw + dth)
    fx, fy, fyaw = best
    ds2 = downsample_scan(points, buckets=72)
    for dx in _frange(-0.08, 0.08, 0.03):
        for dy in _frange(-0.08, 0.08, 0.03):
            for dth in _frange(-math.radians(8), math.radians(8), math.radians(2)):
                s = score_pose(grid, fx + dx, fy + dy, fyaw + dth, ds2)
                if s is not None and s > best_s:
                    best_s = s
                    best = (fx + dx, fy + dy, fyaw + dth)
    if best_s < MIN_LOCAL_SCORE:
        return None
    return Match(best[0], best[1], best[2], best_s)


def match_local(grid, x, y, yaw, points, xy_m=0.10, yaw_rad=0.38):
    return match_scan(grid, x, y, yaw, points, xy_m=xy_m, yaw_rad=yaw_rad)


def match_yaw(grid, x, y, points):
    """xy는 고정하고 yaw만 한 바퀴 본다. 주행 중 로컬 매칭 실패용."""
    if grid.occ_n < MIN_OCC:
        return None
    ds = downsample_scan(points, buckets=48)
    if len(ds) < MIN_HITS:
        return None
    best_yaw = None
    best_s = -1e9
    for yaw in _frange(0.0, 2 * math.pi - 0.01, math.radians(8)):
        s = score_pose(grid, x, y, yaw, ds)
        if s is not None and s > best_s:
            best_s = s
            best_yaw = yaw
    if best_yaw is None:
        return None
    ds2 = downsample_scan(points, buckets=72)
    for dth in _frange(-math.radians(10), math.radians(10), math.radians(2)):
        s = score_pose(grid, x, y, best_yaw + dth, ds2)
        if s is not None and s > best_s:
            best_s = s
            best_yaw = best_yaw + dth
    if best_s < MIN_HEADING_SCORE:
        return None
    return Match(x, y, best_yaw, best_s)


def match_heading(grid, x, y, points, xy_m=0.35, min_score=MIN_HEADING_SCORE):
    """위치는 대략 알고 yaw는 모를 때. 한 바퀴를 다 본다."""
    if grid.occ_n < MIN_OCC:
        return None
    ds = downsample_scan(points)
    if len(ds) < MIN_HITS:
        return None
    best = None
    best_s = -1e9
    second_s = -1e9
    second_yaw = None
    for yaw in _frange(0.0, 2 * math.pi - 0.01, math.radians(10)):
        for dx in _frange(-xy_m, xy_m, 0.10):
            for dy in _frange(-xy_m, xy_m, 0.10):
                s = score_pose(grid, x + dx, y + dy, yaw, ds)
                if s is None:
                    continue
                if s > best_s:
                    second_s = best_s
                    second_yaw = None if best is None else best[2]
                    best_s = s
                    best = (x + dx, y + dy, yaw)
                elif s > second_s:
                    second_s = s
                    second_yaw = yaw
    if best is None:
        return None
    fx, fy, fyaw = best
    for dx in _frange(-0.12, 0.12, 0.04):
        for dy in _frange(-0.12, 0.12, 0.04):
            for dth in _frange(-math.radians(12), math.radians(12), math.radians(2)):
                s = score_pose(grid, fx + dx, fy + dy, fyaw + dth, ds)
                if s is not None and s > best_s:
                    best_s = s
                    best = (fx + dx, fy + dy, fyaw + dth)
    if best_s < min_score:
        return None
    found = Match(best[0], best[1], best[2], best_s)
    found.ambiguous = False
    if second_yaw is not None:
        gap = abs(wrap_angle(found.yaw - second_yaw))
        if gap > 1.2 and best_s - second_s < 0.06:
            found.ambiguous = True
    return found
