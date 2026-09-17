"""바닥 장애물·라이다 걸림 회복.

스캔 평면 아래 물건은 맵에 안 그려진다. DWA/TEB도 스캔이 끊기면 못 쓴다.
그래서 Bug 식으로 짧게 후진 → 빈 옆으로 이동한 뒤, 부딪힌 자리를
extra_disk로 남기고 A*로 남은 점에 다시 붙는다.
"""

import math
import time

from robot.localize import wrap_angle
from robot.pins import (
    HIT_AHEAD_M,
    HIT_DEPTH_M,
    NAV_SPEED,
    RECOVER_BACK_M,
    RECOVER_CLEAR_M,
    RECOVER_HOP_S,
    RECOVER_REVERSE_S,
    RECOVER_SECTOR_DEG,
    RECOVER_SIDE_M,
    RECOVER_TURN_RAD,
    RECOVER_TURN_S,
    RECOVER_WAIT_S,
    REJOIN_OFF_M,
    SCAN_DISK_FRONT_DEG,
    SCAN_DISK_MAX_M,
    SCAN_DISK_R,
    SPIN_SPEED,
    SURVEY_RAD,
    SURVEY_TURN_S,
    VIRTUAL_BLOCK_M,
    YAW_STALL_RAD,
    YAW_STALL_S,
    WHEEL_SKEW_PULSES,
    WHEEL_SKEW_RATIO,
    WHEEL_SKEW_S,
)
from robot.planner import plan


def wheel_skew_side(dl, dr, left_sign, right_sign, min_pulses=WHEEL_SKEW_PULSES, ratio=WHEEL_SKEW_RATIO):
    """양쪽 모터가 도는데 한쪽 홀만 뛰면 그 바퀴가 걸린 것. 제자리 회전도 본다. 'left'/'right'/None."""
    if left_sign == 0 or right_sign == 0:
        return None
    mx = max(dl, dr)
    mn = min(dl, dr)
    if mx < min_pulses:
        return None
    if mn > ratio * mx:
        return None
    return "left" if dl < dr else "right"


class HallWatch:
    """모터가 양쪽 다 도는데 한쪽 홀만 뛰는지 본다. 앞뒤 진동은 펄스 절대값으로 합친다."""

    def __init__(self, window_s=WHEEL_SKEW_S):
        self.window_s = window_s
        self.t0 = None
        self.left0 = 0
        self.right0 = 0
        self.left_pulses = 0
        self.right_pulses = 0

    def reset(self):
        self.t0 = None
        self.left_pulses = 0
        self.right_pulses = 0

    def poll(self, odo, now):
        if self.t0 is None:
            self.t0 = now
            self.left0 = odo.left_count
            self.right0 = odo.right_count
            self.left_pulses = 0
            self.right_pulses = 0
            return None
        self.left_pulses += abs(odo.left_count - self.left0)
        self.right_pulses += abs(odo.right_count - self.right0)
        self.left0 = odo.left_count
        self.right0 = odo.right_count
        if now - self.t0 < self.window_s:
            return None
        side = wheel_skew_side(
            self.left_pulses, self.right_pulses, odo.left_sign, odo.right_sign
        )
        self.t0 = now
        self.left_pulses = 0
        self.right_pulses = 0
        return side


def sector_min_range(points, center_rad, half_rad):
    """해당 부채꼴의 최근 거리. 점이 없으면 멀리 열린 것으로 본다."""
    if not points:
        return 0.0
    hits = []
    for ang, rng in points:
        if rng <= 0.05:
            continue
        if abs(wrap_angle(ang - center_rad)) <= half_rad:
            hits.append(rng)
    if not hits:
        return 8.0
    return min(hits)


def dist_point_seg(p, a, b):
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def dist_to_polyline(xy, points):
    if not points:
        return 1e9
    best = math.hypot(xy[0] - points[0][0], xy[1] - points[0][1])
    for i in range(len(points) - 1):
        best = min(best, dist_point_seg(xy, points[i], points[i + 1]))
    return best


def hop_pose(x, y, yaw, side, turn_rad, hop_m):
    tyaw = wrap_angle(yaw + side * turn_rad)
    return (
        x + hop_m * math.cos(tyaw),
        y + hop_m * math.sin(tyaw),
        tyaw,
    )


def side_clearance(points, side):
    """90° 옆 여유. 앞옆(55°)까지 min 하면, 앞에 선 사람 때문에 옆 빈길을 놓친다."""
    return sector_min_range(
        points, side * math.pi / 2, math.radians(RECOVER_SECTOR_DEG)
    )


def dist_to_disks(xy, disks):
    """디스크 표면까지 거리. 안이면 음수. 없으면 멀리."""
    if not disks:
        return 8.0
    best = None
    for d in disks:
        gap = math.hypot(xy[0] - d[0], xy[1] - d[1]) - d[2]
        if best is None or gap < best:
            best = gap
    return best if best is not None else 8.0


def grid_segment_blocked(grid, a, b, radius=0.14, step=0.05):
    """후보 우회 궤적이 지도에 이미 있는 벽을 스치는지 본다."""
    if grid is None:
        return False
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    samples = max(1, int(math.ceil(length / step)))
    cell_r = max(1, int(math.ceil(radius / grid.resolution)))
    for i in range(1, samples + 1):
        t = i / samples
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        cell = grid.world_to_cell(x, y)
        if cell is None:
            return True
        row, col = cell
        for dr in range(-cell_r, cell_r + 1):
            for dc in range(-cell_r, cell_r + 1):
                if dr * dr + dc * dc > cell_r * cell_r:
                    continue
                rr, cc = row + dr, col + dc
                if 0 <= rr < grid.n and 0 <= cc < grid.n:
                    if grid.log_odds[rr][cc] > 0.5:
                        return True
    return False


def hit_corridor(x, y, yaw, ahead_m=HIT_AHEAD_M, depth_m=HIT_DEPTH_M, radius=VIRTUAL_BLOCK_M):
    """실패한 진행 방향 앞을 여러 원으로 막아 같은 통로로 안 돌아가게 한다."""
    n = 3
    out = []
    span = max(depth_m - ahead_m, 0.05)
    for i in range(n):
        t = ahead_m + span * i / max(n - 1, 1)
        out.append(
            (
                x + t * math.cos(yaw),
                y + t * math.sin(yaw),
                radius,
            )
        )
    return out


def choose_detour(
    scan, x, y, yaw, rest, clear_m=RECOVER_CLEAR_M, disks=None, grid=None
):
    """안전한 후보 중 원래 진행 방향을 유지하면서 더 열린 옆길을 고른다."""
    min_go = min(0.18, clear_m)
    rest = list(rest or [])
    left = side_clearance(scan, 1)
    right = side_clearance(scan, -1)
    route_target, _ = look_ahead_target((x, y), rest, min_m=0.65)
    route_dist = (
        math.hypot(route_target[0] - x, route_target[1] - y)
        if route_target is not None
        else 0.0
    )
    best_side = 0
    best_score = None
    for side, cl in ((1, left), (-1, right)):
        if cl < min_go:
            continue
        hx, hy, tyaw = hop_pose(x, y, yaw, side, RECOVER_TURN_RAD, RECOVER_SIDE_M)
        look = (
            hx + 0.28 * math.cos(tyaw),
            hy + 0.28 * math.sin(tyaw),
        )
        d_disk = dist_to_disks(look, disks)
        d_land = dist_to_disks((hx, hy), disks)
        if d_land < 0.08:
            continue
        if grid_segment_blocked(grid, (x, y), look):
            continue
        if rest:
            d_path = dist_to_polyline((hx, hy), rest)
            target = route_target if route_target is not None else rest[0]
            want = math.atan2(target[1] - hy, target[0] - hx)
            turn_after = abs(wrap_angle(want - tyaw))
            progress = route_dist - math.hypot(target[0] - look[0], target[1] - look[1])
        else:
            d_path = 0.0
            turn_after = 0.0
            progress = 0.0
        tight_penalty = max(0.0, clear_m + 0.15 - cl)
        score = (
            -0.55 * min(cl, 1.0)
            + 0.55 * d_path
            + 0.45 * turn_after
            - 1.80 * progress
            + 2.20 * tight_penalty
            - 0.70 * min(max(d_disk, -0.2), 1.6)
        )
        if best_score is None or score < best_score:
            best_score = score
            best_side = side
    if best_side == 0 and max(left, right) >= 0.16:
        best_side = 1 if left >= right else -1
    return best_side, left, right


def prefer_side(yaw, here, target):
    """목표점이 로봇 왼쪽이면 +1, 오른쪽이면 -1."""
    if target is None:
        return 0
    want = math.atan2(target[1] - here[1], target[0] - here[0])
    err = wrap_angle(want - yaw)
    if err > 0.15:
        return 1
    if err < -0.15:
        return -1
    return 0


def pick_side(points, prefer=0, clear_m=RECOVER_CLEAR_M, rest=None, pose=None):
    """+1 왼쪽, -1 오른쪽, 0 양쪽 막힘."""
    x, y, yaw = (0.0, 0.0, 0.0) if pose is None else pose
    rest = list(rest or [])
    if not rest and prefer != 0:
        rest = [(1.0, 0.45 * prefer)]
    return choose_detour(points, x, y, yaw, rest, clear_m=clear_m, disks=None)


def skip_near(points, hit_xy, skip_m=0.40):
    rest = [p for p in points if math.hypot(p[0] - hit_xy[0], p[1] - hit_xy[1]) > skip_m]
    if rest:
        return rest
    if points:
        return [points[-1]]
    return []


def point_in_disks(p, disks, pad=0.18):
    for d in disks or []:
        if math.hypot(p[0] - d[0], p[1] - d[1]) <= d[2] + pad:
            return True
    return False


def skip_blocked(points, disks, pad=0.10):
    """기억한 장애물 안·바로 앞 점은 버린다. 같은 방향으로 다시 붙지 않게."""
    if not points:
        return []
    if not disks:
        return list(points)
    kept = [p for p in points if not point_in_disks(p, disks, pad=pad)]
    if kept:
        return kept
    return [points[-1]]


def thin_path(points, step_m=0.18):
    if not points:
        return []
    out = [points[0]]
    acc = 0.0
    for i in range(1, len(points)):
        x0, y0 = out[-1][0], out[-1][1]
        x1, y1 = points[i][0], points[i][1]
        acc += math.hypot(x1 - x0, y1 - y0)
        if acc >= step_m:
            out.append(points[i])
            acc = 0.0
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out


def seg_hits_disks(a, b, disks, pad=0.12):
    for disk in disks or []:
        if dist_point_seg((disk[0], disk[1]), a, b) < disk[2] + pad:
            return True
    return False


def splice_path(grid, odo, rest, extra_disks):
    """우회한 자리에서 남은 점으로. 기억 장애물을 지나는 빵가루는 잇지 않는다."""
    rest = skip_blocked(rest, extra_disks)
    if not rest:
        return []
    here = (odo.x, odo.y)
    path = [here]
    for nxt in rest:
        nxt = (float(nxt[0]), float(nxt[1]))
        cur = path[-1]
        blocked_pt = point_in_disks(nxt, extra_disks)
        if blocked_pt or seg_hits_disks(cur, nxt, extra_disks, pad=0.18):
            chunk = plan(grid, cur, nxt, extra_disks=extra_disks)
            if len(chunk) >= 2:
                path.extend(chunk[1:])
            continue
        path.append(nxt)
    if len(path) < 2 and rest:
        chunk = plan(grid, here, rest[-1], extra_disks=None)
        if len(chunk) >= 2:
            return thin_path(chunk)
        return []
    if len(path) < 2:
        return []
    return thin_path(path)


def merge_disk(disks, disk, cell=0.14, cap=48):
    x, y, r = disk
    for i, d in enumerate(disks):
        if math.hypot(d[0] - x, d[1] - y) < cell:
            disks[i] = ((d[0] + x) / 2.0, (d[1] + y) / 2.0, max(d[2], r))
            return
    disks.append(disk)
    if len(disks) > cap:
        del disks[0 : len(disks) - cap]


def scan_hits_to_disks(odo, points, grid=None, max_m=SCAN_DISK_MAX_M, radius=SCAN_DISK_R):
    """맵에 없던 가까운 전방 히트만 가상 장애물로. 벽 칸은 빼서 A*가 복도를 막지 않게."""
    if not points:
        return []
    half = math.radians(SCAN_DISK_FRONT_DEG)
    out = []
    for ang, rng in points:
        if rng < 0.15 or rng > max_m:
            continue
        if abs(wrap_angle(ang)) > half:
            continue
        wx = odo.x + rng * math.cos(odo.yaw + ang)
        wy = odo.y + rng * math.sin(odo.yaw + ang)
        if grid is not None:
            cell = grid.world_to_cell(wx, wy)
            if cell is not None and grid.log_odds[cell[0]][cell[1]] > 0.5:
                continue
        merge_disk(out, (wx, wy, radius), cell=0.12, cap=24)
    return out


def look_ahead_target(here, rest, min_m=0.55, max_skip=14):
    if not rest:
        return None, 0
    for i, p in enumerate(rest):
        if i > max_skip:
            break
        if math.hypot(p[0] - here[0], p[1] - here[1]) >= min_m:
            return (float(p[0]), float(p[1])), i
    last = rest[-1]
    return (float(last[0]), float(last[1])), len(rest) - 1


def rejoin_path(grid, odo, rest, extra_disks, scan=None):
    """지금 자리에서 원래 경로의 앞쪽 점으로 A* 후 나머지를 잇는다."""
    rest_orig = [(float(p[0]), float(p[1])) for p in (rest or [])]
    if not rest_orig:
        return []
    disks = list(extra_disks or [])
    if scan:
        for d in scan_hits_to_disks(odo, scan, grid=grid):
            merge_disk(disks, d)
    rest = skip_blocked(rest_orig, disks)
    if not rest:
        rest = [rest_orig[-1]]
    here = (odo.x, odo.y)
    off = dist_to_polyline(here, rest)
    first_hit = seg_hits_disks(here, rest[0], disks, pad=0.18) or point_in_disks(
        rest[0], disks
    )
    if off < REJOIN_OFF_M and not first_hit:
        path = splice_path(grid, odo, rest, extra_disks)
        if len(path) >= 2:
            return path
    for min_m in (0.45, 0.85, 1.30, 2.20):
        tgt, idx = look_ahead_target(here, rest, min_m=min_m)
        if tgt is None:
            break
        chunk = plan(grid, here, tgt, extra_disks=disks)
        if len(chunk) >= 2:
            tail = rest[idx + 1 :]
            return thin_path(chunk + tail)
    path = splice_path(grid, odo, rest, disks)
    if len(path) >= 2:
        return path
    for tgt in (rest[-1], rest_orig[-1]):
        chunk = plan(grid, here, tgt, extra_disks=None)
        if len(chunk) >= 2:
            return thin_path(chunk)
    return []


class Recoverer:
    def __init__(self):
        self.state = "idle"
        self.reason = ""
        self.tries = 0
        self.side = 0
        self.disks = []
        self._t0 = 0.0
        self._stuck = (0.0, 0.0, 0.0)
        self._hop_from = (0.0, 0.0)
        self._target_yaw = 0.0
        self._target = None
        self._rest = []
        self._turn_rad = RECOVER_TURN_RAD
        self._hop_m = RECOVER_SIDE_M
        self._survey_phase = 0
        self._survey_targets = []
        self._grid = None
        self._spin_yaw = None
        self._spin_move_t = 0.0
        self.log = None

    def active(self):
        return self.state not in ("idle",)

    def reset(self):
        self.state = "idle"
        self.reason = ""
        self.tries = 0
        self.side = 0
        self.disks = []
        self.log = None

    def abort(self):
        self.state = "idle"
        self.log = None

    def ingest_scan(self, odo, scan, grid=None):
        if grid is not None:
            self._grid = grid

    def start(self, odo, reason, target=None, rest=None, grid=None, stuck_wheel=None):
        self.state = "reverse"
        self.reason = reason
        self.tries += 1
        self.side = 0
        self._t0 = time.monotonic()
        self._stuck = (odo.x, odo.y, odo.yaw)
        self._target = target
        self._rest = list(rest or [])
        self._grid = grid
        if target is not None and not self._rest:
            self._rest = [target]
        route_target, _ = look_ahead_target(
            (odo.x, odo.y), self._rest, min_m=0.55
        )
        route_yaw = odo.yaw
        if route_target is not None:
            route_yaw = math.atan2(
                route_target[1] - odo.y, route_target[0] - odo.x
            )
        if len(self.disks) > 9:
            self.disks = self.disks[-9:]
        self.disks.extend(hit_corridor(odo.x, odo.y, route_yaw))
        if stuck_wheel in ("left", "right"):
            side = 1 if stuck_wheel == "left" else -1
            nx = -side * math.sin(odo.yaw)
            ny = side * math.cos(odo.yaw)
            self.disks.append((odo.x + 0.20 * nx, odo.y + 0.20 * ny, 0.18))
        self.log = f"후진 {RECOVER_BACK_M:.2f}m ({reason})"

    def _begin_spin(self, odo, now):
        self._spin_yaw = odo.yaw
        self._spin_move_t = now

    def _spin_stuck(self, odo, now, scan=None):
        """회전에 몸체가 걸리면 yaw가 안 변한다. 그때는 더 돌리지 않는다."""
        if self._spin_yaw is None:
            self._begin_spin(odo, now)
            return False
        moved = abs(wrap_angle(odo.yaw - self._spin_yaw))
        if moved >= YAW_STALL_RAD:
            self._spin_yaw = odo.yaw
            self._spin_move_t = now
            return False
        if scan and sector_min_range(scan, 0.0, math.radians(22)) < 0.16:
            return True
        return now - self._spin_move_t >= YAW_STALL_S

    def _go_open_side(self, drive, odo, scan, now):
        rest = skip_blocked(self._rest, self.disks)
        side, left, right = choose_detour(
            scan or [],
            odo.x,
            odo.y,
            odo.yaw,
            rest,
            disks=self.disks,
            grid=self._grid,
        )
        if side == 0:
            drive.stop(odo)
            self.state = "idle"
            self.log = (
                f"빈 옆길 없음 L={left:.2f} R={right:.2f}. 여기서 재연결"
            )
            return "ok"
        self.side = side
        name = "왼쪽" if side > 0 else "오른쪽"
        self._turn_rad = RECOVER_TURN_RAD
        self._hop_m = RECOVER_SIDE_M
        if min(left, right) < 0.35:
            self._turn_rad = min(self._turn_rad, 0.32)
            self._hop_m = min(self._hop_m, 0.18)
        self._target_yaw = wrap_angle(odo.yaw + side * self._turn_rad)
        self.state = "turn"
        self._t0 = now
        self._begin_spin(odo, now)
        self.log = (
            f"{name} 빈길로  L={left:.2f}m R={right:.2f}m  "
            f"각 {math.degrees(self._turn_rad):.0f}° 옆 {self._hop_m:.2f}m"
        )
        return None

    def hit_xy(self):
        if not self.disks:
            return self._stuck[0], self._stuck[1]
        return self.disks[-1][0], self.disks[-1][1]

    def step(self, drive, odo, scan, now=None):
        """모터 한 틱. 'ok' 우회 끝, 'fail' 포기, None 진행 중."""
        if now is None:
            now = time.monotonic()
        if self.state == "idle":
            return None
        if self.state == "reverse":
            return self._reverse(drive, odo, now)
        if self.state == "wait_lidar":
            return self._wait_lidar(drive, odo, scan, now)
        if self.state == "survey":
            return self._survey(drive, odo, scan, now)
        if self.state == "turn":
            return self._turn(drive, odo, now, scan)
        if self.state == "hop":
            return self._hop(drive, odo, scan, now)
        return None

    def _fail(self, drive, odo, msg):
        drive.stop(odo)
        self.state = "idle"
        self.log = msg
        return "fail"

    def _reverse(self, drive, odo, now):
        sx, sy, _syaw = self._stuck
        gone = math.hypot(odo.x - sx, odo.y - sy)
        if gone >= RECOVER_BACK_M:
            drive.stop(odo)
            self.state = "wait_lidar"
            self._t0 = now
            self.log = "후진 끝. 라이다 대기"
            return None
        if now - self._t0 >= RECOVER_REVERSE_S:
            if gone < 0.06:
                return self._fail(drive, odo, "후진 불가. 손으로 빼세요")
            drive.stop(odo)
            self.state = "wait_lidar"
            self._t0 = now
            self.log = "후진 시간 초과. 라이다 대기"
            return None
        drive.set_speeds(-NAV_SPEED * 0.85, -NAV_SPEED * 0.85, odo)
        return None

    def _wait_lidar(self, drive, odo, scan, now):
        drive.stop(odo)
        if scan and len(scan) >= 12:
            self.ingest_scan(odo, scan, self._grid)
            return self._go_open_side(drive, odo, scan, now)
        if now - self._t0 >= RECOVER_WAIT_S:
            return self._fail(drive, odo, "라이다가 다시 안 돕니다")
        return None

    def _survey(self, drive, odo, scan, now):
        if scan:
            self.ingest_scan(odo, scan, self._grid)
        if self._spin_stuck(odo, now, scan):
            drive.stop(odo)
            self.state = "idle"
            self.log = "수색 회전 걸림. 여기서 재연결"
            return "ok"
        err = wrap_angle(self._target_yaw - odo.yaw)
        done = abs(err) < 0.14 or now - self._t0 >= SURVEY_TURN_S
        if done:
            drive.stop(odo)
            return self._go_open_side(drive, odo, scan, now)
        spin = SPIN_SPEED * 0.85
        if err > 0:
            drive.set_speeds(-spin, spin, odo)
        else:
            drive.set_speeds(spin, -spin, odo)
        return None

    def _after_survey(self, drive, odo, scan, now):
        return self._go_open_side(drive, odo, scan, now)

    def _turn(self, drive, odo, now, scan=None):
        if self._spin_stuck(odo, now, scan):
            drive.stop(odo)
            self.state = "idle"
            self.log = "우회 회전 걸림. 여기서 재연결"
            return "ok"
        err = wrap_angle(self._target_yaw - odo.yaw)
        if abs(err) < 0.12:
            drive.stop(odo)
            self.state = "hop"
            self._t0 = now
            self._hop_from = (odo.x, odo.y)
            self.log = "회전 끝. 빈길로 이동"
            return None
        if now - self._t0 >= RECOVER_TURN_S:
            drive.stop(odo)
            self.state = "idle"
            self.log = "회전 시간 초과. 여기서 재연결"
            return "ok"
        spin = SPIN_SPEED
        if err > 0:
            drive.set_speeds(-spin, spin, odo)
        else:
            drive.set_speeds(spin, -spin, odo)
        return None

    def _hop(self, drive, odo, scan, now):
        gone = math.hypot(odo.x - self._hop_from[0], odo.y - self._hop_from[1])
        if gone >= self._hop_m:
            drive.stop(odo)
            self.state = "idle"
            self.log = f"우회 {gone:.2f}m. 경로 재연결"
            return "ok"
        if scan:
            half = math.radians(25)
            if sector_min_range(scan, 0.0, half) < 0.22:
                drive.stop(odo)
                self.state = "idle"
                self.log = f"옆도 막힘. 여기까지 우회 {gone:.2f}m"
                return "ok"
        if now - self._t0 >= RECOVER_HOP_S:
            drive.stop(odo)
            self.state = "idle"
            self.log = f"옆 이동 시간 초과 {gone:.2f}m. 경로 재연결"
            return "ok"
        drive.set_speeds(NAV_SPEED, NAV_SPEED, odo)
        return None
