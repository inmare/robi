"""바닥 장애물·라이다 걸림 회복.

스캔 평면 아래 물건은 맵에 안 그려진다. DWA/TEB도 스캔이 끊기면 못 쓴다.
그래서 Bug 식으로 후진 → 좌우 여유 → 옆 이동 후, 부딪힌 자리를
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
    SPIN_SPEED,
    VIRTUAL_BLOCK_M,
)
from robot.planner import plan


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
    """90° 옆 + 실제 회전하는 앞옆(약 55°). 옆만 보면 앞옆 벽을 놓친다."""
    half = math.radians(RECOVER_SECTOR_DEG)
    side_c = sector_min_range(points, side * math.pi / 2, half)
    front_c = sector_min_range(
        points, side * math.radians(55), math.radians(38)
    )
    return min(side_c, front_c)


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


def hit_corridor(x, y, yaw, ahead_m=HIT_AHEAD_M, depth_m=HIT_DEPTH_M, radius=VIRTUAL_BLOCK_M):
    """실패한 진행 방향 앞을 여러 원으로 막아 같은 통로로 안 돌아가게 한다."""
    n = 4
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


def choose_detour(scan, x, y, yaw, rest, clear_m=RECOVER_CLEAR_M, disks=None):
    """남은 경로에 가깝되, 기억한 장애물 쪽으로는 안 간다."""
    min_go = min(0.32, clear_m)
    rest = list(rest or [])
    left = side_clearance(scan, 1)
    right = side_clearance(scan, -1)
    best_side = 0
    best_score = None
    for side, cl in ((1, left), (-1, right)):
        if cl < min_go:
            continue
        hx, hy, tyaw = hop_pose(x, y, yaw, side, RECOVER_TURN_RAD, RECOVER_SIDE_M)
        look = (
            hx + 0.50 * math.cos(tyaw),
            hy + 0.50 * math.sin(tyaw),
        )
        d_disk = dist_to_disks(look, disks)
        d_land = dist_to_disks((hx, hy), disks)
        if d_land < 0.08:
            continue
        if rest:
            d_path = dist_to_polyline((hx, hy), rest)
            want = math.atan2(rest[0][1] - hy, rest[0][0] - hx)
            turn_after = abs(wrap_angle(want - tyaw))
        else:
            d_path = 0.0
            turn_after = 0.0
        score = (
            d_path
            + 0.30 * turn_after
            - 0.06 * min(cl, 1.2)
            - 0.85 * min(max(d_disk, -0.2), 1.6)
        )
        if best_score is None or score < best_score:
            best_score = score
            best_side = side
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


def skip_blocked(points, disks, pad=0.18):
    """기억한 장애물 안·바로 앞 점은 버린다. 같은 방향으로 다시 붙지 않게."""
    if not points:
        return []
    if not disks:
        return list(points)
    kept = [p for p in points if not point_in_disks(p, disks, pad=pad)]
    if kept:
        return kept
    return []


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
        if point_in_disks(nxt, extra_disks):
            continue
        cur = path[-1]
        if not seg_hits_disks(cur, nxt, extra_disks, pad=0.18):
            path.append(nxt)
            continue
        chunk = plan(grid, cur, nxt, extra_disks=extra_disks)
        if len(chunk) >= 2:
            path.extend(chunk[1:])
    if len(path) < 2:
        return []
    return thin_path(path)


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

    def start(self, odo, reason, target=None, rest=None):
        self.state = "reverse"
        self.reason = reason
        self.tries += 1
        self.side = 0
        self._t0 = time.monotonic()
        self._stuck = (odo.x, odo.y, odo.yaw)
        self._target = target
        self._rest = list(rest or [])
        if target is not None and not self._rest:
            self._rest = [target]
        self.disks.extend(hit_corridor(odo.x, odo.y, odo.yaw))
        self.log = f"후진 {RECOVER_BACK_M:.2f}m ({reason})"

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
        if self.state == "turn":
            return self._turn(drive, odo, now)
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
            rest = skip_blocked(self._rest, self.disks)
            side, left, right = choose_detour(
                scan, odo.x, odo.y, odo.yaw, rest, disks=self.disks
            )
            if side == 0:
                return self._fail(
                    drive,
                    odo,
                    f"좌우 막힘 L={left:.2f} R={right:.2f}",
                )
            self.side = side
            name = "왼쪽" if side > 0 else "오른쪽"
            self._target_yaw = wrap_angle(odo.yaw + side * RECOVER_TURN_RAD)
            self.state = "turn"
            self._t0 = now
            self.log = (
                f"{name} 우회(경로쪽)  L={left:.2f}m R={right:.2f}m  "
                f"목표각 {math.degrees(self._target_yaw):.0f}°"
            )
            return None
        if now - self._t0 >= RECOVER_WAIT_S:
            return self._fail(drive, odo, "라이다가 다시 안 돕니다")
        return None

    def _turn(self, drive, odo, now):
        err = wrap_angle(self._target_yaw - odo.yaw)
        if abs(err) < 0.12:
            drive.stop(odo)
            self.state = "hop"
            self._t0 = now
            self._hop_from = (odo.x, odo.y)
            self.log = "회전 끝. 옆으로 이동"
            return None
        if now - self._t0 >= RECOVER_TURN_S:
            self.state = "hop"
            self._t0 = now
            self._hop_from = (odo.x, odo.y)
            self.log = "회전 시간 초과. 옆으로 이동"
            return None
        spin = SPIN_SPEED
        if err > 0:
            drive.set_speeds(-spin, spin, odo)
        else:
            drive.set_speeds(spin, -spin, odo)
        return None

    def _hop(self, drive, odo, scan, now):
        gone = math.hypot(odo.x - self._hop_from[0], odo.y - self._hop_from[1])
        if gone >= RECOVER_SIDE_M:
            drive.stop(odo)
            self.state = "idle"
            self.log = f"우회 {gone:.2f}m. 경로 재연결"
            return "ok"
        if scan:
            half = math.radians(25)
            if sector_min_range(scan, 0.0, half) < 0.28:
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
