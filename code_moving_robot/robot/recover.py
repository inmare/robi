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


def pick_side(points, prefer=0, clear_m=RECOVER_CLEAR_M):
    """+1 왼쪽, -1 오른쪽, 0 양쪽 막힘."""
    half = math.radians(RECOVER_SECTOR_DEG)
    left = sector_min_range(points, math.pi / 2, half)
    right = sector_min_range(points, -math.pi / 2, half)
    left_ok = left >= clear_m
    right_ok = right >= clear_m
    if left_ok and right_ok:
        if prefer > 0:
            return 1, left, right
        if prefer < 0:
            return -1, left, right
        return (1 if left >= right else -1), left, right
    if left_ok:
        return 1, left, right
    if right_ok:
        return -1, left, right
    tight = 0.35
    if max(left, right) >= tight:
        return (1 if left >= right else -1), left, right
    return 0, left, right


def skip_near(points, hit_xy, skip_m=0.40):
    rest = [p for p in points if math.hypot(p[0] - hit_xy[0], p[1] - hit_xy[1]) > skip_m]
    if rest:
        return rest
    if points:
        return [points[-1]]
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


def splice_path(grid, odo, rest, extra_disks):
    """우회한 자리에서 남은 점으로. 먼저 A*, 실패하면 직선 이음."""
    rest = list(rest)
    if not rest:
        return []
    here = (odo.x, odo.y)
    chunk = plan(grid, here, rest[0], extra_disks=extra_disks)
    if len(chunk) < 2:
        chunk = [here, rest[0]]
    return thin_path(chunk + rest[1:])


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

    def start(self, odo, reason, target=None):
        self.state = "reverse"
        self.reason = reason
        self.tries += 1
        self.side = 0
        self._t0 = time.monotonic()
        self._stuck = (odo.x, odo.y, odo.yaw)
        self._target = target
        hx = odo.x + HIT_AHEAD_M * math.cos(odo.yaw)
        hy = odo.y + HIT_AHEAD_M * math.sin(odo.yaw)
        self.disks.append((hx, hy, VIRTUAL_BLOCK_M))
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
            nxt = self._target
            prefer = prefer_side(odo.yaw, (odo.x, odo.y), nxt)
            side, left, right = pick_side(scan, prefer=prefer)
            if side == 0:
                return self._fail(
                    drive,
                    odo,
                    f"좌우 막힘 L={left:.2f} R={right:.2f}",
                )
            self.side = side
            name = "왼쪽" if side > 0 else "오른쪽"
            self._target_yaw = wrap_angle(self._stuck[2] + side * RECOVER_TURN_RAD)
            self.state = "turn"
            self._t0 = now
            self.log = (
                f"{name} 우회  L={left:.2f}m R={right:.2f}m  "
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
