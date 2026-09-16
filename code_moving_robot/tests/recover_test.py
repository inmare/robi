"""회복·우회 순수 함수. gpio 없이 PC에서 돈다."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.grid import OccupancyGrid
from robot.planner import plan
from robot.recover import choose_detour, pick_side, skip_near, splice_path


class FakeOdo:
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


def test_pick_right_when_left_blocked():
    points = [(math.pi / 2, 0.2), (-math.pi / 2, 2.0)]
    side, left, right = pick_side(points)
    assert side == -1, (side, left, right)
    assert left < right


def test_pick_none_when_both_close():
    points = [(math.pi / 2, 0.15), (-math.pi / 2, 0.15)]
    side, _left, _right = pick_side(points)
    assert side == 0


def test_choose_path_side_not_the_more_open_room():
    """왼쪽이 더 넓어도 남은 경로가 오른쪽이면 오른쪽으로."""
    scan = [
        (math.pi / 2, 2.5),
        (math.radians(55), 2.4),
        (-math.pi / 2, 0.70),
        (-math.radians(55), 0.72),
    ]
    rest = [(1.0, -0.25), (2.0, -0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == -1, (side, left, right)


def test_choose_path_side_left():
    scan = [
        (math.pi / 2, 0.70),
        (math.radians(55), 0.72),
        (-math.pi / 2, 2.5),
        (-math.radians(55), 2.4),
    ]
    rest = [(1.0, 0.25), (2.0, 0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == 1, (side, left, right)


def test_skip_near_hit():
    pts = [(0.0, 0.0), (0.1, 0.0), (1.0, 0.0)]
    out = skip_near(pts, (0.0, 0.0), skip_m=0.4)
    assert abs(out[0][0] - 1.0) < 1e-9


def test_plan_goes_around_virtual_disk():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    disks = [(0.6, 0.0, 0.24)]
    path = plan(grid, (0.0, 0.0), (1.4, 0.0), extra_disks=disks)
    assert len(path) >= 2
    mid = path[len(path) // 2]
    assert abs(mid[1]) > 0.15, mid
    for p in path:
        assert math.hypot(p[0] - 0.6, p[1] - 0.0) > 0.18, p


def test_splice_skips_hit():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    odo = FakeOdo(0.0, 0.4)
    rest = [(0.6, 0.0), (1.4, 0.0)]
    disks = [(0.6, 0.0, 0.24)]
    path = splice_path(grid, odo, rest, disks)
    assert len(path) >= 2
    assert path[-1][0] > 1.0


if __name__ == "__main__":
    test_pick_right_when_left_blocked()
    test_pick_none_when_both_close()
    test_choose_path_side_not_the_more_open_room()
    test_choose_path_side_left()
    test_skip_near_hit()
    test_plan_goes_around_virtual_disk()
    test_splice_skips_hit()
    print("recover_test ok")
