"""회복·우회 순수 함수. gpio 없이 PC에서 돈다."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.grid import OccupancyGrid
from robot.planner import plan
from robot.recover import (
    choose_detour,
    hit_corridor,
    pick_side,
    rejoin_path,
    skip_blocked,
    skip_near,
    splice_path,
)


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


def test_choose_open_side_even_if_path_is_other_way():
    """경로가 오른쪽이어도 왼쪽이 비어 있으면 왼쪽으로."""
    scan = [
        (math.pi / 2, 2.5),
        (math.radians(55), 2.4),
        (-math.pi / 2, 0.70),
        (-math.radians(55), 0.72),
    ]
    rest = [(1.0, -0.25), (2.0, -0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == 1, (side, left, right)


def test_choose_open_side_right():
    scan = [
        (math.pi / 2, 0.70),
        (math.radians(55), 0.72),
        (-math.pi / 2, 2.5),
        (-math.radians(55), 2.4),
    ]
    rest = [(1.0, 0.25), (2.0, 0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == -1, (side, left, right)


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


def test_skip_blocked_drops_corridor():
    disks = hit_corridor(0.0, 0.0, 0.0)
    pts = [(0.2, 0.0), (0.5, 0.0), (1.6, 0.0)]
    out = skip_blocked(pts, disks)
    assert abs(out[0][0] - 1.6) < 1e-9, out


def test_plan_refuses_goal_in_disk():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    path = plan(grid, (0.0, 0.0), (0.6, 0.0), extra_disks=[(0.6, 0.0, 0.30)])
    assert path == []


def test_splice_does_not_reenter_hit():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    odo = FakeOdo(-0.2, 0.35)
    rest = [(0.2, 0.0), (0.5, 0.0), (0.8, 0.0), (1.6, 0.0)]
    disks = hit_corridor(0.0, 0.0, 0.0)
    path = splice_path(grid, odo, rest, disks)
    assert len(path) >= 2
    assert path[-1][0] > 1.2
    for p in path:
        for d in disks:
            gap = math.hypot(p[0] - d[0], p[1] - d[1]) - d[2]
            assert gap > -0.05, (p, d, gap)


def test_splice_skips_hit():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    odo = FakeOdo(0.0, 0.4)
    rest = [(0.6, 0.0), (1.4, 0.0)]
    disks = [(0.6, 0.0, 0.24)]
    path = splice_path(grid, odo, rest, disks)
    assert len(path) >= 2
    assert path[-1][0] > 1.0


def test_rejoin_from_side_reaches_path():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    odo = FakeOdo(0.4, 0.45)
    rest = [(0.4, 0.0), (0.8, 0.0), (1.6, 0.0)]

    path = rejoin_path(grid, odo, rest, extra_disks=None)
    assert len(path) >= 2
    assert path[-1][0] > 1.2


def test_rejoin_goes_around_new_disk():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    odo = FakeOdo(0.0, 0.0)
    rest = [(0.3, 0.0), (1.5, 0.0)]

    path = rejoin_path(grid, odo, rest, extra_disks=[(0.45, 0.0, 0.22)])
    assert len(path) >= 2
    assert path[-1][0] > 1.2
    for p in path:
        assert math.hypot(p[0] - 0.45, p[1] - 0.0) > 0.16, p


if __name__ == "__main__":
    test_pick_right_when_left_blocked()
    test_pick_none_when_both_close()
    test_choose_open_side_even_if_path_is_other_way()
    test_choose_open_side_right()
    test_skip_near_hit()
    test_skip_blocked_drops_corridor()
    test_plan_goes_around_virtual_disk()
    test_plan_refuses_goal_in_disk()
    test_splice_skips_hit()
    test_splice_does_not_reenter_hit()
    test_rejoin_from_side_reaches_path()
    test_rejoin_goes_around_new_disk()
    print("recover_test ok")
