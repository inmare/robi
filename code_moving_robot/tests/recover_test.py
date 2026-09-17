"""회복·우회 순수 함수. gpio 없이 PC에서 돈다."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.grid import OccupancyGrid
from robot.planner import plan
from robot.recover import (
    HallWatch,
    choose_detour,
    hit_corridor,
    pick_side,
    rejoin_path,
    skip_blocked,
    skip_near,
    splice_path,
    wheel_skew_side,
)


class FakeOdo:
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.left_count = 0
        self.right_count = 0
        self.left_sign = 0
        self.right_sign = 0


def test_pick_right_when_left_blocked():
    points = [(math.pi / 2, 0.2), (-math.pi / 2, 2.0)]
    side, left, right = pick_side(points)
    assert side == -1, (side, left, right)
    assert left < right


def test_pick_none_when_both_close():
    points = [(math.pi / 2, 0.15), (-math.pi / 2, 0.15)]
    side, _left, _right = pick_side(points)
    assert side == 0


def test_choose_route_side_when_both_sides_are_safe():
    """양쪽 모두 통과 가능하면 조금 좁아도 원래 경로 쪽을 고른다."""
    scan = [
        (math.pi / 2, 2.5),
        (math.radians(55), 2.4),
        (-math.pi / 2, 0.70),
        (-math.radians(55), 0.72),
    ]
    rest = [(1.0, -0.25), (2.0, -0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == -1, (side, left, right)


def test_choose_open_side_right():
    scan = [
        (math.pi / 2, 0.70),
        (math.radians(55), 0.72),
        (-math.pi / 2, 2.5),
        (-math.radians(55), 2.4),
    ]
    rest = [(1.0, 0.25), (2.0, 0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == 1, (side, left, right)


def test_choose_other_side_when_route_side_is_blocked():
    scan = [(math.pi / 2, 2.0), (-math.pi / 2, 0.15)]
    rest = [(1.0, -0.25), (2.0, -0.45)]
    side, left, right = choose_detour(scan, 0.0, 0.0, 0.0, rest)
    assert side == 1, (side, left, right)


def test_choose_avoids_obstacle_already_in_grid():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    cell = grid.world_to_cell(0.25, 0.35)
    grid.log_odds[cell[0]][cell[1]] = 2.0
    scan = [(math.pi / 2, 2.0), (-math.pi / 2, 2.0)]
    rest = [(1.0, 0.45), (2.0, 0.55)]
    side, left, right = choose_detour(
        scan, 0.0, 0.0, 0.0, rest, grid=grid
    )
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


def test_skip_blocked_keeps_last():
    disks = [(0.0, 0.0, 2.0)]
    pts = [(0.1, 0.0), (0.2, 0.0)]
    out = skip_blocked(pts, disks)
    assert len(out) == 1
    assert abs(out[0][0] - 0.2) < 1e-9


def test_plan_reaches_goal_in_disk():
    grid = OccupancyGrid(size_m=8.0, resolution=0.05)
    path = plan(grid, (0.0, 0.0), (0.6, 0.0), extra_disks=[(0.6, 0.0, 0.30)])
    assert len(path) >= 2
    assert path[-1][0] > 0.4


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


def test_wheel_skew_left_stuck():
    assert wheel_skew_side(0, 6, 1, 1) == "left"
    assert wheel_skew_side(6, 0, 1, 1) == "right"
    assert wheel_skew_side(5, 5, 1, 1) is None
    assert wheel_skew_side(0, 6, -1, 1) == "left"
    assert wheel_skew_side(0, 2, 1, 1) is None
    assert wheel_skew_side(0, 6, 1, 0) is None


def test_hall_watch_oscillating_free_wheel():
    odo = FakeOdo(0.0, 0.0)
    odo.left_sign = 1
    odo.right_sign = 1
    watch = HallWatch(window_s=0.80)
    assert watch.poll(odo, 0.00) is None
    odo.right_count = 4
    assert watch.poll(odo, 0.30) is None
    odo.right_count = 1
    odo.left_sign = -1
    odo.right_sign = 1
    assert watch.poll(odo, 0.81) == "left"


if __name__ == "__main__":
    test_pick_right_when_left_blocked()
    test_pick_none_when_both_close()
    test_choose_route_side_when_both_sides_are_safe()
    test_choose_open_side_right()
    test_choose_other_side_when_route_side_is_blocked()
    test_choose_avoids_obstacle_already_in_grid()
    test_skip_near_hit()
    test_skip_blocked_drops_corridor()
    test_skip_blocked_keeps_last()
    test_plan_goes_around_virtual_disk()
    test_plan_reaches_goal_in_disk()
    test_splice_skips_hit()
    test_splice_does_not_reenter_hit()
    test_rejoin_from_side_reaches_path()
    test_rejoin_goes_around_new_disk()
    test_wheel_skew_left_stuck()
    test_hall_watch_oscillating_free_wheel()
    print("recover_test ok")
