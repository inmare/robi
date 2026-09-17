"""경로 선 투영·왕복. gpio 없이 PC에서 돈다."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.path import (
    densify_route,
    lateral_offsets,
    project_polyline,
    remaining_from,
    round_trip_points,
    round_trip_remaining,
)


def test_round_trip_goes_out_and_back():
    out = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    trip = round_trip_points(out)
    assert trip[0][0] == 0.0
    assert trip[-1][0] == 0.0
    xs = [p[0] for p in trip]
    assert max(xs) >= 1.9
    assert xs.count(0.0) >= 2


def test_remaining_from_middle():
    trip = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0)]
    rest = remaining_from((1.0, 0.0), trip, arrive_m=0.15)
    assert rest[-1][0] == 0.0
    assert rest[0][0] >= 0.9


def test_remaining_from_off_path():
    start = [0.0, 0.0, 0.0]
    waypoints = [[1.0, 0.0, 0.0]]
    goal = [2.0, 0.0, 0.0]
    _out, _trip, rest = round_trip_remaining((1.0, 0.35), start, waypoints, goal)
    assert rest[-1][0] < 0.2
    assert any(p[0] > 1.7 for p in rest)


def test_project_sideways():
    pts = [(0.0, 0.0), (2.0, 0.0)]
    hit = project_polyline((1.0, 0.4), pts)
    assert abs(hit[0] - 1.0) < 0.05
    assert abs(hit[1]) < 0.05
    assert abs(hit[4] - 0.4) < 0.05


def test_lateral_offsets_include_center():
    pts = lateral_offsets(0.0, 0.0, 0.0, 0.5, n=1)
    assert (0.0, 0.0) in pts
    ys = sorted(p[1] for p in pts)
    assert ys[0] < -0.4 and ys[-1] > 0.4


def test_densify_keeps_ends():
    pts = densify_route([(0.0, 0.0, 0.0), (1.2, 0.0, 0.0)], step_m=0.4, max_n=40)
    assert abs(pts[0][0]) < 1e-9
    assert abs(pts[-1][0] - 1.2) < 1e-9
    assert len(pts) >= 3


if __name__ == "__main__":
    test_round_trip_goes_out_and_back()
    test_remaining_from_middle()
    test_remaining_from_off_path()
    test_project_sideways()
    test_lateral_offsets_include_center()
    test_densify_keeps_ends()
    print("path_test ok")
