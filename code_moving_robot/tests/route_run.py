"""수동으로 지도를 그리며 시작·경유·도착을 찍고, 그 경로를 따라간다.

사용법: docs/route.md
"""

import json
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.drive import Drive
from robot.follow import Follower
from robot.grid import OccupancyGrid
from robot.lidar import Lidar
from robot.odometry import Odometry
from robot.pins import ARRIVE_M, FRONT_STOP_DEG, FRONT_STOP_M, TELEOP_SPEED, TRACK_M
from robot.planner import astar

OUT_PGM = ROOT / "maps" / "last_map.pgm"
OUT_JSON = ROOT / "maps" / "last_route.json"


class Keys:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def read(self, timeout):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        return sys.stdin.read(1)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def front_blocked(points):
    if not points:
        return False
    lim = FRONT_STOP_DEG * math.pi / 180.0
    for angle, rng in points:
        if abs(angle) < lim and 0.05 < rng < FRONT_STOP_M:
            return True
    return False


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


def remaining_targets(odo, start, waypoints, goal):
    seq = [start] + list(waypoints)
    if goal is not None:
        seq.append(goal)
    out = []
    for p in seq:
        if math.hypot(odo.x - p[0], odo.y - p[1]) > ARRIVE_M:
            out.append(p)
    return out


def build_path(grid, odo, targets):
    if not targets:
        return []
    path = []
    cur = (odo.x, odo.y)
    for tgt in targets:
        chunk = astar(grid, cur, (tgt[0], tgt[1]))
        if len(chunk) < 2:
            chunk = [cur, (tgt[0], tgt[1])]
        if path:
            chunk = chunk[1:]
        path.extend(chunk)
        cur = (tgt[0], tgt[1])
    return thin_path(path)


def pose_tuple(odo):
    return [round(odo.x, 3), round(odo.y, 3), round(odo.yaw, 3)]


def save_session(grid, odo, start, waypoints, goal):
    grid.save_pgm(OUT_PGM)
    data = {
        "start": start,
        "waypoints": waypoints,
        "goal": goal,
        "pose": pose_tuple(odo),
        "track_m": TRACK_M,
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return OUT_PGM, OUT_JSON


def mark_list(start, waypoints, goal):
    marks = []
    if start:
        marks.append((start[0], start[1], "S"))
    for i, wp in enumerate(waypoints, start=1):
        marks.append((wp[0], wp[1], str(min(i, 9))))
    if goal:
        marks.append((goal[0], goal[1], "G"))
    return marks


def help_text():
    return (
        "wasd 이동  스페이스 정지  +/- 속도\n"
        "1 시작점  2 경유점  3 도착점  m 지도  r 자율주행  t 수동\n"
        "p 좌표  s 저장  q 종료  | 모터 6V는 이 글 이후에 켜세요"
    )


def main():
    print(help_text())
    print(f"원점=시작. TRACK_M={TRACK_M:.3f}m (바퀴 중심 사이, robot/pins.py)")
    input("준비되면 Enter. 모터 6V는 그 다음에 켜세요...")

    keys = Keys()
    drive = None
    lidar = None
    odo = None
    grid = None
    start = [0.0, 0.0, 0.0]
    waypoints = []
    goal = None
    try:
        drive = Drive()
        odo = Odometry()
        lidar = Lidar()
        grid = OccupancyGrid(size_m=16.0, resolution=0.05)
        speed = TELEOP_SPEED
        mode = "teleop"
        follower = None
        last_status = 0.0
        last_points = None

        drive.enable()
        lidar.start()
        print("지도+조작 시작")
        while True:
            points = lidar.read()
            if points:
                last_points = points
                grid.add_scan(odo.x, odo.y, odo.yaw, points)

            if mode == "auto":
                if front_blocked(last_points):
                    drive.stop(odo)
                    mode = "teleop"
                    follower = None
                    print("전방 장애물. 수동으로 전환")
                elif follower is None or follower.done():
                    drive.stop(odo)
                    mode = "teleop"
                    follower = None
                    print("경로 끝")
                else:
                    follower.step(drive, odo)

            ch = keys.read(0.0 if mode == "auto" else 0.02)
            if ch is None:
                now = time.monotonic()
                if now - last_status >= 2.0:
                    last_status = now
                    print(
                        f"[{mode}] x={odo.x:.2f} y={odo.y:.2f} "
                        f"yaw={math.degrees(odo.yaw):.0f} "
                        f"wp={len(waypoints)} goal={'O' if goal else '-'}"
                    )
                continue

            if ch in ("\x03", "q", "Q"):
                break
            if ch == "t":
                mode = "teleop"
                follower = None
                drive.stop(odo)
                print("수동")
                continue
            if mode == "auto" and ch not in ("m", "p"):
                mode = "teleop"
                follower = None
                drive.stop(odo)
                print("키 입력 → 수동")

            if ch == "w":
                drive.set_speeds(speed, speed, odo)
            elif ch == "s":
                drive.set_speeds(-speed, -speed, odo)
            elif ch == "a":
                drive.set_speeds(-speed, speed, odo)
            elif ch == "d":
                drive.set_speeds(speed, -speed, odo)
            elif ch == " ":
                drive.stop(odo)
            elif ch == "+":
                speed = min(0.4, speed + 0.04)
                print(f"속도 {speed:.2f}")
            elif ch == "-":
                speed = max(0.12, speed - 0.04)
                print(f"속도 {speed:.2f}")
            elif ch == "1":
                start = pose_tuple(odo)
                print(f"시작 {start}")
            elif ch == "2":
                waypoints.append(pose_tuple(odo))
                print(f"경유 {len(waypoints)} {waypoints[-1]}")
            elif ch == "3":
                goal = pose_tuple(odo)
                print(f"도착 {goal}")
            elif ch == "p":
                print(
                    f"pose {pose_tuple(odo)} start {start} "
                    f"wp {waypoints} goal {goal}"
                )
            elif ch == "m":
                print(
                    grid.render_ascii(
                        pose=(odo.x, odo.y),
                        marks=mark_list(start, waypoints, goal),
                    )
                )
            elif ch == "s":
                pgm, js = save_session(grid, odo, start, waypoints, goal)
                print(f"저장 {pgm} {js}")
            elif ch == "r":
                targets = remaining_targets(odo, start, waypoints, goal)
                if not targets:
                    print("이미 찍은 점에 다 와 있습니다")
                    continue
                path = build_path(grid, odo, targets)
                if len(path) < 2:
                    print("경로 없음. 지도를 더 그리고 빈 칸으로 찍으세요")
                    continue
                follower = Follower(path)
                mode = "auto"
                print(f"자율주행 점 {len(path)}개. 아무 키나 누르면 수동")
            elif ch in ("\n", "\r"):
                pass
            else:
                pass
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        if drive is not None:
            drive.disable()
        if lidar is not None:
            lidar.close()
        keys.close()
        if grid is not None and odo is not None:
            save_session(grid, odo, start, waypoints, goal)
            print("라이다·모터 OFF. 지도 저장", OUT_PGM)


if __name__ == "__main__":
    main()
