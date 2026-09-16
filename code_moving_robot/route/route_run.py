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
from robot.localize import match_heading, wrap_angle
from robot.odometry import Odometry
from robot.pins import (
    ARRIVE_M,
    FRONT_STOP_DEG,
    FRONT_STOP_M,
    LIDAR_STALL_S,
    RECORD_DT,
    RECORD_MIN_M,
    RECORD_MIN_YAW,
    RECOVER_MAX,
    STALL_MOVE_M,
    STALL_S,
    STALL_YAW,
    TELEOP_SPEED,
    TRACK_M,
)
from robot.planner import plan
from robot.recover import Recoverer, skip_blocked, splice_path, point_in_disks, seg_hits_disks
from robot.slam import Slam

OUT_PGM = ROOT / "maps" / "last_map.pgm"
OUT_BMP = ROOT / "maps" / "last_map.bmp"
OUT_TXT = ROOT / "maps" / "last_map.txt"
OUT_JSON = ROOT / "maps" / "last_route.json"

USE_COLOR = sys.stdout.isatty()


def _paint(code, text):
    if not USE_COLOR:
        return text
    return f"\033[{code}m{text}\033[0m"


def c_ok(text):
    return _paint("32", text)


def c_info(text):
    return _paint("36", text)


def c_warn(text):
    return _paint("33", text)


def c_err(text):
    return _paint("31;1", text)


def c_auto(text):
    return _paint("35;1", text)


def c_dim(text):
    return _paint("2", text)


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


def _xy_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def remaining_targets(odo, start, waypoints, goal):
    """찍어 둔 점을 앞에서부터 따라간다. 이미 지난 점은 건너뛴다."""
    here = (odo.x, odo.y)
    near_start = _xy_dist(here, start) <= 0.45
    near_goal = goal is not None and _xy_dist(here, goal) <= 0.45
    if near_goal and not near_start:
        if _xy_dist(here, start) > ARRIVE_M:
            return [start]
        return []

    seq = list(waypoints)
    if goal is not None:
        seq.append(goal)
    seq = [p for p in seq if _xy_dist(here, p) > ARRIVE_M]
    if not seq:
        return []
    nearest_i = min(range(len(seq)), key=lambda i: _xy_dist(here, seq[i]))
    if nearest_i > 0:
        seq = seq[nearest_i:]
    return seq


def build_path(grid, odo, targets, extra_disks=None):
    """기록점은 이미 지나온 길이므로 짧은 구간은 그대로 잇는다.

    맵의 # 조각·미지 칸 때문에 A*가 전부 실패하던 것을 막는다.
    기억한 바닥 장애물을 지나는 짧은 직선은 잇지 않는다.
    """
    if not targets:
        return []
    pts = skip_blocked([(t[0], t[1]) for t in targets], extra_disks)
    if not pts:
        return []
    path = [(odo.x, odo.y)]
    hop_m = 1.6
    for nxt in pts:
        nxt = (float(nxt[0]), float(nxt[1]))
        cur = path[-1]
        dist = _xy_dist(cur, nxt)
        hits = extra_disks and (
            point_in_disks(nxt, extra_disks) or seg_hits_disks(cur, nxt, extra_disks)
        )
        if dist < hop_m and not hits:
            path.append(nxt)
            continue
        if hits or dist >= hop_m:
            chunk = plan(grid, cur, nxt, extra_disks=extra_disks)
            if len(chunk) >= 2:
                path.extend(chunk[1:])
                continue
            if hits:
                continue
        path.append(nxt)
    return thin_path(path)


def pose_tuple(odo):
    return [round(odo.x, 3), round(odo.y, 3), round(odo.yaw, 3)]


def last_path_pose(start, waypoints, goal):
    if goal is not None:
        return goal
    if waypoints:
        return waypoints[-1]
    return start


def maybe_auto_record(odo, start, waypoints, now, last_t):
    if now - last_t < RECORD_DT:
        return waypoints, last_t, False
    last = last_path_pose(start, waypoints, None)
    dist = math.hypot(odo.x - last[0], odo.y - last[1])
    last_yaw = last[2] if len(last) > 2 else 0.0
    dyaw = abs(wrap_angle(odo.yaw - last_yaw))
    if dist < RECORD_MIN_M and dyaw < RECORD_MIN_YAW:
        return waypoints, last_t, False
    if len(waypoints) >= 80:
        return waypoints, now, False
    waypoints.append(pose_tuple(odo))
    return waypoints, now, True


def seal_goal(odo, waypoints, goal):
    """기록 종료 때 지금 자리를 도착으로. 마지막 경유와 같으면 합친다."""
    if goal is not None:
        return waypoints, goal
    p = pose_tuple(odo)
    if waypoints:
        last = waypoints[-1]
        if math.hypot(p[0] - last[0], p[1] - last[1]) <= ARRIVE_M:
            return waypoints[:-1], last
    return waypoints, p


def save_session(grid, odo, start, waypoints, goal, ascii_map=None):
    grid.save_pgm(OUT_PGM)
    grid.save_bmp(OUT_BMP)
    if ascii_map is None:
        ascii_map = grid.render_ascii(
            pose=(odo.x, odo.y),
            marks=mark_list(start, waypoints, goal),
        )
    OUT_TXT.parent.mkdir(parents=True, exist_ok=True)
    OUT_TXT.write_text(ascii_map + "\n", encoding="utf-8")
    data = {
        "start": start,
        "waypoints": waypoints,
        "goal": goal,
        "pose": pose_tuple(odo),
        "track_m": TRACK_M,
        "grid": {"size_m": grid.size_m, "resolution": grid.resolution},
    }
    OUT_JSON.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return OUT_PGM, OUT_JSON


def print_save_hint():
    pgm = OUT_PGM.resolve()
    bmp = OUT_BMP.resolve()
    txt = OUT_TXT.resolve()
    print(c_ok(f"저장 {pgm}"))
    print(c_info(f"그림 {bmp}  (윈도우에서 열림)"))
    print(c_dim(f"글자 {txt}"))
    print(c_dim(f"PC에서: scp USER@파이IP:{bmp} ."))


def load_session():
    if not OUT_JSON.exists() or not OUT_PGM.exists():
        return None
    try:
        data = json.loads(OUT_JSON.read_text(encoding="utf-8"))
        meta = data.get("grid") or {}
        grid = OccupancyGrid.from_pgm(
            OUT_PGM,
            size_m=float(meta.get("size_m", 16.0)),
            resolution=float(meta.get("resolution", 0.05)),
        )
        return data, grid
    except (OSError, ValueError, json.JSONDecodeError, TypeError) as exc:
        print(c_err(f"이전 지도 읽기 실패: {exc}"))
        return None


def collect_scan(lidar, n=4, timeout_s=8.0):
    best = None
    got = 0
    t0 = time.monotonic()
    while got < n and time.monotonic() - t0 < timeout_s:
        points = lidar.read()
        if not points:
            continue
        got += 1
        if best is None or len(points) > len(best):
            best = points
    return best


def restore_heading(grid, odo, lidar, data):
    points = collect_scan(lidar)
    if not points:
        print(c_warn("방향 맞춤용 스캔이 없습니다"))
        return False
    start = data.get("start") or [0.0, 0.0, 0.0]
    spots = [("시작점", float(start[0]), float(start[1]))]
    pose = data.get("pose")
    if pose is not None and (
        abs(float(pose[0]) - float(start[0])) > 0.2
        or abs(float(pose[1]) - float(start[1])) > 0.2
    ):
        spots.append(("마지막 위치", float(pose[0]), float(pose[1])))
    best = None
    best_name = None
    for name, x, y in spots:
        found = match_heading(grid, x, y, points)
        if found is None:
            continue
        if best is None or found.score > best.score:
            best = found
            best_name = name
    if best is None:
        print(c_err("이전 지도와 지금 스캔이 안 맞습니다. 빈 지도로 다시 그립니다"))
        return False
    if best.ambiguous:
        print(c_warn("방향이 여러 개로 맞습니다. 지도를 다시 그린 뒤 쓰세요"))
        return False
    odo.set_pose(best.x, best.y, best.yaw)
    extra = " (방향이 비슷해서 애매할 수 있음)" if best.ambiguous else ""
    print(
        c_ok(
            f"방향 맞춤 {best_name} x={best.x:.2f} y={best.y:.2f} "
            f"yaw={math.degrees(best.yaw):.0f}° score={best.score:.2f}{extra}"
        )
    )
    return True


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
        "wasd 이동  스페이스 정지  +/- 속도  k 경로기록 on/off\n"
        "수동: 1 시작  2 경유  3 도착  | 기록은 1초마다, 도착에서 k\n"
        "m 지도  r 재생  t 수동  l 방향  p 좌표  S 저장  q 종료\n"
        "자동: 자홍=위치  노랑=정체/회복(후진·좌우·우회)"
    )


def peek_saved_route():
    if not OUT_JSON.exists() or not OUT_PGM.exists():
        return False
    try:
        data = json.loads(OUT_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError, TypeError):
        return False
    if data.get("goal") is not None:
        return True
    wps = data.get("waypoints") or []
    return len(wps) > 0


def choose_start_mode(has_route):
    """기록/재생을 시작 전에 고른다. wasd 옆 e 오입을 피하려고 기록은 k."""
    if not has_route:
        print(c_info("저장 경로 없음. 기록 모드. 도착에서 k"))
        return "record"
    print(c_ok("저장한 경로가 있습니다."))
    print(c_info("  1  재생 모드  (기록 꺼짐, r 로 따라가기)"))
    print(c_info("  2  기록 모드  (새로 찍기, 도착에서 k)"))
    raw = input(c_warn("선택 [1 재생]: ")).strip().lower()
    if raw in ("2", "k", "record", "기록"):
        return "record"
    return "replay"


def auto_progress_line(odo, follower):
    n = len(follower.points)
    i = min(follower.i + 1, n)
    cur = follower.current()
    if cur is None:
        return f"[auto] 점 {n}/{n} 도착"
    dist = math.hypot(cur[0] - odo.x, cur[1] - odo.y)
    left = follower.remaining_m(odo)
    err_deg = math.degrees(follower.heading_err(odo))
    return (
        f"[auto] 점 {i}/{n}  "
        f"지금 ({odo.x:.2f},{odo.y:.2f})  "
        f"다음 ({cur[0]:.2f},{cur[1]:.2f})  "
        f"여기까지 {dist:.2f}m  남은 {left:.2f}m  "
        f"앞각 {err_deg:+.0f}°"
    )


def main():
    print(c_info(help_text()))
    print(c_dim(f"원점=시작. TRACK_M={TRACK_M:.3f}m (바퀴 중심 사이, robot/pins.py)"))
    start_mode = choose_start_mode(peek_saved_route())
    input(c_warn("준비되면 Enter. 모터 6V는 그 다음에 켜세요..."))

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
        loaded = load_session()
        speed = TELEOP_SPEED
        mode = "teleop"
        follower = None
        last_status = 0.0
        last_points = None
        slam = None
        recording = start_mode == "record"
        last_record_t = time.monotonic()
        last_move_xy = (0.0, 0.0)
        last_move_yaw = 0.0
        last_move_t = time.monotonic()
        last_follow_i = -1
        last_scan_t = None
        recover = Recoverer()

        drive.enable()
        lidar.start()
        if loaded is not None:
            data, loaded_grid = loaded
            if restore_heading(loaded_grid, odo, lidar, data):
                grid = loaded_grid
                start = data.get("start") or [0.0, 0.0, 0.0]
                waypoints = list(data.get("waypoints") or [])
                goal = data.get("goal")
                if goal is not None or waypoints:
                    if start_mode == "replay":
                        recording = False
                        print(c_ok("재생 모드. 기록 꺼짐. r 이면 따라갑니다"))
                    else:
                        waypoints = []
                        goal = None
                        recording = True
                        print(c_info("기록 모드. 이전 경로는 비웠습니다. 지도는 유지. 도착에서 k"))
                else:
                    if start_mode == "replay":
                        recording = False
                        print(c_info("저장 지도만 있습니다. 재생할 점이 없어 기록은 꺼둠. k 로 찍기"))
                    else:
                        recording = True
                        print(c_info("저장 지도를 썼습니다. 주행하면 경로가 자동으로 찍힙니다"))
            else:
                grid = OccupancyGrid(size_m=16.0, resolution=0.05)
                if start_mode == "replay":
                    recording = True
                    print(c_warn("이전 지도와 안 맞아 빈 지도입니다. 기록 모드로 바꿉니다"))
        else:
            grid = OccupancyGrid(size_m=16.0, resolution=0.05)
            if start_mode == "replay":
                recording = True
                print(c_warn("저장 파일을 못 읽어 기록 모드로 시작합니다"))
        slam = Slam(grid)
        slam.seed(odo)
        if recording:
            print(c_info("지도+조작 시작. 주행 중 1초마다 경로 기록, 도착에서 k"))
        else:
            print(c_info("지도+조작 시작. 저장 경로 재생. r"))
        while True:
            points = lidar.read()
            now = time.monotonic()
            if points:
                last_points = points
                last_scan_t = now
                slam.process(points, odo)

            if mode == "teleop" and recording and goal is None:
                waypoints, last_record_t, added = maybe_auto_record(
                    odo, start, waypoints, now, last_record_t
                )
                if added:
                    print(c_info(f"경로 {len(waypoints)} {waypoints[-1]}"))

            if mode == "auto":
                moved_xy = math.hypot(odo.x - last_move_xy[0], odo.y - last_move_xy[1])
                moved_yaw = abs(wrap_angle(odo.yaw - last_move_yaw))
                if moved_xy >= STALL_MOVE_M or moved_yaw >= STALL_YAW:
                    last_move_xy = (odo.x, odo.y)
                    last_move_yaw = odo.yaw
                    last_move_t = now
                scan_age = None if last_scan_t is None else now - last_scan_t
                fresh = scan_age is not None and scan_age < 0.7
                live_scan = last_points if fresh else None

                if recover.active():
                    before = recover.log
                    result = recover.step(drive, odo, live_scan, now)
                    if recover.log and recover.log != before:
                        if result == "ok":
                            print(c_ok(recover.log))
                        elif result == "fail":
                            print(c_err(recover.log))
                        else:
                            print(c_warn(recover.log))
                    if result == "ok":
                        rest = []
                        skipped = 0
                        if follower is not None:
                            raw = follower.remaining_points()
                            rest = skip_blocked(raw, recover.disks)
                            skipped = max(0, len(raw) - len(rest))
                        path = splice_path(grid, odo, rest, recover.disks)
                        if len(path) < 2:
                            drive.stop(odo)
                            mode = "teleop"
                            follower = None
                            recover.abort()
                            print(c_err("우회 후 붙을 점이 없습니다. 수동"))
                        else:
                            follower = Follower(path)
                            last_follow_i = -1
                            last_move_xy = (odo.x, odo.y)
                            last_move_yaw = odo.yaw
                            last_move_t = now
                            print(
                                c_auto(
                                    f"경로 재연결 {len(path)}점  "
                                    f"막힌점 {skipped}  기억 {len(recover.disks)}  "
                                    f"지금 ({odo.x:.2f},{odo.y:.2f})"
                                )
                            )
                    elif result == "fail":
                        drive.stop(odo)
                        mode = "teleop"
                        follower = None
                        print(c_err("회복 실패. 수동"))
                elif follower is None or follower.done():
                    drive.stop(odo)
                    mode = "teleop"
                    follower = None
                    recover.abort()
                    print(c_ok("경로 끝"))
                else:
                    stuck_s = now - last_move_t
                    trigger = None
                    if scan_age is not None and scan_age >= LIDAR_STALL_S:
                        trigger = f"라이다 {scan_age:.1f}s 정지. 바닥에 걸린 듯"
                    elif fresh and front_blocked(last_points):
                        trigger = "전방 장애물"
                    elif stuck_s >= STALL_S:
                        trigger = f"움직임 정체 {stuck_s:.1f}s"
                    if trigger:
                        drive.stop(odo)
                        if recover.tries >= RECOVER_MAX:
                            mode = "teleop"
                            follower = None
                            print(c_err(f"회복 {RECOVER_MAX}회 초과. 수동 ({trigger})"))
                        else:
                            recover.start(
                                odo,
                                trigger,
                                target=follower.current(),
                                rest=follower.remaining_points(),
                            )
                            print(
                                c_warn(
                                    f"회복 {recover.tries}/{RECOVER_MAX}: {recover.log}"
                                )
                            )
                    else:
                        if follower.i != last_follow_i:
                            last_follow_i = follower.i
                            n = len(follower.points)
                            cur = follower.current()
                            if cur is not None:
                                print(
                                    c_auto(
                                        f"점 {follower.i + 1}/{n} 추종  "
                                        f"다음 ({cur[0]:.2f},{cur[1]:.2f})  "
                                        f"남은 {follower.remaining_m(odo):.2f}m"
                                    )
                                )
                        follower.step(drive, odo)

            ch = keys.read(0.0 if mode == "auto" else 0.02)
            if ch is None:
                now = time.monotonic()
                if now - last_status >= 2.0:
                    last_status = now
                    if mode == "auto" and recover.active():
                        print(
                            c_warn(
                                f"[회복 {recover.state}] {recover.reason}  "
                                f"x={odo.x:.2f} y={odo.y:.2f}"
                            )
                        )
                    elif mode == "auto" and follower is not None and not follower.done():
                        print(c_auto(auto_progress_line(odo, follower)))
                    else:
                        rec = c_ok("ON") if recording else c_dim("off")
                        print(
                            f"[{c_info(mode)}] rec={rec} "
                            f"x={odo.x:.2f} y={odo.y:.2f} "
                            f"yaw={math.degrees(odo.yaw):.0f} "
                            f"wp={len(waypoints)} goal={'O' if goal else '-'} "
                            f"slam={'-' if slam.last_score is None else f'{slam.last_score:.2f}'}"
                        )
                continue

            if ch in ("\x03", "q", "Q"):
                break
            if ch == "t":
                mode = "teleop"
                follower = None
                recover.abort()
                drive.stop(odo)
                print(c_info("수동"))
                continue
            if mode == "auto" and ch not in ("m", "p"):
                mode = "teleop"
                follower = None
                recover.abort()
                drive.stop(odo)
                print(c_warn("키 입력 → 수동"))

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
                speed = min(0.92, speed + 0.04)
                print(c_dim(f"속도 {speed:.2f}"))
            elif ch == "-":
                speed = max(0.42, speed - 0.04)
                print(c_dim(f"속도 {speed:.2f}"))
            elif ch in ("k", "K"):
                if recording:
                    waypoints, goal = seal_goal(odo, waypoints, goal)
                    recording = False
                    print(
                        c_ok(
                            f"경로 기록 끝. 도착 {goal} 점 {len(waypoints)}개. r 로 재생"
                        )
                    )
                else:
                    recording = True
                    goal = None
                    last_record_t = time.monotonic()
                    print(c_info("경로 기록 시작. 도착 칸에서 다시 k"))
            elif ch in ("e", "E"):
                print(c_dim("기록은 k 입니다. e는 wasd 옆이라 뺐습니다. 재생은 r"))
            elif ch == "1":
                start = pose_tuple(odo)
                print(c_ok(f"시작 {start}"))
            elif ch == "2":
                waypoints.append(pose_tuple(odo))
                print(c_info(f"경유 {len(waypoints)} {waypoints[-1]}"))
            elif ch == "3":
                waypoints, goal = seal_goal(odo, waypoints, goal)
                recording = False
                print(c_ok(f"도착 {goal}"))
            elif ch == "p":
                print(
                    c_dim(
                        f"pose {pose_tuple(odo)} start {start} "
                        f"wp {waypoints} goal {goal}"
                    )
                )
            elif ch == "l":
                scan = last_points if last_points else collect_scan(lidar, n=2)
                if not scan:
                    print(c_warn("스캔 없음"))
                    continue
                found = match_heading(grid, odo.x, odo.y, scan)
                if found is None:
                    print(c_err("방향 맞춤 실패. 지도를 더 그리거나 시작 칸에 두세요"))
                    continue
                odo.set_pose(found.x, found.y, found.yaw)
                slam.seed(odo)
                extra = " (애매)" if found.ambiguous else ""
                print(
                    c_ok(
                        f"방향 맞춤 x={found.x:.2f} y={found.y:.2f} "
                        f"yaw={math.degrees(found.yaw):.0f}° "
                        f"score={found.score:.2f}{extra}"
                    )
                )
            elif ch == "m":
                ascii_map = grid.render_ascii(
                    pose=(odo.x, odo.y),
                    marks=mark_list(start, waypoints, goal),
                )
                print(ascii_map)
                save_session(
                    grid, odo, start, waypoints, goal, ascii_map=ascii_map
                )
                print_save_hint()
            elif ch == "S":
                save_session(grid, odo, start, waypoints, goal)
                print_save_hint()
            elif ch == "r":
                if recording:
                    waypoints, goal = seal_goal(odo, waypoints, goal)
                    recording = False
                    print(c_ok(f"기록 종료. 도착 {goal}"))
                targets = remaining_targets(odo, start, waypoints, goal)
                if not targets:
                    print(c_warn("이미 찍은 점에 다 와 있습니다"))
                    continue
                bits = []
                for p in targets:
                    dist = math.hypot(odo.x - p[0], odo.y - p[1])
                    bits.append(f"({p[0]:.2f},{p[1]:.2f}) {dist:.2f}m")
                print(c_info("다음 목표 " + " → ".join(bits)))
                path = build_path(grid, odo, targets, extra_disks=recover.disks)
                if len(path) < 2:
                    print(c_err("따라갈 점이 없습니다"))
                    continue
                follower = Follower(path)
                mode = "auto"
                recover.abort()
                recover.tries = 0
                last_follow_i = -1
                last_move_xy = (odo.x, odo.y)
                last_move_yaw = odo.yaw
                last_move_t = time.monotonic()
                print(
                    c_auto(
                        f"경로 재생 {len(path)}점. "
                        f"시작 ({odo.x:.2f},{odo.y:.2f}) → "
                        f"끝 ({path[-1][0]:.2f},{path[-1][1]:.2f})  "
                        f"아무 키면 수동"
                    )
                )
            elif ch in ("\n", "\r"):
                pass
            else:
                pass
    except KeyboardInterrupt:
        print(c_warn("Ctrl+C"))
    finally:
        if drive is not None:
            drive.disable()
        if lidar is not None:
            lidar.close()
        keys.close()
        if grid is not None and odo is not None:
            save_session(grid, odo, start, waypoints, goal)
            print(c_dim("라이다·모터 OFF. 지도 저장"))
            print_save_hint()


if __name__ == "__main__":
    main()
