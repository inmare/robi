"""슬롯을 고른 뒤의 주행. 왕복, 경로 옆 맞춤, 장애 수색 후 재연결."""

import math
import time

from robot.follow import Follower
from robot.grid import OccupancyGrid
from robot.localize import (
    MIN_OCC,
    match_along_route,
    match_heading,
    match_scan,
    wrap_angle,
)
from robot.path import (
    inbound_points,
    recorded_route,
    remaining_from,
    round_trip_points,
)
from robot.pins import (
    ARRIVE_M,
    FRONT_STOP_DEG,
    FRONT_STOP_M,
    LIDAR_STALL_S,
    RECORD_DT,
    RECORD_MIN_M,
    RECORD_MIN_YAW,
    RECOVER_MAX,
    REJOIN_OFF_M,
    ROUTE_LATERAL_M,
    STALL_MOVE_M,
    STALL_S,
    TELEOP_SPEED,
    TRACK_M,
)
from robot.recover import HallWatch, Recoverer, dist_to_polyline, rejoin_path, skip_blocked
from robot.tui import Keys, c_auto, c_dim, c_err, c_info, c_ok, c_warn, prompt


def front_blocked(points, grid=None, odo=None):
    """맵에 이미 있는 벽은 장애물이 아니다. 앞에 새로 나온 것만 회복한다."""
    if not points:
        return False
    lim = FRONT_STOP_DEG * math.pi / 180.0
    for angle, rng in points:
        if abs(angle) >= lim or rng <= 0.05 or rng >= FRONT_STOP_M:
            continue
        if grid is not None and odo is not None:
            wx = odo.x + rng * math.cos(odo.yaw + angle)
            wy = odo.y + rng * math.sin(odo.yaw + angle)
            cell = grid.world_to_cell(wx, wy)
            if cell is not None and grid.log_odds[cell[0]][cell[1]] > 0.5:
                continue
        return True
    return False


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
    if goal is not None:
        return waypoints, goal
    p = pose_tuple(odo)
    if waypoints:
        last = waypoints[-1]
        if math.hypot(p[0] - last[0], p[1] - last[1]) <= ARRIVE_M:
            return waypoints[:-1], last
    return waypoints, p


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


def mark_list(start, waypoints, goal):
    marks = []
    if start:
        marks.append((start[0], start[1], "S"))
    for i, wp in enumerate(waypoints, start=1):
        marks.append((wp[0], wp[1], str(min(i, 9))))
    if goal:
        marks.append((goal[0], goal[1], "G"))
    return marks


def apply_motion(drive, odo, motion, speed):
    if motion == "fwd":
        drive.set_speeds(speed, speed, odo)
    elif motion == "back":
        drive.set_speeds(-speed, -speed, odo)
    elif motion == "left":
        drive.set_speeds(-speed, speed, odo)
    elif motion == "right":
        drive.set_speeds(speed, -speed, odo)


def help_text():
    return (
        "wasd 이동  스페이스 정지  ↑↓ 속도  k 경로기록 on/off\n"
        "수동: 1 시작  2 경유  3 도착  | 기록은 1초마다, 도착에서 k\n"
        "g 목적지로  b 원래 자리  r 왕복  t 수동  l 경로위치  p 좌표  S 저장  q 메뉴\n"
        "중앙 TCP가 붙어 있으면 같은 g/b를 센터에서도 보낼 수 있음. 여기 키가 우선"
    )


def snap_to_route(grid, odo, slam, scan, start, waypoints, goal, prefer_here=True):
    if not scan:
        return None
    outbound = recorded_route(start, waypoints, goal)
    local = match_scan(
        grid, odo.x, odo.y, odo.yaw, scan, xy_m=0.40, yaw_rad=0.70
    )
    if local is None or local.score < 0.08:
        local = match_heading(grid, odo.x, odo.y, scan, xy_m=0.45, min_score=0.08)
    route = None
    if len(outbound) >= 1:
        route = match_along_route(
            grid, scan, outbound, lateral_m=ROUTE_LATERAL_M, min_score=0.07
        )
    best_name, best = None, None
    if prefer_here and local is not None:
        if route is None:
            best_name, best = "지금 자리", local
        else:
            jump = math.hypot(route.x - odo.x, route.y - odo.y)
            if jump > 0.65 and local.score >= route.score - 0.10:
                best_name, best = "지금 자리", local
            elif route.score > local.score + 0.14 and jump < 1.0:
                best_name, best = "경로", route
            else:
                best_name, best = "지금 자리", local
    else:
        picks = []
        if route is not None:
            picks.append(("경로", route))
        if local is not None:
            picks.append(("지금 자리", local))
        if not picks:
            return None
        best_name, best = max(picks, key=lambda item: item[1].score)
    if best is None:
        return None
    odo.set_pose(best.x, best.y, best.yaw)
    slam.seed(odo)
    return best_name, best


def restore_heading(grid, odo, lidar, data):
    print(c_dim("라이다 켜는 중. 1초 대기..."))
    time.sleep(1.0)
    points = collect_scan(lidar, n=6, timeout_s=12.0)
    if not points:
        print(c_warn("방향 맞춤용 스캔이 없습니다"))
        return False
    if grid.occ_n < MIN_OCC:
        print(c_err(f"저장 지도가 거의 비었습니다 (칸 {grid.occ_n}). 다시 그려야 합니다"))
        return False
    start = data.get("start") or [0.0, 0.0, 0.0]
    waypoints = list(data.get("waypoints") or [])
    goal = data.get("goal")
    outbound = recorded_route(start, waypoints, goal)
    picks = []
    if outbound:
        found = match_along_route(
            grid, points, outbound, lateral_m=ROUTE_LATERAL_M, min_score=0.08
        )
        if found is not None:
            picks.append(("경로", found))
    sx, sy = float(start[0]), float(start[1])
    syaw = float(start[2]) if len(start) > 2 else 0.0
    prior = match_scan(grid, sx, sy, syaw, points, xy_m=0.55, yaw_rad=0.85)
    if prior is not None and prior.score >= 0.10:
        picks.append(("시작점(저장각)", prior))
    head = match_heading(grid, sx, sy, points, xy_m=0.55, min_score=0.10)
    if head is not None:
        picks.append(("시작점", head))
    if not picks:
        print(c_err("이전 지도와 지금 스캔이 안 맞습니다"))
        return False
    best_name, best = max(picks, key=lambda item: item[1].score)
    odo.set_pose(best.x, best.y, best.yaw)
    extra = " (여러 각이 비슷. 틀리면 l)" if best.ambiguous else ""
    print(
        c_ok(
            f"방향 맞춤 {best_name} x={best.x:.2f} y={best.y:.2f} "
            f"yaw={math.degrees(best.yaw):.0f}° score={best.score:.2f}{extra}"
        )
    )
    return True


def route_progress(cur, start, waypoints, goal):
    seq = recorded_route(start, waypoints, goal)
    n = len(seq)
    if n == 0 or cur is None:
        return 0, 0
    i = min(range(n), key=lambda k: math.hypot(cur[0] - seq[k][0], cur[1] - seq[k][1]))
    return i + 1, n


def auto_progress_line(odo, follower, start, waypoints, goal, stuck_s=0.0, going_home=False, auto_leg=None):
    cur = follower.current()
    if auto_leg == "go":
        phase = "목적지로"
    elif auto_leg == "back":
        phase = "원래 자리로"
    else:
        phase = "복귀" if going_home else "왕복"
    if cur is None:
        return f"[auto] {phase} 도착"
    i, n = follower_route_progress(odo, start, waypoints, goal, auto_leg)
    dist = math.hypot(cur[0] - odo.x, cur[1] - odo.y)
    left = follower.remaining_m(odo)
    err_deg = math.degrees(follower.heading_err(odo))
    stall = f"  정체 {stuck_s:.1f}s" if stuck_s >= 1.0 else ""
    return (
        f"[auto] {phase} 점 {i}/{n}  "
        f"지금 ({odo.x:.2f},{odo.y:.2f})  "
        f"다음 ({cur[0]:.2f},{cur[1]:.2f})  "
        f"여기까지 {dist:.2f}m  남은 {left:.2f}m  "
        f"앞각 {err_deg:+.0f}°{stall}"
    )


def trip_going_home(odo, start, waypoints, goal, follower):
    outbound = recorded_route(start, waypoints, goal)
    if len(outbound) < 2 or follower is None or follower.done():
        return False
    goal_xy = outbound[-1]
    start_xy = outbound[0]
    cur = follower.current()
    if cur is None:
        return False
    d_goal = math.hypot(odo.x - goal_xy[0], odo.y - goal_xy[1])
    d_start = math.hypot(cur[0] - start_xy[0], cur[1] - start_xy[1])
    passed_goal = d_goal < 0.55
    rest = follower.remaining_m(odo)
    half = 0.0
    for j in range(len(outbound) - 1):
        a, b = outbound[j], outbound[j + 1]
        half += math.hypot(b[0] - a[0], b[1] - a[1])
    return passed_goal or (d_start < 0.8 and rest < half * 0.95)


LEG_LABEL = {
    "go": "목적지로 이동",
    "back": "원래 자리로",
    "round": "왕복",
}


def leg_line(leg, start, waypoints, goal):
    outbound = recorded_route(start, waypoints, goal)
    if leg == "back":
        return inbound_points(start, waypoints, goal)
    if leg == "round":
        return round_trip_points(outbound)
    return outbound


def rest_for_leg(leg, here, start, waypoints, goal, arrive_m=ARRIVE_M):
    line = leg_line(leg, start, waypoints, goal)
    return remaining_from(here, line, arrive_m=arrive_m)


def follower_route_progress(odo, start, waypoints, goal, leg):
    seq = leg_line(leg or "go", start, waypoints, goal)
    n = len(seq)
    if n == 0:
        return 0, 0
    cur = (odo.x, odo.y)
    i = min(range(n), key=lambda k: math.hypot(cur[0] - seq[k][0], cur[1] - seq[k][1]))
    return i + 1, n


def start_leg(leg, grid, odo, slam, lidar, start, waypoints, goal, recover, last_points):
    """한 구간을 붙인다. ('ok', follower) | ('already', None) | ('no_path', None)."""
    label = LEG_LABEL.get(leg, "이동")
    print(c_dim("경로 위치 맞추는 중..."))
    scan = collect_scan(lidar, n=4, timeout_s=6.0) or last_points
    hit = snap_to_route(grid, odo, slam, scan, start, waypoints, goal)
    if hit is None:
        print(c_warn(f"스캔 맞춤 실패. 지금 좌표로 {label}"))
    else:
        name, found = hit
        extra = " (애매)" if found.ambiguous else ""
        print(
            c_ok(
                f"{name} 맞춤 x={found.x:.2f} y={found.y:.2f} "
                f"yaw={math.degrees(found.yaw):.0f}° "
                f"score={found.score:.2f}{extra}"
            )
        )
    rest = rest_for_leg(leg, (odo.x, odo.y), start, waypoints, goal)
    if not rest:
        print(c_warn(f"이미 도착. {label} 할 나머지가 없습니다"))
        return "already", None
    off = dist_to_polyline((odo.x, odo.y), rest)
    if off >= REJOIN_OFF_M:
        print(c_info(f"경로에서 {off:.2f}m 옆. A*로 선에 붙입니다"))
    path = rejoin_path(grid, odo, rest, recover.disks, scan=None)
    if len(path) < 2:
        print(c_err("따라갈 점이 없습니다"))
        return "no_path", None
    i, n = follower_route_progress(odo, start, waypoints, goal, leg)
    print(
        c_auto(
            f"{label} {i}/{n}. "
            f"시작 ({odo.x:.2f},{odo.y:.2f}) → "
            f"끝 ({path[-1][0]:.2f},{path[-1][1]:.2f})  "
            f"아무 키면 수동"
        )
    )
    return "ok", Follower(path)


def start_round_trip(grid, odo, slam, lidar, start, waypoints, goal, recover, last_points):
    kind, follower = start_leg(
        "round", grid, odo, slam, lidar, start, waypoints, goal, recover, last_points
    )
    return follower if kind == "ok" else None


def save_now(store, slot_index, grid, odo, start, waypoints, goal, name, replace_map):
    if store is None or slot_index is None:
        return None
    info = store.save(
        slot_index,
        grid,
        odo,
        start,
        waypoints,
        goal,
        name,
        replace_map=replace_map,
    )
    print(c_ok(f"슬롯 {info.index} 저장 「{info.name}」 {info.saved_at}"))
    print(c_dim(str(info.directory.resolve())))
    return info


def run_drive(
    *,
    store,
    slot_index,
    slot_name,
    recording,
    map_writable,
    grid=None,
    data=None,
    center_host="",
    center_port=9000,
):
    """하드웨어를 열고 키로 기록/왕복한다. 끝나면 세션 dict."""
    print(c_info(help_text()))
    print(c_dim(f"원점=시작. TRACK_M={TRACK_M:.3f}m (바퀴 중심 사이, robot/pins.py)"))
    if slot_index is not None:
        mode_name = "기록" if recording else "재생"
        print(c_ok(f"슬롯 {slot_index} 「{slot_name}」 {mode_name}"))
    else:
        print(c_info("새 경로. 끝나면 슬롯에 저장합니다"))
    prompt(c_warn("준비되면 Enter. 모터 6V는 그 다음에 켜세요..."))

    from robot.drive import Drive
    from robot.lidar import Lidar
    from robot.odometry import Odometry
    from robot.slam import Slam

    keys = Keys()
    drive = None
    lidar = None
    odo = None
    start = [0.0, 0.0, 0.0]
    waypoints = []
    goal = None
    name = slot_name
    center = None
    dirty = False
    try:
        drive = Drive()
        odo = Odometry()
        lidar = Lidar()
        speed = TELEOP_SPEED
        motion = None
        mode = "teleop"
        follower = None
        last_status = 0.0
        last_points = None
        slam = None
        last_record_t = time.monotonic()
        last_progress_m = None
        last_move_t = time.monotonic()
        last_follow_i = -1
        last_scan_t = None
        recover = Recoverer()
        hall_watch = HallWatch()
        dirty = False

        drive.enable()
        lidar.start()
        if data is not None and grid is not None:
            start = data.get("start") or [0.0, 0.0, 0.0]
            waypoints = list(data.get("waypoints") or [])
            goal = data.get("goal")
            ok = restore_heading(grid, odo, lidar, data)
            if not ok:
                odo.set_pose(
                    float(start[0]),
                    float(start[1]),
                    float(start[2]) if len(start) > 2 else 0.0,
                )
                print(
                    c_warn(
                        "방향 맞춤 실패. 저장한 시작 좌표를 씁니다. "
                        "앞이 반대면 l. 지도는 버리지 않습니다"
                    )
                )
            if recording:
                waypoints = []
                goal = None
                map_writable = True
                print(c_info("기록 모드. 이전 경로는 비웠습니다. 지도는 유지. 도착에서 k"))
            else:
                map_writable = False
                print(c_ok("재생 모드. g 목적지, b 원래 자리, r 왕복"))
        else:
            grid = OccupancyGrid(size_m=16.0, resolution=0.05)
            recording = True
            map_writable = True
            print(c_info("빈 지도. 주행 중 1초마다 경로 기록, 도착에서 k"))

        slam = Slam(grid, update_map=map_writable)
        slam.seed(odo)

        auto_leg = None
        nav_src = None
        if center_host:
            from robot.center_client import CenterClient

            center = CenterClient(str(center_host), int(center_port or 9000))
            center.start()
            print(
                c_info(
                    f"중앙 {center_host}:{center_port} 연결 시도. "
                    "여기 키와 센터 명령을 같이 씁니다. 키 쪽이 우선"
                )
            )

        def current_phase():
            return {"go": "go", "back": "back", "round": "round"}.get(auto_leg, "idle")

        def phase_cmd(leg):
            return "robot.back" if leg == "back" else "robot.go"

        def emit_robot_ack(src, cmd):
            if center is None or not src:
                return
            center.emit(f"A {src['id']} ACK {cmd}")

        def emit_robot_fail(reason, src=None):
            src = src or nav_src
            if center is None or not src:
                return
            center.emit(
                f"F {src['id']} FAIL {src.get('cmd', 'robot.go')} reason={reason}"
            )

        def emit_robot_prog():
            if center is None or follower is None:
                return
            i, n = follower_route_progress(odo, start, waypoints, goal, auto_leg)
            src = nav_src or {"id": "0000", "cmd": phase_cmd(auto_leg)}
            center.emit(
                f"P {src['id']} PROG {src['cmd']} i={i} n={n} phase={current_phase()}"
            )

        def emit_robot_done(reason):
            nonlocal nav_src
            if center is None:
                nav_src = None
                return
            src = nav_src or {"id": "0000", "cmd": phase_cmd(auto_leg)}
            i, n = follower_route_progress(odo, start, waypoints, goal, auto_leg)
            center.emit(
                f"D {src['id']} DONE {src['cmd']} reason={reason} "
                f"i={i} n={n} phase=idle"
            )
            nav_src = None

        def emit_robot_status(msg_id):
            if center is None:
                return
            i, n = 0, 0
            if follower is not None:
                i, n = follower_route_progress(odo, start, waypoints, goal, auto_leg)
            busy = 1 if mode == "auto" else 0
            state = "NAV" if busy else "IDLE"
            center.emit(
                f"S {msg_id} STATUS state={state} phase={current_phase()} "
                f"i={i} n={n} busy={busy}"
            )

        def abort_nav(reason="halted"):
            nonlocal mode, follower, auto_leg, nav_src, motion
            if nav_src:
                if reason in {"arrived", "home", "already", "round_done"}:
                    emit_robot_done(reason)
                else:
                    emit_robot_fail(reason, nav_src)
                    nav_src = None
            auto_leg = None
            follower = None
            mode = "teleop"
            motion = None
            recover.abort()
            drive.stop(odo)

        def begin_auto(leg, src=None):
            nonlocal mode, follower, auto_leg, nav_src, recording, waypoints, goal, dirty
            nonlocal last_follow_i, last_progress_m, last_move_t
            if nav_src and (src is None or nav_src.get("id") != src.get("id")):
                emit_robot_fail("replaced", nav_src)
                nav_src = None
            if recording:
                waypoints, goal = seal_goal(odo, waypoints, goal)
                recording = False
                dirty = True
                print(c_ok(f"기록 종료. 도착 {goal}"))
            if not waypoints and goal is None:
                print(c_warn("기록된 경로가 없습니다"))
                if src:
                    emit_robot_ack(src, src["cmd"])
                    emit_robot_fail("no_route", src)
                return
            if src:
                emit_robot_ack(src, src["cmd"])
            kind, next_f = start_leg(
                leg,
                grid,
                odo,
                slam,
                lidar,
                start,
                waypoints,
                goal,
                recover,
                last_points,
            )
            src_use = src or {"id": "0000", "cmd": phase_cmd(leg)}
            if kind == "already":
                auto_leg = leg
                nav_src = src_use
                emit_robot_done("already")
                auto_leg = None
                return
            if kind != "ok" or next_f is None:
                emit_robot_fail("no_path", src_use)
                return
            follower = next_f
            auto_leg = leg
            nav_src = src_use
            mode = "auto"
            recover.abort()
            recover.tries = 0
            hall_watch.reset()
            last_follow_i = -1
            last_progress_m = None
            last_move_t = time.monotonic()
            emit_robot_prog()

        def apply_center_item(item):
            cmd = item.get("cmd") or ""
            if cmd in ("status", "robot.status"):
                emit_robot_status(item["id"])
                return
            if cmd in ("halt", "robot.halt"):
                emit_robot_ack(item, cmd)
                abort_nav("halted")
                if center is not None:
                    center.emit(f"D {item['id']} DONE {cmd} reason=stopped")
                print(c_info("중앙 정지 → 수동"))
                return
            if cmd in ("robot.go", "go"):
                begin_auto("go", item)
                return
            if cmd in ("robot.back", "back"):
                begin_auto("back", item)
                return
            emit_robot_ack(item, cmd)
            emit_robot_fail("unknown", item)

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
                    dirty = True
                    print(c_info(f"경로 {len(waypoints)} {waypoints[-1]}"))

            if center is not None:
                item = center.poll()
                if item:
                    apply_center_item(item)

            if mode == "auto":
                scan_age = None if last_scan_t is None else now - last_scan_t
                fresh = scan_age is not None and scan_age < 0.7
                live_scan = last_points if fresh else None

                if recover.active():
                    if live_scan:
                        recover.ingest_scan(odo, live_scan, grid)
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
                        rest = rest_for_leg(
                            auto_leg or "round",
                            (odo.x, odo.y),
                            start,
                            waypoints,
                            goal,
                        )
                        skipped = 0
                        path = rejoin_path(
                            grid, odo, rest, recover.disks, scan=None
                        )
                        if len(path) < 2:
                            path = rejoin_path(grid, odo, rest, extra_disks=[], scan=None)
                        if len(path) < 2:
                            abort_nav("no_path")
                            print(c_err("우회 후 붙을 점이 없습니다. 수동"))
                        else:
                            follower = Follower(path)
                            last_follow_i = -1
                            last_progress_m = None
                            last_move_t = now
                            print(
                                c_auto(
                                    f"경로 재연결 {len(path)}점  "
                                    f"기억 {len(recover.disks)}  "
                                    f"지금 ({odo.x:.2f},{odo.y:.2f})"
                                )
                            )
                    elif result == "fail":
                        abort_nav("recover")
                        print(c_err("회복 실패. 수동"))
                elif follower is None or follower.done():
                    reason = {
                        "go": "arrived",
                        "back": "home",
                        "round": "round_done",
                    }.get(auto_leg, "done")
                    label = {
                        "go": "목적지 도착",
                        "back": "원래 자리",
                        "round": "왕복 끝. 시작점",
                    }.get(auto_leg, "자동 끝")
                    drive.stop(odo)
                    emit_robot_done(reason)
                    mode = "teleop"
                    follower = None
                    auto_leg = None
                    recover.abort()
                    print(c_ok(label))
                else:
                    left = follower.remaining_m(odo)
                    heading = abs(follower.heading_err(odo))
                    skew = hall_watch.poll(odo, now)
                    if last_progress_m is None:
                        last_progress_m = left
                        last_move_t = now
                    elif skew is not None:
                        pass
                    elif heading > 0.40:
                        last_move_t = now
                    elif last_progress_m - left >= STALL_MOVE_M:
                        last_progress_m = left
                        last_move_t = now
                    stuck_s = now - last_move_t
                    trigger = None
                    stuck_wheel = None
                    if skew is not None:
                        stuck_wheel = skew
                        name = "왼쪽" if skew == "left" else "오른쪽"
                        trigger = f"{name} 바퀴 걸림"
                    elif scan_age is not None and scan_age >= LIDAR_STALL_S:
                        trigger = f"라이다 {scan_age:.1f}s 정지. 바닥에 걸린 듯"
                    elif last_points and (
                        scan_age is None or scan_age < LIDAR_STALL_S
                    ) and front_blocked(last_points, grid, odo):
                        trigger = "전방 장애물"
                    elif stuck_s >= STALL_S:
                        trigger = f"움직임 정체 {stuck_s:.1f}s"
                    if trigger:
                        drive.stop(odo)
                        if recover.tries >= RECOVER_MAX:
                            abort_nav("recover")
                            print(c_err(f"회복 {RECOVER_MAX}회 초과. 수동 ({trigger})"))
                        else:
                            rest = rest_for_leg(
                                auto_leg or "round",
                                (odo.x, odo.y),
                                start,
                                waypoints,
                                goal,
                            )
                            recover.start(
                                odo,
                                trigger,
                                target=follower.current(),
                                rest=rest or follower.remaining_points(),
                                grid=grid,
                                stuck_wheel=stuck_wheel,
                            )
                            hall_watch.reset()
                            print(
                                c_warn(
                                    f"회복 {recover.tries}/{RECOVER_MAX}: {recover.log}"
                                )
                            )
                    else:
                        follower.step(drive, odo, speed)
                        if follower.i != last_follow_i:
                            if (
                                last_follow_i >= 0
                                and follower.i > last_follow_i
                                and recover.tries
                            ):
                                recover.tries = 0
                                print(c_ok("점 통과. 회복 횟수 리셋"))
                            last_follow_i = follower.i
                            cur = follower.current()
                            if cur is not None:
                                i, n = follower_route_progress(
                                    odo, start, waypoints, goal, auto_leg
                                )
                                phase = LEG_LABEL.get(auto_leg, "이동")
                                print(
                                    c_auto(
                                        f"{phase} 점 {i}/{n} 추종  "
                                        f"다음 ({cur[0]:.2f},{cur[1]:.2f})  "
                                        f"남은 {follower.remaining_m(odo):.2f}m"
                                    )
                                )
                                emit_robot_prog()

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
                        print(
                            c_auto(
                                auto_progress_line(
                                    odo,
                                    follower,
                                    start,
                                    waypoints,
                                    goal,
                                    stuck_s=now - last_move_t,
                                    going_home=trip_going_home(
                                        odo, start, waypoints, goal, follower
                                    ),
                                    auto_leg=auto_leg,
                                )
                            )
                        )
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
                abort_nav("halted")
                print(c_info("수동"))
                continue
            if mode == "auto" and ch not in (
                "m",
                "p",
                "up",
                "down",
                "+",
                "-",
                "=",
                "r",
                "l",
                "g",
                "b",
            ):
                abort_nav("halted")
                print(c_warn("키 입력 → 수동"))

            if ch == "w":
                motion = "fwd"
                drive.set_speeds(speed, speed, odo)
            elif ch == "s":
                motion = "back"
                drive.set_speeds(-speed, -speed, odo)
            elif ch == "a":
                motion = "left"
                drive.set_speeds(-speed, speed, odo)
            elif ch == "d":
                motion = "right"
                drive.set_speeds(speed, -speed, odo)
            elif ch == " ":
                motion = None
                drive.stop(odo)
            elif ch in ("up", "+", "="):
                speed = min(0.95, speed + 0.08)
                apply_motion(drive, odo, motion, speed)
                print(c_info(f"속도 {speed:.2f}"))
            elif ch in ("down", "-", "_"):
                speed = max(0.35, speed - 0.08)
                apply_motion(drive, odo, motion, speed)
                print(c_info(f"속도 {speed:.2f}"))
            elif ch in ("k", "K"):
                if recording:
                    waypoints, goal = seal_goal(odo, waypoints, goal)
                    recording = False
                    dirty = True
                    print(
                        c_ok(
                            f"경로 기록 끝. 도착 {goal} 점 {len(waypoints)}개. r 로 왕복"
                        )
                    )
                else:
                    recording = True
                    map_writable = True
                    slam.update_map = True
                    goal = None
                    last_record_t = time.monotonic()
                    dirty = True
                    print(c_info("경로 기록 시작. 도착 칸에서 다시 k"))
            elif ch in ("e", "E"):
                print(c_dim("기록은 k 입니다. e는 wasd 옆이라 뺐습니다. 왕복은 r"))
            elif ch == "1":
                start = pose_tuple(odo)
                dirty = True
                print(c_ok(f"시작 {start}"))
            elif ch == "2":
                waypoints.append(pose_tuple(odo))
                dirty = True
                print(c_info(f"경유 {len(waypoints)} {waypoints[-1]}"))
            elif ch == "3":
                waypoints, goal = seal_goal(odo, waypoints, goal)
                recording = False
                dirty = True
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
                hit = snap_to_route(grid, odo, slam, scan, start, waypoints, goal)
                if hit is None:
                    print(c_err("방향 맞춤 실패. 지도를 더 그리거나 경로 위에 두세요"))
                    continue
                name_hit, found = hit
                extra = " (애매)" if found.ambiguous else ""
                print(
                    c_ok(
                        f"{name_hit} 맞춤 x={found.x:.2f} y={found.y:.2f} "
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
                if slot_index is not None:
                    save_now(
                        store,
                        slot_index,
                        grid,
                        odo,
                        start,
                        waypoints,
                        goal,
                        name,
                        replace_map=map_writable,
                    )
                    dirty = False
            elif ch == "S":
                if slot_index is None:
                    print(c_warn("아직 슬롯이 없습니다. q 로 나가서 저장하세요"))
                else:
                    save_now(
                        store,
                        slot_index,
                        grid,
                        odo,
                        start,
                        waypoints,
                        goal,
                        name,
                        replace_map=map_writable,
                    )
                    dirty = False
            elif ch in ("g", "G"):
                begin_auto("go")
            elif ch in ("b", "B"):
                begin_auto("back")
            elif ch == "r":
                begin_auto("round")
            elif ch in ("\n", "\r"):
                pass
    except KeyboardInterrupt:
        print(c_warn("Ctrl+C"))
    finally:
        if center is not None:
            center.close()
        if drive is not None:
            try:
                drive.stop(odo)
            except Exception:
                drive.disable()
        if lidar is not None:
            lidar.close()
        if odo is not None:
            odo.close()
        if drive is not None:
            drive.close()
        keys.close()
        print(c_dim("라이다·모터 OFF"))

    return {
        "grid": grid,
        "odo": odo,
        "start": start,
        "waypoints": waypoints,
        "goal": goal,
        "name": name,
        "dirty": dirty or recording,
        "map_writable": map_writable,
        "slot_index": slot_index,
        "has_route": bool(waypoints) or goal is not None,
    }
