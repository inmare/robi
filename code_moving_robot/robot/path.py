"""기록 경로의 선 투영, 왕복, 옆 이탈 샘플."""

import math


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _xy(p):
    return float(p[0]), float(p[1])


def recorded_route(start, waypoints, goal):
    seq = []
    for p in [start, *(waypoints or [])]:
        if p is None:
            continue
        yaw = float(p[2]) if len(p) > 2 else 0.0
        seq.append((float(p[0]), float(p[1]), yaw))
    if goal is not None:
        yaw = float(goal[2]) if len(goal) > 2 else (seq[-1][2] if seq else 0.0)
        seq.append((float(goal[0]), float(goal[1]), yaw))
    return collapse_points(seq)


def collapse_points(points, min_m=0.05):
    if not points:
        return []
    out = [points[0]]
    for p in points[1:]:
        a = out[-1]
        if math.hypot(p[0] - a[0], p[1] - a[1]) >= min_m:
            out.append(p)
        else:
            out[-1] = p
    return out


def round_trip_points(outbound):
    """나갔던 점을 그대로 밟고 시작으로 돌아온다."""
    if not outbound:
        return []
    if len(outbound) == 1:
        return list(outbound)
    back = list(reversed(outbound))
    return collapse_points(list(outbound) + back[1:], min_m=0.04)


def project_polyline(xy, points):
    """선 위 최근점. (cx, cy, seg_i, t, dist, s_along)."""
    if not points:
        return None
    px, py = _xy(xy)
    best = (points[0][0], points[0][1], 0, 0.0, math.hypot(px - points[0][0], py - points[0][1]), 0.0)
    s_before = 0.0
    for i in range(len(points) - 1):
        ax, ay = points[i][0], points[i][1]
        bx, by = points[i + 1][0], points[i + 1][1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            t = 0.0
            cx, cy = ax, ay
            dist = math.hypot(px - ax, py - ay)
            seg_len = 0.0
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
            cx, cy = ax + t * dx, ay + t * dy
            dist = math.hypot(px - cx, py - cy)
            seg_len = math.sqrt(l2)
        s = s_before + t * seg_len
        if dist < best[4]:
            best = (cx, cy, i, t, dist, s)
        s_before += seg_len
    if len(points) == 1:
        return best
    last = points[-1]
    dist_last = math.hypot(px - last[0], py - last[1])
    if dist_last < best[4]:
        best = (last[0], last[1], max(0, len(points) - 2), 1.0, dist_last, s_before)
    return best


def dist_to_polyline(xy, points):
    hit = project_polyline(xy, points)
    if hit is None:
        return 1e9
    return hit[4]


def remaining_from(here, polyline, arrive_m=0.15):
    """폴리라인 투영 지점부터 끝까지. 이미 지난 점은 버린다."""
    if not polyline:
        return []
    hit = project_polyline(here, polyline)
    if hit is None:
        return [(p[0], p[1]) for p in polyline]
    cx, cy, seg_i, t, dist, _s = hit
    if t >= 0.85 and seg_i + 1 < len(polyline):
        start_i = seg_i + 1
        pts = [(polyline[start_i][0], polyline[start_i][1])]
        pts.extend((p[0], p[1]) for p in polyline[start_i + 1 :])
    else:
        pts = [(cx, cy)]
        pts.extend((p[0], p[1]) for p in polyline[seg_i + 1 :])
    if pts and math.hypot(here[0] - pts[0][0], here[1] - pts[0][1]) < arrive_m:
        pts = pts[1:]
    return collapse_points([(p[0], p[1], 0.0) if len(p) < 3 else p for p in pts], min_m=0.04)


def round_trip_remaining(here, start, waypoints, goal, arrive_m=0.15):
    outbound = recorded_route(start, waypoints, goal)
    trip = round_trip_points(outbound)
    rest = remaining_from(here, trip, arrive_m=arrive_m)
    return outbound, trip, rest


def densify_route(pts, step_m=0.45, max_n=40):
    if not pts:
        return []
    out = [pts[0]]
    for x1, y1, yaw1 in pts[1:]:
        x0, y0, yaw0 = out[-1]
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(round(dist / step_m)))
        for k in range(1, n + 1):
            t = k / n
            heading = math.atan2(y1 - y0, x1 - x0) if dist > 1e-6 else yaw1
            yaw = heading if k < n else yaw1
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw if k == n else yaw0))
            if k < n:
                out[-1] = (out[-1][0], out[-1][1], heading)
    if len(out) <= max_n:
        return out
    step = len(out) / max_n
    thin = [out[min(len(out) - 1, int(i * step))] for i in range(max_n)]
    if thin[-1] != out[-1]:
        thin[-1] = out[-1]
    return thin


def lateral_offsets(x, y, yaw, lateral_m, n=1):
    """경로 진행 방향의 법선으로 옆 점. 가운데 포함."""
    out = [(x, y)]
    if lateral_m <= 1e-9 or n <= 0:
        return out
    nx, ny = -math.sin(yaw), math.cos(yaw)
    step = lateral_m / n
    for k in range(1, n + 1):
        d = k * step
        out.append((x + d * nx, y + d * ny))
        out.append((x - d * nx, y - d * ny))
    return out


def point_along(points, i, lookahead_m):
    """points[i]부터 선 따라 lookahead_m 앞. (x, y, heading)."""
    if not points:
        return None
    if i >= len(points):
        p = points[-1]
        return p[0], p[1], 0.0
    left = lookahead_m
    x, y = points[i][0], points[i][1]
    heading = 0.0
    j = i
    while j + 1 < len(points) and left > 1e-9:
        x1, y1 = points[j + 1][0], points[j + 1][1]
        dx, dy = x1 - x, y1 - y
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            j += 1
            continue
        heading = math.atan2(dy, dx)
        if seg >= left:
            t = left / seg
            return x + t * dx, y + t * dy, heading
        left -= seg
        x, y = x1, y1
        j += 1
    p = points[-1]
    if len(points) >= 2:
        heading = math.atan2(points[-1][1] - points[-2][1], points[-1][0] - points[-2][0])
    return p[0], p[1], heading
