"""경로·지도 슬롯 4개. 이름과 기록 시각을 같이 둔다."""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from robot.grid import OccupancyGrid
from robot.pins import TRACK_M

SLOT_COUNT = 4
DEFAULT_NAME = "이름 없는 경로"


@dataclass
class SlotInfo:
    index: int
    empty: bool
    name: str
    saved_at: str | None
    saved_at_iso: str | None
    directory: Path

    def label(self):
        if self.empty:
            return "[비어 있음]"
        when = self.saved_at or ""
        return f"{self.name}    {when}".rstrip()


class SlotStore:
    def __init__(self, root: Path, n=SLOT_COUNT):
        self.root = Path(root)
        self.n = n
        self.root.mkdir(parents=True, exist_ok=True)

    def slot_dir(self, index):
        self._check(index)
        return self.root / f"slot_{index}"

    def list(self):
        return [self.info(i) for i in range(1, self.n + 1)]

    def info(self, index):
        folder = self.slot_dir(index)
        meta_path = folder / "meta.json"
        route_path = folder / "route.json"
        if not meta_path.exists() and not route_path.exists():
            return SlotInfo(index, True, "", None, None, folder)
        name = DEFAULT_NAME
        iso = None
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, json.JSONDecodeError, TypeError):
                meta = {}
            if meta.get("empty"):
                return SlotInfo(index, True, "", None, None, folder)
            name = str(meta.get("name") or DEFAULT_NAME).strip() or DEFAULT_NAME
            iso = meta.get("saved_at")
        if not route_path.exists():
            return SlotInfo(index, True, "", None, None, folder)
        return SlotInfo(index, False, name, _fmt_when(iso), iso, folder)

    def load(self, index):
        """(data, grid, info). 비었거나 깨지면 None."""
        info = self.info(index)
        if info.empty:
            return None
        folder = info.directory
        json_path = folder / "route.json"
        pgm_path = folder / "map.pgm"
        try:
            data = json.loads(json_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError, TypeError):
            return None
        grid = None
        if pgm_path.exists():
            try:
                meta = data.get("grid") or {}
                grid = OccupancyGrid.from_pgm(
                    pgm_path,
                    size_m=float(meta.get("size_m", 16.0)),
                    resolution=float(meta.get("resolution", 0.05)),
                )
            except (OSError, ValueError, TypeError):
                grid = None
        if grid is None:
            grid = OccupancyGrid(size_m=16.0, resolution=0.05)
        return data, grid, info

    def save(
        self,
        index,
        grid,
        odo,
        start,
        waypoints,
        goal,
        name,
        replace_map=True,
        ascii_map=None,
    ):
        folder = self.slot_dir(index)
        folder.mkdir(parents=True, exist_ok=True)
        if replace_map and grid is not None:
            grid.save_pgm(folder / "map.pgm")
            grid.save_bmp(folder / "map.bmp")
        if ascii_map is None and grid is not None and odo is not None:
            ascii_map = grid.render_ascii(
                pose=(odo.x, odo.y),
                marks=_marks(start, waypoints, goal),
            )
        if ascii_map is not None:
            (folder / "map.txt").write_text(ascii_map + "\n", encoding="utf-8")
        now = datetime.now().astimezone()
        iso = now.isoformat(timespec="seconds")
        clean = (name or "").strip() or DEFAULT_NAME
        data = {
            "start": start,
            "waypoints": waypoints,
            "goal": goal,
            "pose": _pose_tuple(odo) if odo is not None else [0.0, 0.0, 0.0],
            "track_m": TRACK_M,
            "grid": {
                "size_m": grid.size_m if grid is not None else 16.0,
                "resolution": grid.resolution if grid is not None else 0.05,
            },
            "name": clean,
            "saved_at": iso,
        }
        (folder / "route.json").write_text(
            json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        (folder / "meta.json").write_text(
            json.dumps(
                {"name": clean, "saved_at": iso, "empty": False},
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        return self.info(index)

    def _check(self, index):
        if index < 1 or index > self.n:
            raise ValueError(f"슬롯은 1..{self.n}")


def _fmt_when(iso):
    if not iso:
        return None
    try:
        dt = datetime.fromisoformat(iso)
        return dt.strftime("%Y-%m-%d %H:%M")
    except ValueError:
        return str(iso)


def _pose_tuple(odo):
    return [round(odo.x, 3), round(odo.y, 3), round(odo.yaw, 3)]


def _marks(start, waypoints, goal):
    marks = []
    if start:
        marks.append((start[0], start[1], "S"))
    for i, wp in enumerate(waypoints or [], start=1):
        marks.append((wp[0], wp[1], str(min(i, 9))))
    if goal:
        marks.append((goal[0], goal[1], "G"))
    return marks
