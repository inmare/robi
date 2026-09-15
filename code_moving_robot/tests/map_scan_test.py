"""제자리 스캔으로 occupancy grid 를 maps/last_scan.pgm 에 저장한다.

로봇은 움직이지 않는다. 홀 오도메트리는 아직 안 붙인다.
끝나면 라이다 모터를 끈다.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import ydlidar
from robot.grid import OccupancyGrid
from robot.lidar import Lidar

SCANS = 15
OUT = ROOT / "maps" / "last_scan.pgm"


def main():
    grid = OccupancyGrid(size_m=12.0, resolution=0.05)
    lidar = Lidar()
    used = 0
    try:
        lidar.start()
        print(f"제자리 {SCANS}바퀴 스캔. 끝나면 {OUT}")
        while used < SCANS and ydlidar.os_isOk():
            points = lidar.read()
            if points is None:
                print("스캔 실패")
                continue
            grid.add_scan(0.0, 0.0, 0.0, points)
            used += 1
            print(f"{used}/{SCANS}  점 {len(points)}")
        path = grid.save_pgm(OUT)
        print("저장:", path)
    except KeyboardInterrupt:
        print("Ctrl+C")
        if used:
            path = grid.save_pgm(OUT)
            print("중간 저장:", path)
    finally:
        lidar.close()
        print("라이다 모터 OFF")


if __name__ == "__main__":
    main()
