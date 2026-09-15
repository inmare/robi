"""X4 Pro 전방 거리 시험. 종료·예외 때 모터를 반드시 끈다."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import ydlidar
from robot.lidar import Lidar


def main():
    lidar = Lidar()
    try:
        lidar.start()
        print("스캔 중. Ctrl+C 하면 모터를 끄고 끝낸다.")
        while ydlidar.os_isOk():
            points = lidar.read()
            if points is None:
                print("스캔 실패")
                continue
            print("점 개수:", len(points))
            for angle, rng in points:
                deg = angle * 180.0 / math.pi
                if abs(deg) < 5:
                    print(f"전방 {deg:.1f}도, {rng:.3f} m")
                    break
    except KeyboardInterrupt:
        print("Ctrl+C")
    finally:
        lidar.close()
        print("라이다 모터 OFF, 포트 닫음")


if __name__ == "__main__":
    main()
