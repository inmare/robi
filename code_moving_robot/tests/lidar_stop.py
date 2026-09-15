"""이미 돌고 있는 X4 Pro 모터만 끈다. 스캔하지 않는다."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot.lidar import PORT, force_motor_off


def main():
    force_motor_off(PORT)
    print(f"DTR high, HUPCL off ({PORT}). 모터가 느려지거나 멈추는지 보세요.")
    print("변화가 없으면 USB를 뽑는 수밖에 없다. 어댑터에 DTR이 안 이어진 경우다.")


if __name__ == "__main__":
    main()
