"""YDLIDAR X4 Pro. 모터는 시리얼 DTR로 켜고 끈다."""

import atexit
import fcntl
import os
import struct
import time

import ydlidar

PORT = "/dev/ttyUSB0"
BAUD = 128000
SCAN_HZ = 7.0
SAMPLE_RATE = 5

# linux/termios.h — USB-시리얼이 포트를 닫아도 DTR이 남아 있으면 모터가 계속 돈다
TIOCMBIC = 0x5417
TIOCM_DTR = 0x002
TIOCM_RTS = 0x004


def force_motor_off(port=PORT):
    """SDK 없이 DTR을 내려 모터를 끈다. 프로그램이 깨진 뒤에도 이걸 쓴다."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        bits = struct.pack("I", TIOCM_DTR | TIOCM_RTS)
        fcntl.ioctl(fd, TIOCMBIC, bits)
        time.sleep(0.4)
    finally:
        os.close(fd)


class Lidar:
    def __init__(self, port=PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.laser = None
        self._os_inited = False
        self._open = False
        self._motor = False
        atexit.register(self.close)

    def _make_laser(self):
        laser = ydlidar.CYdLidar()
        laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baud)
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, SCAN_HZ)
        laser.setlidaropt(ydlidar.LidarPropSampleRate, SAMPLE_RATE)
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        # X4/X4 Pro: DTR=1 시작, DTR=0 정지. 이게 False면 turnOff가 모터를 켜 둔다
        laser.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, True)
        return laser

    def start(self):
        """시리얼을 열고 모터를 돌린다. 이미 켜져 있으면 아무 것도 안 한다."""
        if not self._os_inited:
            ydlidar.os_init()
            self._os_inited = True
        if self._motor:
            return
        if not self._open:
            self.laser = self._make_laser()
            if not self.laser.initialize():
                self.laser.disconnecting()
                self.laser = None
                raise RuntimeError("라이다 초기화 실패")
            self._open = True
        if not self.laser.turnOn():
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            self._open = False
            raise RuntimeError("라이다 모터 시작 실패")
        self._motor = True

    def stop(self):
        """스캔 모터만 끈다. USB는 꽂아 둔 채 다시 start() 가능."""
        if self._motor and self.laser is not None:
            self.laser.turnOff()
            time.sleep(0.5)
        self._motor = False

    def close(self):
        """모터를 끄고 포트를 닫는다. 프로그램 종료·미사용 시 이걸 쓴다."""
        self.stop()
        if self._open and self.laser is not None:
            self.laser.disconnecting()
        self.laser = None
        self._open = False
        try:
            force_motor_off(self.port)
        except OSError:
            pass

    def read(self):
        """한 바퀴. 실패면 None. 성공이면 [(angle_rad, range_m), ...]."""
        if not self._motor or self.laser is None:
            raise RuntimeError("라이다가 꺼져 있다. start() 먼저")
        scan = ydlidar.LaserScan()
        if not self.laser.doProcessSimple(scan):
            return None
        points = []
        for p in scan.points:
            if p.range > 0:
                points.append((p.angle, p.range))
        return points

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
