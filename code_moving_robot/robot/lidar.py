"""YDLIDAR X4 Pro. 모터는 시리얼 DTR로 켜고 끈다."""

import atexit
import array
import fcntl
import os
import termios
import time

import ydlidar

PORT = "/dev/ttyUSB0"
BAUD = 128000
SCAN_HZ = 7.0
SAMPLE_RATE = 5

# linux/ioctl.h
TIOCMBIS = 0x5416  # 비트 켜기
TIOCMBIC = 0x5417  # 비트 끄기
TIOCM_DTR = 0x002
TIOCM_RTS = 0x004

# X4 Pro 데이터시트: M_CTR 전압이 낮을수록 빠름. 0V = 최고속.
# 공식 어댑터는 DTR로 정지(despin)한다.
# SDK SupportMotorDtrCtrl True  → turnOff가 DTR을 내림 → 최고속 (지금 증상)
# False → turnOff가 DTR을 올림 → 정지
DTR_HIGH_STOPS_MOTOR = True


def _ioctl_bits(fd, op, mask):
    buf = array.array("I", [mask])
    fcntl.ioctl(fd, op, buf)


def force_motor_off(port=PORT):
    """DTR을 올린 채 포트를 닫는다. HUPCL을 끄지 않으면 닫을 때 DTR이 내려가 다시 최고속이 된다."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        if DTR_HIGH_STOPS_MOTOR:
            _ioctl_bits(fd, TIOCMBIS, TIOCM_DTR)
        else:
            _ioctl_bits(fd, TIOCMBIC, TIOCM_DTR)
        time.sleep(0.5)
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
        # False: start=DTR low(회전), stop=DTR high(정지). X4 Pro M_CTR 극성에 맞춤
        laser.setlidaropt(ydlidar.LidarPropSupportMotorDtrCtrl, False)
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
