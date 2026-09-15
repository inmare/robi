"""YDLIDAR X4 Pro. 쓸 때만 모터를 켜고, 안 쓰면 turnOff 한다."""

import atexit
import ydlidar

PORT = "/dev/ttyUSB0"
BAUD = 128000
SCAN_HZ = 7.0
SAMPLE_RATE = 5


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
        """스캔 모터만 끈다. USB는 꽂아 둬도 된다. 다시 start() 하면 재개."""
        if self._motor and self.laser is not None:
            self.laser.turnOff()
        self._motor = False

    def close(self):
        """모터를 끄고 포트를 닫는다. 프로그램 종료·미사용 시 이걸 쓴다."""
        self.stop()
        if self._open and self.laser is not None:
            self.laser.disconnecting()
        self.laser = None
        self._open = False

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
