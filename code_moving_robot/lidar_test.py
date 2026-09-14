import math
import ydlidar

ydlidar.os_init()
laser = ydlidar.CYdLidar()

laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 7.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)

ok = laser.initialize()
if not ok:
    print("초기화 실패")
else:
    ok = laser.turnOn()
    scan = ydlidar.LaserScan()
    while ok and ydlidar.os_isOk():
        if laser.doProcessSimple(scan):
            print("점 개수:", scan.points.size())
            for p in scan.points:
                deg = p.angle * 180.0 / math.pi
                if abs(deg) < 5:
                    print(f"전방 {deg:.1f}도, {p.range:.3f} m")
                    break
        else:
            print("스캔 실패")
    laser.turnOff()
    laser.disconnecting()