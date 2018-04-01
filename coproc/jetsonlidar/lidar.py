import serial
import time
from networktables import NetworkTables
NetworkTables.initialize(server="10.28.98.2")
sd = NetworkTables.getTable("SmartDashboard")

ard = serial.Serial("/dev/ttyUSB0", 9600)

LIDAR1X=0
LIDAR2X=10
LIDAR3X=20
LIDAR4X=30


while True:
    try:
        thingy = ard.readline()
        v1, v2, v3, v4 = [int(i) for i in thingy.split(',')]
        sd.putNumber("LIDAR1",v1)
        sd.putNumber("LIDAR2",v2)
        sd.putNumber("LIDAR3",v3)
        sd.putNumber("LIDAR4",v4)
        print(v1,v2,v3,v4)
        degrees = (v2 - v1)/(LIDAR2X-LIDAR1X)
        sd.putNumber("cube degrees",degrees)
    except:pass
    time.sleep(1.0/80.0)
