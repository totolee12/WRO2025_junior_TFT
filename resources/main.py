#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import Font
from  mindsensorsPYB import PPS58
from  mindsensorsPYB import EV3Matrix
from  mindsensorsPYB import DIST_ToF
from  mindsensorsPYB import EV3RFid
from  mindsensorsPYB import LINELEADER
from  mindsensorsPYB import TFTPACK
from  mindsensorsPYB import ABSIMU
from  mindsensorsPYB import IRThermometer
from  mindsensorsPYB import VOLT
from  mindsensorsPYB import PFMATE
from  mindsensorsPYB import SUMOEYES

import os
import sys
import time


def ABSIMUTest() :
# Create your objects here.
    ev3 = EV3Brick()
    imu = ABSIMU(Port.S1,0x22)


    # Write your program here.
    print(imu.GetDeviceId())
    print(imu.GetVendorName())
    print(imu.GetFirmwareVersion())
    print(imu.get_accelall(),imu. get_heading(),imu.get_gyroall())
    count = 0
    while count < 200 :
        count =count +1
        time.sleep(0.1) 
        print(imu.get_accelall(),imu. get_heading(),imu.get_gyroall())
    ev3.speaker.beep()



ABSIMUTest()
