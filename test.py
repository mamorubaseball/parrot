# -*- coding: UTF-8 -*-
#仕様書の作成
'''
関数の説明をここで記入
drone_by()


'''
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
import olympe.enums.move as mode
import math
from math import sin, cos, tan, atan2, acos, pi
import os, csv, time, tempfile
from phote import *
# from phote import *
import csv
import pandas as pd

DRONE_IP = "192.168.42.1"

start = [35.7099482, 139.5230989, 1.0]
p0 = [35.7099068, 139.5231090, 1.0]
goal = [35.709901, 139.523350, 1.0]



def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()

def practice():
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    set_gimbal(drone)
    time.sleep(5)
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    move_take_phote(drone,start)
    print('=====================start=====================================')
    move_take_phote(drone,p0)
    print('=====================p0=====================================')
    move_take_phote(drone,goal)
    print('=====================GOAL=====================================')
    drone(Landing()).wait()
    drone_gps = drone.get_state(PositionChanged)
    print(get_distance(goal[0], goal[1], drone_gps['latitude'], drone_gps['longitude'], 8))

def move_to():
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone_direction=0
    assert drone(TakeOff()).wait().success()
    time.sleep(2)

    drone(moveTo(start[0],start[1],start[2],MoveTo_Orientation_mode.TO_TARGET,0.0)
        >> moveToChanged(status='DONE', _timeout=10)).wait()

    assert drone(Landing()).wait().success()
    time.sleep(3)



def move_by(drone):
    sita=int(input())

    sita=90
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    time.sleep(10)
    sita=math.pi/2
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()


if __name__ == "__main__":
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(1)
    moveBy(drone)
    assert drone(Landing).wait().success()
    time.sleep(1)




