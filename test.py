# -*- coding: UTF-8 -*-
#仕様書の作成
'''
関数の説明をここで記入
drone_by()
'''
# import olympe
# import time
# # from __future__ import print_function  # python2/3 compatibility for the print function
# from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.move import extended_move_by,extended_move_to
# from olympe.messages.ardrone3.PilotingState import PositionChanged
# from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
# from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
# from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
# from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
# from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
# import olympe.enums.move as mode

#from olympe.messages.ardrone3.PilotingSettings import MaxVerticalSpeed,
import math
from math import sin, cos, tan, atan2, acos, pi
import os, csv, time, tempfile
# from phote import *
# from phote import *
import csv
import pandas as pd

DRONE_IP = "192.168.42.1"
SKYCTRL_IP = "192.168.53.1"

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

def move_to(drone):
    assert drone(TakeOff()).wait().success()
    time.sleep(2)

    drone(moveTo(start[0],start[1],start[2],MoveTo_Orientation_mode.TO_TARGET,0.0)
        >> moveToChanged(status='DONE', _timeout=10)).wait()

    drone(extended_move_to(latitude=goal[0], longitude=goal[1], altitude=goal[2],max_horizontal_speed=0.1))
    assert drone(Landing()).wait().success()
    time.sleep(3)



def move_by(drone):
    sita=int(input())
    '''
    PARAMETERS
    d_x (float) – Wanted displacement along the front axis [m]
    d_y (float) – Wanted displacement along the right axis [m]
    d_z (float) – Wanted displacement along the down axis [m]
    d_psi (float) – Wanted rotation of heading [rad]
    max_horizontal_speed (float) – Maximum horizontal speed in m/s.
    max_vertical_speed (float) – Maximum vertical speed in m/s.
    max_yaw_rotation_speed (float) – Maximum vertical speed in degrees/s.
    rad:math.pi/2で90°回転(3.14//2)
    '''
    drone(extended_move_by(d_x=3, d_y=0, d_z=0, d_psi=0,max_horizontal_speed=0.1)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    time.sleep(10)
    sita=math.pi/3
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    assert drone(Landing()).wait().success()
def conect_skycontroller():
    drone = olympe.Drone(SKYCTRL_IP)
    drone.connect()
    drone(setPilotingSource(source="Controller")).wait()
    drone.disconnect()

def movie():






if __name__ == "__main__":
    # drone = olympe.Drone(DRONE_IP)
    # drone.connect()
    # assert drone(TakeOff()).wait().success()
    # time.sleep(1)
    # moveBy(drone)
    # assert drone(Landing).wait().success()
    # time.sleep(1)
    sita=2
    print(sita*180/math.pi)




