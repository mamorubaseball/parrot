from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.move import extended_move_by,extended_move_to
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
import olympe.enums.move as mode
import math
from math import *
from math import sin, cos, tan, atan2, acos, pi
import os, csv, time, tempfile
from parrot.simulation import simulation
import csv
import pandas as pd

olympe.log.update_config({"loggers": {"olympe": {"level": "INFO"}}})
DRONE_IP = "192.168.42.1"
SKYCTRL_IP = "192.168.53.1"
DRONE_SSID = os.environ.get("DRONE_SSID", "Anafi_PC_000000")
DRONE_SECURITY_KEY = os.environ.get("DRONE_SECURITY_KEY", "")
DRONE_SERIAL = os.environ.get("DRONE_SERIAL", "000000")

#直線スタートとゴールの差
start=[35.709905295943436, 139.52311346901035,0]
goal=[35.70988297239259, 139.52310743404036,0]
def prepare(drone):
    assert drone(TakeOff()).wait().success()
    def get_now_gps(drone):
        # Wait for GPS fix
        drone(GPSFixStateChanged(_policy='wait'))
        return drone.get_state(HomeChanged)
    drone_gps=get_now_gps(drone)
    print(drone_gps['latitude'], drone_gps['longitude'])
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()
CSV_FILE='CSV/GPS.csv'
def move_by(drone):
    sita_l,dis_l=simulation(CSV_FILE)
    assert drone(TakeOff()).wait().success()
    time.sleep(3)

    drone(moveBy(0, 0, 0, sita_l[0])
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    drone(moveBy(dis_l[0], 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    assert drone(Landing()).wait().success()


def move_to(drone):
    assert drone(TakeOff()).wait().success()
    time.sleep(2)
    drone(extended_move_to(latitude=start[0], longitude=start[1], altitude=start[2],max_horizontal_speed=0.1))
    drone(extended_move_to(latitude=goal[0], longitude=goal[1], altitude=goal[2],max_horizontal_speed=0.1))
    assert drone(Landing()).wait().success()
    time.sleep(3)

if __name__ == '__main__':
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    prepare(drone)


    
    