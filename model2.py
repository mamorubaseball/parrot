
from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo,moveBy,Circle, PCMD
# from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged,moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
import olympe.enums.move as mode
import math
import os, csv, time, tempfile
import pandas as pd
# from move_phote_from_GPS import get_distance,get_direction
import time
import math
from math import *
from olympe.messages.move import extended_move_by,extended_move_to
from phote import *

CSV='CSV/ki2_2.csv'

DRONE_IP = "192.168.42.1"

def get_now_gps(drone):
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy='wait'))
    return drone.get_state(HomeChanged)

def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    start=time.time()
    df = pd.DataFrame(CSV)
    gps_list=[]

    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    for i,d in df.iterrows():
        #２分以上の飛行をNGとする
        if time.time()-start>120:
            print('=========２分以上の飛行========')
            break
        drone(extended_move_to(d[0], d[1],0,0,0,1.0,1.0,0.5)
              >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
        gps=get_now_gps(drone)
        gps_list.append([gps['latitude'],gps['longitude'],gps['altitude']])
    drone(Landing()).wait()

if __name__ == '__main__':
    main()





