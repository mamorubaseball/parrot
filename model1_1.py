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
from phote import *


DRONE_IP = "192.168.42.1"
def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connection()
    drone(TakeOff()).wait().success()
    first_rotation=2*(80/360)/math.pi
    right_rotation=2*(100/360)/math.pi
    left_rotation=2*(80/360)/math.pi


    drone(moveBy(0, 0, 0, first_rotation)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    def left(drone):
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
        
        drone(moveBy(3.8, 0, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
        drone(moveBy(0, 0, 0, left_rotation)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        
        drone(moveBy(1.75, 0, 0, 0)
        >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        drone(moveBy(0, 0, 0, right_rotation)
        >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
    
    def right(drone):
        drone(moveBy(3.8, 0, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
        drone(moveBy(0, 0, 0, -right_rotation)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        drone(moveBy(1.75, 0, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
        drone(moveBy(0, 0, 0, -right_rotation)
        >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    for i in range(5):
        left(drone)
        right(drone)

if __name__ == '__main__':
    main()
