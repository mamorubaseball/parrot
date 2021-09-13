from __future__ import print_function  # python2/3 compatibility for the print function
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
import os, csv, time, tempfile
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged,moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
import olympe.enums.move as mode
def location():
    # Connection
    drone = olympe.Drone("192.168.42.1")
    drone.connection()

    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy = 'wait'))
    print("GPS position before take-off :", drone.get_state(HomeChanged))
    # Take-off
    drone(TakeOff()).wait()
    print("GPS position after take-off : ", drone.get_state(PositionChanged))
    drone.disconnection()
def moveto(drone):
    drone(moveTo(
        35.709929,
        139.523326,
        1.0,
        MoveTo_Orientation_mode.TO_TARGET,
        0.0
        )
        >> moveToChanged(status='DONE', _timeout=10)
    ).wait()
#時計台からスタートして、木台まで移動する関数
def mokudai():
    start=[35.709929,139.523326,1.0]
    p0=[35.709901,139.523328,1.0]
    goal=[35.709901,139.523350,1.0]

    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone(TakeOff()).wait(2)
    drone(moveTo(start[0],start[1],start[2],MoveTo_Orientation_mode.TO_TARGET,0.0)
         >> moveToChanged(status='DONE', _timeout=10)).wait(2)
    print('GOAL')
    drone(Landing()).wait()
    print("GPS position after take-off : ", drone.get_state(PositionChanged))
    drone.disconnection()
      
#去年の研究まで
def move_by_gpsdata(gps_data):
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone(TakeOff()).wait(2)
    p_lsts=[[35.709903,139.523559,2.0],[35.709888,139.523511,2.0],[35.709846,139.523514,2.0]]
    for p in gps_data:
        drone(moveTo(p[0], p[1], p[2], MoveTo_Orientation_mode.TO_TARGET, 0.0)
              >> moveToChanged(status='DONE', _timeout=10)).wait()

    print('GOAL')
    drone(TakeOff()).wait()
    print("GPS position after take-off : ", drone.get_state(PositionChanged))
    drone.disconnection()


if __name__ == '__main__':
    mokudai()
