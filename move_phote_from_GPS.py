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
import csv
import pandas as pd


start = [35.7099482, 139.5230989, 1.0]
p0 = [35.7099068, 139.5231090, 1.0]
goal = [35.709901, 139.523350, 1.0]


def get_now_gps(drone):
    # Wait for GPS fix
    drone(GPSFixStateChanged(_policy='wait'))
    return drone.get_state(HomeChanged)

#回転はさせないで移動させる
def calcurate(drone, p):
    gps = get_now_gps(drone)
    print('=' * 10)
    lat1, log1, alt1 = gps['latitude'], gps['longitude'], gps['altitude']
    lat2, log2, alt2 = p[0], p[1], p[2]
    disctance = get_distance(lat1, log1, lat2, log2, 8)
    direction = get_direction(lat1, log1, lat2, log2)
    x = disctance * math.cos(math.radians(direction))
    y = disctance * math.sin(math.radians(direction))
    z = 0
    if x > 5:x = 5
    elif x < -5:x = -5
    if y > 5:y = 5
    elif y < -5:y = -5
    return x, y, z, direction

#回転させて前(y)に進めための関数
def distance_direction(drone, p):
    gps = get_now_gps(drone)
    print('=' * 10)
    print('=========dorne_GPS'+str(gps))
    lat1, log1, alt1 = gps['latitude'], gps['longitude'], gps['altitude']
    lat2, log2, alt2 = p[0], p[1], p[2]
    distance = get_distance(lat1, log1, lat2, log2, 8)
    direction = get_direction(lat1, log1, lat2, log2)
    return distance,direction

def get_distance(lat1, log1, lat2, log2, precision):
    distance = 0
    if abs(lat1 - lat2) < 0.000001 and abs(log1 - log2) < 0.0000001:
        distance = 0
    else:
        lat1 = lat1 * math.pi / 180
        lat2 = lat2 * math.pi / 180
        log1 = log1 * math.pi / 180
        log2 = log2 * math.pi / 180
        A = 6378140
        B = 6356755
        F = (A - B) / A
        P1 = math.atan((B / A) * math.tan(lat1))
        P2 = math.atan((B / A) * math.tan(lat2))
        X = math.acos(math.sin(P1) * math.sin(P2) + math.cos(P1) * math.cos(P2) * math.cos(log1 - log2))
        L = (F / 8) * ((math.sin(X) - X) * math.pow((math.sin(P1) + math.sin(P2)), 2) / math.pow(math.cos(X / 2), 2) - (
                math.sin(X) - X) * math.pow(math.sin(P1) - math.sin(P2), 2) / math.pow(math.sin(X), 2))
        distance = A * (X + L)
        decimal_no = math.pow(10, precision)
        distance = round(decimal_no * distance / 1) / decimal_no
        return distance

def get_direction(lat1, log1, lat2, log2):
    Y = math.cos(log2 * math.pi / 180) * math.sin(lat2 * math.pi / 180 - lat1 * math.pi / 180);
    X = math.cos(log1 * math.pi / 180) * math.sin(log2 * math.pi / 180) - math.sin(log1 * math.pi / 180) * math.cos(
        log2 * math.pi / 180) * math.cos(lat2 * math.pi / 180 - lat1 * math.pi / 180)
    dirE0 = 180 * math.atan2(Y, X) / math.pi;  # 東向けがゼロ
    if dirE0 < 0:
        dirE0 += 360
    dirN0 = (dirE0 + 90) % 360
    dirN0 = dirN0 / 360 * math.pi
    return dirN0  # 北をゼロとして、角

#kakudo
# def azimuth(x1, y1, x2, y2):
#     # Radian角に修正
#     _x1, _y1, _x2, _y2 = x1*pi/180, y1*pi/180, x2*pi/180, y2*pi/180
#     Δx = _x2 - _x1
#     _y = sin(Δx)
#     _x = cos(_y1) * tan(_y2) - sin(_y1) * cos(Δx)

#     psi = atan2(_y, _x) * 180 / pi
#     if psi < 0:
#         return 360 + atan2(_y, _x) * 180 / pi
#     else:
#         return atan2(_y, _x) * 180 / pi
# #距離
# def distance(x1, y1, x2, y2, 6378.137e3):
#     _x1, _y1, _x2, _y2 = x1*pi/180, y1*pi/180, x2*pi/180, y2*pi/180
#     Δx = _x2 - _x1
#     val = sin(_y1) * sin(_y2) + cos(_y1) * cos(_y2) * cos(Δx)
#     return r * acos(val)*1000

#回転させて進んだら写真撮る関数[pは
def move_take_phote(drone,p,drone_direction):
    distance, direction = distance_direction(drone, p)
    sita=direction-drone_direction
    print('============sita={}==========='.format(str(sita)))
    if distance>4:
        distance=4
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    setup_photo_burst_mode(drone)
    take_photo_burst(drone)
    return direction

def move_take_phote_sita(drone,distance,sita):
    if distance>4:
        distance=4
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    setup_photo_burst_mode(drone)
    take_photo_burst(drone)


def move_take_phote_moveTo():
    start = time.time()
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone_direction=0
    set_gimbal(drone)
    time.sleep(5)
    df=pd.read_csv('CSV/GPS10_1.csv')
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    for i,d in df.iterrows():
        #２分以上の飛行をNGとする
        if time.time()-start>120:
            print('=========２分以上の飛行========')
            break
        gps=[d[0],d[1],d[2]]
        drone(moveTo(d[0], d[1], d[2], MoveTo_Orientation_mode.TO_TARGET, 0.0)
              >> moveToChanged(status='hovering', _timeout=10)).wait()
        time.sleep(3)
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)

    drone(Landing()).wait()
    drone_gps = drone.get_state(PositionChanged)
    print(get_distance(goal[0], goal[1], drone_gps['latitude'], drone_gps['longitude'], 8))


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

def main():
    start = time.time()
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone_direction=0
    set_gimbal(drone)
    time.sleep(5)
    df=pd.read_csv('CSV/GPS10_1.csv')
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    for i in range(len(df)-1):
        print('===========droneGPS{}====='.format(get_now_gps(drone)))
        if time.time() - start > 120:
            print('=========２分以上の飛行========')
            break
        drone_p=df.loc[i,:]
        gps=df.loc[i+1,:]
        lat1,log1=drone_p[0],drone_p[1]
        lat2,log2=gps[0],gps[1]
        distance = get_distance(lat1, log1, lat2, log2, 8)
        direction = get_direction(lat1, log1, lat2, log2)
        sita=drone_direction-direction
        move_take_phote_sita(drone,distance,sita)
        drone_direction=direction
        print('distance='+str(distance))
        print('direction'+str(direction))
    # for i,d in df.iterrows():
    #     #２分以上の飛行をNGとする
    #     if time.time()-start>120:
    #         print('=========２分以上の飛行========')
    #         break
    #     if i==5:
    #         break
    #     gps=[d[0],d[1],d[2]]
    #     direct=move_take_phote(drone, gps,drone_direction)
    #     drone_direction=direct
    #     print('======現在地点{}==========='.format(gps))
    #     print('======ドローン方向{}==========='.format(drone_direction))
    drone(Landing()).wait()
    drone_gps = drone.get_state(PositionChanged)
    print(get_distance(goal[0], goal[1], drone_gps['latitude'], drone_gps['longitude'], 8))

if __name__ == '__main__':
    main()