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
from phote import *
import csv
import pandas as pd
from simulation import simulation

start = [35.7099482, 139.5230989, 1.0]
p0 = [35.7099068, 139.5231090, 1.0]
goal = [35.709867, 139.523072, 1.0]
CSV_FILE='CSV/orange.csv'
DRONE_IP='192.168.42.1'


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
    Y = math.cos(log2 * math.pi / 180) * math.sin(lat2 * math.pi / 180 - lat1 * math.pi / 180)
    X = math.cos(log1 * math.pi / 180) * math.sin(log2 * math.pi / 180) - math.sin(log1 * math.pi / 180) * math.cos(
        log2 * math.pi / 180) * math.cos(lat2 * math.pi / 180 - lat1 * math.pi / 180)
    dirE0 = 180 * math.atan2(Y, X) / math.pi # 東向けがゼロ
    if dirE0 < 0:
        dirE0 += 360
    dirN0 = (dirE0 + 90) % 360
    dirN0 = dirN0 / 360 * math.pi
    return dirN0  # 北をゼロとして、角



# 楕円体
ELLIPSOID_GRS80 = 1 # GRS80
ELLIPSOID_WGS84 = 2 # WGS84

# 楕円体ごとの長軸半径と扁平率
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80]長軸半径
        1 / 298.257222101, # [GRS80]扁平率
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84]長軸半径
        1 / 298.257223563, # [WGS84]扁平率
    ],
}

# 反復計算の上限回数
ITERATION_LIMIT = 1000

'''
Vincenty法(逆解法)
2地点の座標(緯度経度)から、距離と方位角を計算する
:param lat1: 始点の緯度
:param lon1: 始点の経度
:param lat2: 終点の緯度
:param lon2: 終点の経度
:param ellipsoid: 楕円体
:return: 距離と方位角
'''
def vincenty_inverse(lat1, lon1, lat2, lon2, ellipsoid=None):

    # 差異が無ければ0.0を返す
    if isclose(lat1, lat2) and isclose(lon1, lon2):
        return {
            'distance': 0.0,
            'azimuth1': 0.0,
            'azimuth2': 0.0,
        }

    # 計算時に必要な長軸半径(a)と扁平率(ƒ)を定数から取得し、短軸半径(b)を算出する
    # 楕円体が未指定の場合はGRS80の値を用いる
    a, ƒ = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
    b = (1 - ƒ) * a

    φ1 = radians(lat1)
    φ2 = radians(lat2)
    λ1 = radians(lon1)
    λ2 = radians(lon2)

    # 更成緯度(補助球上の緯度)
    U1 = atan((1 - ƒ) * tan(φ1))
    U2 = atan((1 - ƒ) * tan(φ2))

    sinU1 = sin(U1)
    sinU2 = sin(U2)
    cosU1 = cos(U1)
    cosU2 = cos(U2)

    # 2点間の経度差
    L = λ2 - λ1

    # λをLで初期化
    λ = L

    # 以下の計算をλが収束するまで反復する
    # 地点によっては収束しないことがあり得るため、反復回数に上限を設ける
    for i in range(ITERATION_LIMIT):
        sinλ = sin(λ)
        cosλ = cos(λ)
        sinσ = sqrt((cosU2 * sinλ) ** 2 + (cosU1 * sinU2 - sinU1 * cosU2 * cosλ) ** 2)
        cosσ = sinU1 * sinU2 + cosU1 * cosU2 * cosλ
        σ = atan2(sinσ, cosσ)
        sinα = cosU1 * cosU2 * sinλ / sinσ
        cos2α = 1 - sinα ** 2
        cos2σm = cosσ - 2 * sinU1 * sinU2 / cos2α
        C = ƒ / 16 * cos2α * (4 + ƒ * (4 - 3 * cos2α))
        λʹ = λ
        λ = L + (1 - C) * ƒ * sinα * (σ + C * sinσ * (cos2σm + C * cosσ * (-1 + 2 * cos2σm ** 2)))

        # 偏差が.000000000001以下ならbreak
        if abs(λ - λʹ) <= 1e-12:
            break
    else:
        # 計算が収束しなかった場合はNoneを返す
        return None

    # λが所望の精度まで収束したら以下の計算を行う
    u2 = cos2α * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    Δσ = B * sinσ * (cos2σm + B / 4 * (cosσ * (-1 + 2 * cos2σm ** 2) - B / 6 * cos2σm * (-3 + 4 * sinσ ** 2) * (-3 + 4 * cos2σm ** 2)))

    # 2点間の楕円体上の距離
    s = b * A * (σ - Δσ)

    # 各点における方位角
    α1 = atan2(cosU2 * sinλ, cosU1 * sinU2 - sinU1 * cosU2 * cosλ)
    α2 = atan2(cosU1 * sinλ, -sinU1 * cosU2 + cosU1 * sinU2 * cosλ) + pi

    if α1 < 0:
        α1 = α1 + pi * 2

    return {
        'distance': s,           # 距離
        'azimuth1': degrees(α1), # 方位角(始点→終点)
        'azimuth2': degrees(α2), # 方位角(終点→始点)
    }





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
def move_take_phote_2(drone,p,drone_direction):
    gps = get_now_gps(drone)
    lat1,log1=gps['latitude'],gps['longitude']
    lat2,log2=p[0],p[1]
    result = vincenty_inverse(lat1, log1, lat2, log2, 1)
    distance = round(result['distance'], 3)
    direction = result['azimuth1']
    sita=direction-drone_direction
    sita = (sita / 180) * pi
    print('============sita={}==========='.format(str(sita)))
    if distance>4:
        print('4メートルを超えた飛行はできません')
        distance=4
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    # drone(moveBy(distance, 0, 0, 0)
    #       >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    drone(extended_move_by(distance,0,0,0,1,0.5,0.5)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    setup_photo_burst_mode(drone)
    take_photo_burst(drone)
    return direction,gps


#スタート地点はいつもの木の上
def move_lst():
    drone=olympe.Drone(DRONE_IP)
    drone.connect()
    drone(TakeOff()
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    sita_lsts,distance_lsts=simulation(CSV_FILE)

    for sita,dis in zip(sita_lsts,distance_lsts):
        drone(moveBy(0, 0, 0, sita)
              >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

        drone(extended_move_by(dis, 0, 0, 0, 1, 0.5, 0.5)
              >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

        setup_photo_burst_mode(drone)
        take_photo_burst(drone)

    drone(Landing()).wait().success()
def move_take_phote_sita(drone,distance,sita):
    if distance>4:
        distance=4
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()

    setup_photo_burst_mode(drone)
    take_photo_burst(drone)
    drone(FlyingStateChanged(state='hovering'))


def take_phote_moveTo():
    start = time.time()
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    drone_direction=0
    set_gimbal(drone)
    time.sleep(5)
    df=pd.read_csv(CSV_FILE)
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    for i,d in df.iterrows():
        #２分以上の飛行をNGとする
        if time.time()-start>120:
            print('=========２分以上の飛行========')
            break
        drone(extended_move_to(d[0], d[1],0,0,0,1.0,1.0,0.5)
              >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
    drone(Landing()).wait()
    drone_gps = drone.get_state(PositionChanged)
    print(get_distance(goal[0], goal[1], drone_gps['latitude'], drone_gps['longitude'], 8))

def main():
    start = time.time()
    drone = olympe.Drone(DRONE_IP)
    drone.connection()
    drone_direction=0
    set_gimbal(drone)
    time.sleep(2)
    df=pd.read_csv(CSV_FILE)
    drone_gps_lst = []
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    drone_direction = 0
    for i in range(len(df)):
        d=df.loc[i,:]
        #２分以上の飛行をNGとする
        if time.time()-start>360:
            print('=========２分以上の飛行========')
            break
        gps=[d[0],d[1],d[2]]
        direct,drone_gps=move_take_phote_2(drone, gps,drone_direction)
        drone_direction=direct
        time.sleep(3)

        drone_gps_lst.append(drone_gps)
        print('======現在地点{}==========='.format(gps))
        print('=======ドローン地点{}======'.format(drone_gps))
        print('======ドローン方向{}==========='.format(drone_direction))
    drone(Landing()).wait()
    drone_gps = drone.get_state(PositionChanged)
    print(get_distance(goal[0], goal[1], drone_gps['latitude'], drone_gps['longitude'], 8))



if __name__ == '__main__':
    # take_phote_moveTo()
    move_lst()
    # main()

