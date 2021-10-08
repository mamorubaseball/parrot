import pandas as pd
# from move_phote_from_GPS import get_distance,get_direction
import time
import math

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

def simulation():
    df = pd.read_csv('GPS10_1.csv')
    start = time.time()
    # print(df.loc[2,:])

    drone_direction = 0
    for i in range(len(df)-1):
        drone=df.loc[i,:]
        gps=df.loc[i+1,:]
        lat1,log1=drone[0],drone[1]
        lat2,log2=gps[0],gps[1]
        distance = get_distance(lat1, log1, lat2, log2, 8)
        direction = get_direction(lat1, log1, lat2, log2)
        sita=drone_direction-direction

        drone_direction = direction
        print('distance='+str(distance))
        print('direction'+str(direction))
        print('======={}======'.format(i))

simulation()