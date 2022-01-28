# -*- coding: UTF-8 -*-
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
import os
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
import argparse

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

parser = argparse.ArgumentParser(description='このプログラムの説明（なくてもよい）')
parser.add_argument('-m','--distance')
args = parser.parse_args()
def move(m):
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(moveBy(m,0,0,0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

if __name__ == '__main__':
    move(args.distance)