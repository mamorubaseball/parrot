# -*- coding: UTF-8 -*-
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
import os
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
import argparse

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

parser = argparse.ArgumentParser(description='右に移動させる場合は正の値、左に移動させる場合は負の値を代入')
parser.add_argument('-r','--rotation',type=float)
args = parser.parse_args()
def slide(r):
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(moveBy(0,0,0,r)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
    time.sleep(3)
    drone.disconnect()

if __name__ == '__main__':
    slide(args.rotation)