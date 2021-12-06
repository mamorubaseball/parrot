# -*- coding: UTF-8 -*-
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
import os
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged


DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")


def left():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(moveBy(0,0.5, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

if __name__ == '__main__':
    left()