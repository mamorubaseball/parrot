# -*- coding: UTF-8 -*-
import olympe
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
import os

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")


def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(Landing()).wait().success()
    time.sleep(10)
    
def left():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(moveBy(0,0.1, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

def right():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone(moveBy(0,-0.1, 0, 0)
              >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()
   
if __name__ == "__main__":
    main()
