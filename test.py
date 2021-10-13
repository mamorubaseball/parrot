# -*- coding: UTF-8 -*-
#仕様書の作成
'''
関数の説明をここで記入
drone_by()
'''
import olympe
import time
from __future__ import print_function  # python2/3 compatibility for the print function
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, moveBy, Circle, PCMD
from olympe.messages.move import extended_move_by,extended_move_to
from olympe.messages.ardrone3.PilotingState import PositionChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.GPSSettingsState import HomeChanged
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status as status
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
import olympe.enums.move as mode

#from olympe.messages.ardrone3.PilotingSettings import MaxVerticalSpeed,
import math
from math import sin, cos, tan, atan2, acos, pi
import os, csv, time, tempfile
# from phote import *
import csv
import pandas as pd

from collections import OrderedDict
import os

import olympe
from olympe.messages.drone_manager import (
    discover_drones,
    connection_state,
    connect,
    forget,
)

olympe.log.update_config({"loggers": {"olympe": {"level": "INFO"}}})

DRONE_IP = "192.168.42.1"
SKYCTRL_IP = "192.168.53.1"
DRONE_SSID = os.environ.get("DRONE_SSID", "Anafi_PC_000000")
DRONE_SECURITY_KEY = os.environ.get("DRONE_SECURITY_KEY", "")
DRONE_SERIAL = os.environ.get("DRONE_SERIAL", "000000")


class SkyControllerExample:
    def __init__(self):
        self.skyctrl = olympe.SkyController(SKYCTRL_IP)

    def skyctrl_connect(self):
        self.skyctrl.connect()

    def update_drones(self):
        discover_results = self.skyctrl(discover_drones()).wait(_timeout=10)
        assert discover_results.success(), "Update drone discovery timedout"
        drone_list_items = discover_results.received_events()
        known_drones = OrderedDict()
        visible_drones = OrderedDict()
        active_drone = None
        for drone_list_item in drone_list_items:
            if drone_list_item.args["visible"] == 1:
                visible_drones[
                    drone_list_item.args["serial"]] = drone_list_item.args
            if drone_list_item.args["active"] == 1:
                active_drone = drone_list_item.args["serial"]
            if drone_list_item.args["connection_order"] != 0:
                known_drones[
                    drone_list_item.args["serial"]] = drone_list_item.args

        self.active_drone = active_drone
        self.known_drones = known_drones
        self.visible_drones = visible_drones

        print("Active drone: ", self.active_drone)
        print("Known drones: ", ", ".join(self.known_drones))
        print("Visible drones: ", ", ".join(self.visible_drones))

    def connect_drone(self, drone_serial, drone_security_key=""):
        self.update_drones()
        if self.active_drone == drone_serial:
            print(
                "SkyController is already connected to {}".format(
                    drone_serial))
            return True
        print(
            "SkyController is not currently connected to {}".format(
                drone_serial))
        if drone_serial in self.visible_drones:
            print("Connecting to {}...".format(drone_serial))
            connection = self.skyctrl(
                connect(
                    serial=drone_serial, key=drone_security_key)
                >> connection_state(
                    state="connected", serial=drone_serial)
            ).wait(_timeout=10)
        elif drone_serial in self.known_drones:
            print(
                "{} is a known drone but is not currently visible".format(
                    drone_serial))
            return
        elif drone_serial is not None:
            print(
                "{} is an unknown drone and not currently visible".format(
                    drone_serial))
            return
        if connection.success():
            print("Connected to {}".format(drone_serial))
            return True
        else:
            print("Failed to connect to {}".format(drone_serial))

    def forget_drone(self, drone_serial):
        if drone_serial == self.active_drone:
            print("Forgetting {} ...".format(drone_serial))
            self.skyctrl(
                forget(serial=drone_serial)
                >> connection_state(
                    state="disconnecting", serial=drone_serial)
                ).wait(_timeout=10)
        elif drone_serial in self.known_drones:
            print("Forgetting {} ...".format(drone_serial))
            self.skyctrl(
                forget(serial=drone_serial)
                ).wait(_timeout=10)
        else:
            print("{} is an unknown drone".format(drone_serial))

    def disconnect_skyctrl(self):
        self.skyctrl.disconnect()


def controller():
    example = SkyControllerExample()
    print("@ Connection to SkyController")
    example.skyctrl_connect()
    example.update_drones()
    print("@ Connection to a drone")
    if example.connect_drone(DRONE_SERIAL, DRONE_SECURITY_KEY):
        example.update_drones()
        print("@ Forgetting a drone")
        example.forget_drone(DRONE_SERIAL)
        example.update_drones()
    print("@ Disconnection from SkyController")
    example.disconnect_skyctrl()




start = [35.7099482, 139.5230989, 1.0]
p0 = [35.7099068, 139.5231090, 1.0]
goal = [35.709901, 139.523350, 1.0]

def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()

def move_to(drone):
    assert drone(TakeOff()).wait().success()
    time.sleep(2)

    drone(moveTo(start[0],start[1],start[2],MoveTo_Orientation_mode.TO_TARGET,0.0)
        >> moveToChanged(status='DONE', _timeout=10)).wait()

    drone(extended_move_to(latitude=goal[0], longitude=goal[1], altitude=goal[2],max_horizontal_speed=0.1))
    assert drone(Landing()).wait().success()
    time.sleep(3)



def move_by(drone):
    sita=int(input())
    '''
    PARAMETERS
    d_x (float) – Wanted displacement along the front axis [m]
    d_y (float) – Wanted displacement along the right axis [m]
    d_z (float) – Wanted displacement along the down axis [m]
    d_psi (float) – Wanted rotation of heading [rad]
    max_horizontal_speed (float) – Maximum horizontal speed in m/s.
    max_vertical_speed (float) – Maximum vertical speed in m/s.
    max_yaw_rotation_speed (float) – Maximum vertical speed in degrees/s.
    rad:math.pi/2で90°回転(3.14//2)
    '''
    drone(extended_move_by(d_x=3, d_y=0, d_z=0, d_psi=0,max_horizontal_speed=0.1)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    time.sleep(10)
    sita=math.pi/3
    drone(moveBy(0, 0, 0, sita)
          >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    assert drone(Landing()).wait().success()


def test_move_sita():
    drone = olympe.Drone("192.168.42.1")
    drone.connection()
    assert drone(TakeOff()
                 >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success()
    direction=0.4207913341510827
    distance = 2.1651991
    drone(moveBy(0, 0, 0, direction)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    distance = 3.225822
    direction=1.5707962529117883
    drone(moveBy(0, 0, 0, direction)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    distance = 3.12916743
    direction=0.45521898636276587
    drone(moveBy(0, 0, 0, direction)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    distance = 3.88626398
    direction=1.0680495561570675
    drone(moveBy(0, 0, 0, direction)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    drone(moveBy(distance, 0, 0, 0)
          >> FlyingStateChanged(state="hovering", _timeout=3)).wait().success()

    drone(Landing()).wait().success()

if __name__ == "__main__":
    controller()
