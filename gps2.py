import math
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, moveTo, Circle, PCMD
from olympe.messages.ardrone3.PilotingState import moveToChanged, FlyingStateChanged, PositionChanged, AttitudeChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode

drone = olympe.Drone("192.168.42.1")
drone.connection()

# Take-off
drone(
    FlyingStateChanged(state="hovering", _policy="check")
    | FlyingStateChanged(state="flying", _policy="check")
    | (
        GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
        >> (
            TakeOff(_no_expect=True)
            & FlyingStateChanged(
                state="hovering", _timeout=10, _policy="check_wait")
        )
    )
).wait()

# Get the home position
drone_location = drone.get_state(GpsLocationChanged)

# Move 10m
drone(
    moveBy(1, 0, 0, math.pi)
    >> PCMD(1, 0, 0, 0, 0, 0)
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait().success()
# Go back home
drone(
    moveTo(drone_location["latitude"],  drone_location["longitude"], drone_location["altitude"], MoveTo_Orientation_mode.TO_TARGET, 0.0)
    >> FlyingStateChanged(state="hovering", _timeout=5)
    >> moveToChanged(latitude=drone_location["latitude"], longitude=drone_location["longitude"], altitude=drone_location["altitude"], orientation_mode=MoveTo_Orientation_mode.TO_TARGET, status='DONE', _policy='wait')
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()

# Landing
drone(
    Landing()
    >> FlyingStateChanged(state="landed", _timeout=5)
).wait()

drone.disconnection()