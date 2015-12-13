import sys
sys.path.insert(0, "/home/poopeye/pydybot/Packages/LeapSDK/lib")
import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import itertools
import time
import pypot.dynamixel
import math

# Find the motors
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise IOError('no port found!')
# print('ports found', ports)
# print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])
Motors = dxl_io.scan(range(10))
print(Motors)
for i in Motors:
    dxl_io.set_moving_speed({i: 100})


LastxPalm = 0
LastyPalm = 0
LastzPalm = 0


class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"


def MotorSpeed(Motors, speed):
    for i in Motors:
        dxl_io.set_moving_speed({i: speed})

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    MotorSpeed(Motors, 120)
    dxl_io.set_goal_position({1: 0})
    dxl_io.set_goal_position({3: 90})
    dxl_io.set_goal_position({5: 0})
    dxl_io.set_goal_position({7: 0})
    time.sleep(3)
    HandsNum = 0
    while HandsNum != 2:
        # Have the sample listener receive events from the controller
        

        xPalm = 0
        yPalm = 0
        zPalm = 0
        for i in list(range(0,5)):
            controller.add_listener(listener)
            frame = controller.frame()
            for hand in frame.hands:
                xPalm = hand.palm_position[0] + xPalm
                yPalm = hand.palm_position[1] + yPalm
                zPalm = hand.palm_position[2] + zPalm
        xPalm = xPalm // 5
        yPalm = yPalm // 5
        zPalm = zPalm // 5

        Palm = [xPalm, yPalm, zPalm]
        LastPalm = [LastxPalm, LastyPalm, LastzPalm]
        for i in list(range(0,3)):
            diff = math.fabs(Palm[i] - LastPalm[i])
            if diff <= 5:
                Palm = LastPalm

        if Palm[0] <= -90:
            J1pos = -90
            dxl_io.set_goal_position({1: J1pos})
        elif Palm[0] >= 90:
            J1pos = 90
            dxl_io.set_goal_position({1: J1pos})
        else:
            J1pos = (-Palm[0]) // 1
            dxl_io.set_goal_position({1: J1pos})
        print(J1pos)        

        if Palm[1] <= 15:
            J2pos = 90
            dxl_io.set_goal_position({3: J2pos})
        elif Palm[1] >= 120:
            J2pos = 0
            dxl_io.set_goal_position({3: J2pos})
        else:
            J2pos = (120 - Palm[1]) // 1
            dxl_io.set_goal_position({3: J2pos})
        print(J2pos)

        if Palm[2] <= -90:
            J3pos = -90
            dxl_io.set_goal_position({5: J3pos})
        elif Palm[2] >= 90:
            J3pos = 90
            dxl_io.set_goal_position({5: J3pos})
        else:
            J3pos = Palm[2] // 1
            dxl_io.set_goal_position({5: J3pos})
        print(J3pos)

        hand = frame.hands
        HandsNum = len(frame.hands)


    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)





if __name__ == "__main__":
    main()
