import sys
sys.path.insert(0, "/home/poopeye/pydybot/LeapSDK/lib")
import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
import itertools
import time
import pypot.dynamixel


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
        return len(frame.hands)


        

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"


def main():
    # Find the motors
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')
    print('ports found', ports)
    print('connecting on the first available port:', ports[0])
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    Motors = dxl_io.scan(range(10))
    print(Motors)
    for i in Motors:
        dxl_io.set_moving_speed({i: 100})

    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    l = 0
    while l != 2:
        # Have the sample listener receive events from the controller
        controller.add_listener(listener)
    
        frame = controller.frame()
        l = len(frame.hands)
        print(l)

        if l == 0:
            dxl_io.set_goal_position({1: 0})
            dxl_io.set_goal_position({3: 0})
            dxl_io.set_goal_position({5: 0})
            dxl_io.set_goal_position({7: 0})
        else:
            dxl_io.set_goal_position({1: 0})
            dxl_io.set_goal_position({3: 90})
            dxl_io.set_goal_position({5: 90})
            dxl_io.set_goal_position({7: 0})


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
