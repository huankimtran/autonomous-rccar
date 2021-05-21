#!/usr/bin/env python
#
# /RCCar/control/front takes an integer (backend allow double) indicating the steering angle level (between -100 and 100)
# /RCCar/control/rear takes an integer (backend allow double) indicating the throttling level (between -100 and 100)

import sys, select, termios, tty, math
import rospy
from std_msgs.msg import Float32, Int32, Bool


class RCCarController:
    def __init__(self, steer_acceleration=10, throttle_acceleration=0.5):
        """
            steer_acceleration how many level will be added per key press to steer level
            throttle_acceleration how many level will be added per key press to throtle level
        """
        # Internal state
        self.steer_acc = steer_acceleration
        self.throttle_acc = throttle_acceleration
        self.steer_level = 0                # At anytime the value of this variable must be [-100, 100]
        self.throttle_level = 0             # At anytime the value of this variable must be [-100, 100]
        self.recorder_camera_status = False # The state of the top camera, off at first
        # Publisher to control the RC Car
        self.steer_controller = rospy.Publisher('/RCCar/control/front', Float32, queue_size=10)
        self.throttle_controller = rospy.Publisher('/RCCar/control/rear', Float32, queue_size=10)
        self.recorder_controller = rospy.Publisher('/RCCar/control/top_camera', Bool, queue_size=10)

    def turn_right(self):
        self.steer_level = max(-100, self.steer_level - self.steer_acc)
        self.steer_controller.publish(self.steer_level)

    def turn_left(self):
        self.steer_level = min(100, self.steer_level + self.steer_acc)
        self.steer_controller.publish(self.steer_level)
    
    def speed_up(self):
        self.throttle_level = min(100, self.throttle_level + self.throttle_acc)
        self.throttle_controller.publish(self.throttle_level)

    def brake(self):
        self.throttle_level = 0.0
        self.throttle_controller.publish(self.throttle_level)

    def slow_down(self):
        self.throttle_level = max(-100, self.throttle_level - self.throttle_acc)
        self.throttle_controller.publish(self.throttle_level)

    def toggle_camera(self):
        self.recorder_camera_status = not self.recorder_camera_status
        self.recorder_controller.publish(self.recorder_camera_status)


def getKey(key_timeout):
    """
        Wait for a key in the given time, return the key, otherwise rturn timeout
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_instructions():
    print('w - forward')
    print('s - backward')
    print('a - counter-clockwise')
    print('d - clockwise')
    print('Space - emergency break')
    print('c - toggle capture on/off')

def key_press_handler(controller):
    # Check if user has pressed a key
    key = getKey(0.1)
    # If keypress is not empty, print out the key.
    if key != '':
        # If keypress is Crtl+C, break loop and exit.
        if key == '\x03':
            return True
        # Update movement values
        elif key == 'w':
            controller.speed_up()
        elif key == 's':
            controller.slow_down()
        elif key == 'a':
            controller.turn_left()
        elif key == 'd':
            controller.turn_right()
        elif key == ' ':
            # Brake to 0
            controller.brake()
        elif key == 'c':
            controller.toggle_camera()
    return False

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Node initialization
    rospy.init_node('RCCar_teleop', anonymous=False)
    # Controller to control the car
    controller = RCCarController()

    # Node done intilizing print out instructions
    print_instructions()

    while not rospy.is_shutdown():
        if key_press_handler(controller):
            break
       
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print('Teleoperation node terminated!')