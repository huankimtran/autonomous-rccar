#!/usr/bin/env python3
#
# This code contains the getKey function, which will get a keypress for you. You'll need to write the rest of the teleoperation node yourself.
# Remember the typical ROS node setup: init the node, set up the publisher, run a loop (probably getting the key press every time you loop), then publish messages based off of that.


import sys, select, termios, tty, math
import rospy
from std_msgs.msg import Float32, Int32, Bool


def getKey(key_timeout):
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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Node initialization
    front_pub = rospy.Publisher('/RCCar/control/front', Int32, queue_size=10)
    rear_pub = rospy.Publisher('/RCCar/control/rear', Float32, queue_size=10)
    top_camera_pub = rospy.Publisher('/RCCar/control/top_camera', Bool, queue_size=10)
    rospy.init_node('RCCar_teleop', anonymous=False)

    # Node done intilizing print out instructions
    print_instructions()

    VEC_ACC = 0.5
    ST_ACC = 10

    steer = 0
    top_camera_capture = False

    while not rospy.is_shutdown():
        key = getKey(0.1)

        # If keypress is not empty, print out the key.
        if key != '':
            # Update movement values
            if key == 'w':
                rear_pub.publish(VEC_ACC)
            elif key == 's':
                rear_pub.publish(-VEC_ACC)
            elif key == 'a':
                steer += ST_ACC
                if steer > 100:
                    steer = 100
                front_pub.publish(steer)
            elif key == 'd':
                steer -= ST_ACC
                if steer < -100:
                    steer = -100
                front_pub.publish(steer)
            elif key == ' ':
                # Brake to 0
                rear_pub.publish(0)
            elif key == 'c':
                top_camera_capture = not top_camera_capture
                top_camera_pub.publish(top_camera_capture)

        # If keypress is Crtl+C, break loop and exit.
        if key == '\x03':
            break
       
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print('Teleoperation node terminated!')