#!/usr/bin/env python
#
# This code contains the getKey function, which will get a keypress for you. You'll need to write the rest of the teleoperation node yourself.
# Remember the typical ROS node setup: init the node, set up the publisher, run a loop (probably getting the key press every time you loop), then publish messages based off of that.


import sys, select, termios, tty, math
import rospy
from std_msgs.msg import Float32


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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Node initialization 
    front_pub = rospy.Publisher('/RCCar/control/front', Float32, queue_size=10)
    rear_pub = rospy.Publisher('/RCCar/control/rear', Float32, queue_size=10)
    rospy.init_node('RCCar_teleop', anonymous=False)

    # Node done intilizing print out instructions
    print_instructions()

    VEC_ACC = 0.5
    ST_ACC = 0.1

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
                front_pub.publish(ST_ACC)
            elif key == 'd':
                front_pub.publish(-ST_ACC)
            elif key == ' ':
                # Brake to 0
                rear_pub.publish(0)

        # If keypress is Crtl+C, break loop and exit.
        if key == '\x03':
            break
       
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print('Teleoperation node terminated!')