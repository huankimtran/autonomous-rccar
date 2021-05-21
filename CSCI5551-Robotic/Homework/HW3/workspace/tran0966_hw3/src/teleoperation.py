#!/usr/bin/env python
#
# This code contains the getKey function, which will get a keypress for you. You'll need to write the rest of the teleoperation node yourself.
# Remember the typical ROS node setup: init the node, set up the publisher, run a loop (probably getting the key press every time you loop), then publish messages based off of that.


import sys, select, termios, tty, math
import rospy
from geometry_msgs.msg import Twist


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
    print('x - emergency break')

def get_twist_msg(vec, rot):
    msg = Twist()
    msg.linear.x = vec
    msg.angular.z = rot
    return msg

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Node initialization 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleoperation', anonymous=False)

    # Movement parameter init
    MAX_VEC = 0.5
    MAX_ROT = 2.8
    VEC_ACC = 0.05
    ROT_ACC = 0.7
    VEC_DEC_ACC = 0.025
    ROT_DEC_ACC = 0.5
    vec = 0
    rot = 0

    # Publish first message to reset everything to zero
    pub.publish(get_twist_msg(vec, rot))

    # Node done intilizing print out instructions
    print_instructions()

    while not rospy.is_shutdown():
        key = getKey(0.1)

        # If keypress is not empty, print out the key.
        if key != '':
            # Update movement values
            if key == 'w':
                vec += VEC_ACC
            elif key == 's':
                vec -= VEC_ACC
            elif key == 'a':
                rot += ROT_ACC
            elif key == 'd':
                rot -= ROT_ACC
            elif key == 'x':
                vec = 0
                rot = 0
            # Limit the value in a specific range
            vec = vec if math.fabs(vec) <= MAX_VEC else math.copysign(MAX_VEC, vec)
            rot = rot if math.fabs(rot) <= MAX_ROT else math.copysign(MAX_ROT, rot)
            print('Velocity ',vec, ', rotation ',rot)
            
        # Decay
        vec = math.copysign(math.fabs(vec) - VEC_DEC_ACC if math.fabs(vec) > VEC_DEC_ACC else 0, vec)
        rot = math.copysign(math.fabs(rot) - ROT_DEC_ACC if math.fabs(rot) > ROT_DEC_ACC else 0, rot)

        # Publish the new movement values
        pub.publish(get_twist_msg(vec, rot))

        # If keypress is Crtl+C, break loop and exit.
        if key == '\x03':
            break
       
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print('Teleoperation node terminated!')