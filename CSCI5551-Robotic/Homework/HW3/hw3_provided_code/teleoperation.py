#!/usr/bin/env python
#
# This code contains the getKey function, which will get a keypress for you. You'll need to write the rest of the teleoperation node yourself.
# Remember the typical ROS node setup: init the node, set up the publisher, run a loop (probably getting the key press every time you loop), then publish messages based off of that.


import sys, select, termios, tty


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    while(1):
        key = getKey(0.1)
        # If keypress is not empty, print out the key.
        if key != '':
            print(key)

        # If keypress is Crtl+C, break loop and exit.
        if key == '\x03':
            break
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
