#! /bin/bash
# Source ROS package
source ../../ROS/RCROS/devel/setup.bash
roscore &
gazebo ../../World/RCcar/RCcarWorld_key_listen.xml&
rosrun RCCar RCCar_teleop_keystroke.py