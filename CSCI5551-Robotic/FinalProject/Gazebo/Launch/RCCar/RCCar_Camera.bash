#! /bin/bash
# Source ROS package
echo 'What map RCCarWorld_camera map? (1, 2, 3)'
read choice
source ../../ROS/RCROS/devel/setup.bash
roscore &
gazebo ../../World/RCcar/RCcarWorld_camera_$choice.xml -u --verbose &
rosrun RCCar RCCar_teleop_camera.py
