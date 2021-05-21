# Description
- Currently developing with Gazebo 9, might try to upgrade to gazebo 11, but the cmake seems to be troublsome
- Using ros noetic, but may be other version could work too (Melodic or Kinetic)
# How to build the plugins
- Go to any of the folders here
- Create folder named build inside that folder
- cd into folder build
- run cmake ../
- then make
- The plugins in folder testing were made by following 
- http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin
- http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin
- Remember to add the directory of the the plugin's build folder you use to the PATH (or GAZEBO_PLUGIN_PATH ?) of the terminal you use to launch the model using these plugings, otherwise, the model will not run. There will be no error or warning either
- Add the your env something like
- export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH}:/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/Plugins/Testing/build"