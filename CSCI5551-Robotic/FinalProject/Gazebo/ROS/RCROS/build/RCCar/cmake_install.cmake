# Install script for directory: /home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/src/RCCar

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/build/RCCar/catkin_generated/installspace/RCCar.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/RCCar/cmake" TYPE FILE FILES
    "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/build/RCCar/catkin_generated/installspace/RCCarConfig.cmake"
    "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/build/RCCar/catkin_generated/installspace/RCCarConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/RCCar" TYPE FILE FILES "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/src/RCCar/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/RCCar" TYPE PROGRAM FILES "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/build/RCCar/catkin_generated/installspace/RCCar_teleop_keystroke.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/RCCar" TYPE PROGRAM FILES "/home/huankimtran/Desktop/Working/CSCI5551-Robotic/FinalProject/Gazebo/ROS/RCROS/build/RCCar/catkin_generated/installspace/RCCar_teleop_camera.py")
endif()

