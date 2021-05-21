cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# For ros
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# For gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Library
find_package( catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
)
find_package( OpenCV REQUIRED)
# For the rc_model_push plugin
add_library(rc_model_push SHARED rc_forward_push.cc)
target_link_libraries(rc_model_push ${GAZEBO_LIBRARIES})

# For the rc_rear_wheel_rolling plugin
add_library(rc_rear_wheel_rolling SHARED rc_rear_wheel_rolling.cc)
target_link_libraries(rc_rear_wheel_rolling ${GAZEBO_LIBRARIES})

# For the rc_key_stroke_listen
add_library(rc_key_stroke_listen SHARED rc_key_stroke_listen.cc)
target_link_libraries(rc_key_stroke_listen ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# For the rc_teleop_camera
add_library(rc_teleop_camera SHARED rc_teleop_camera.cc)
target_link_libraries(rc_teleop_camera ${OpenCV_LIBS} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})