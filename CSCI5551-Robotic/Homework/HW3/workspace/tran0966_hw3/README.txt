Name        : Huan Kim Tran
X500        : tran0966
Depedencies :
- pip3 install simple_pid
Navigation Algorithm:

              ^ L2                   R2
              |  |                   |
            50mm |                   |
              |  |                   |
              - L1                   R1
              |  |                   |
            250mm|                   |
              |  | wh             wh |
              V  | eel---Robot---eel |

                   <-----183mm----->  
                 <------184mm-------->  
                 
L1, L2, R1, R2 are virtual proximity sensors. These sensors are implemented based on the data obtained from LIDAR. If none of the sensor detects obstacle, the robot keep moving forward following the shortest path to the goal. If there is an obstacle, base on which side of the robot, the sensor are on, the robot will try to move toward the location of the sensor on the other side. If both sides are blocked, the sensor choose to turn

Some visualization tool implemented on RVIZ:
- Add a MarkerArray visualizer in Rviz and listen to topic /tran0966/nav_sensors_array to see the real-time value of the proximity sensors

- Add a Pose visualizer in Rviz and listen to /tran0966/point_to_goal topic to see the goal of the robot changing as it sees obstacles 
