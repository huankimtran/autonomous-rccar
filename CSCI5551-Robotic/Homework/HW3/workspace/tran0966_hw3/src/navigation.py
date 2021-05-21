#!/usr/bin/env python

import sys, select, termios, tty, math, time
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, translation_matrix
import tf.transformations as tfs
import numpy as np
# TODO: find a way to force installation of this package
from simple_pid import PID

def point_subtract(a,b):
    """
    return point a - point b
    """
    rs = Point()
    rs.x = a.x - b.x
    rs.y = a.y - b.y
    rs.z = a.z - b.z
    return rs

class Goal:
    def __init__(self, pose=None, xyz=None, intermediate=False, *args,):
        """
        pose: Pose type indicating the location of the goal, pose has type Pose()
        xyz : Another way to specify the goal using a list, tuple (x,y,z)
        intermediate: Boolean flag, if True, indicating that this goal is an intermediate goal, when reached, no anouncement will be published
        """
        self.position = Point()
        self.is_inter = intermediate
        if xyz != None:
            self.position.x = xyz[0]
            self.position.y = xyz[1]
            self.position.z = xyz[2]
        elif pose != None:
            self.position.x = pose.position.x
            self.position.y = pose.position.y
            self.position.z = pose.position.z


class LaserData:
    def __init__(self, raw=None):
        """
        raw is of type sensor_msgs.msg.LaserScan
        """
        self.raw_data = raw
        self.angle_list = []
        if self.raw_data != None:
            self.regenerate_angle_list()

    def regenerate_angle_list(self):
        if self.raw_data != None:
            tmp_list = [self.raw_data.angle_min]
            for i in range(1,len(self.raw_data.ranges)):
                tmp_list.append(tmp_list[len(tmp_list)-1] + self.raw_data.angle_increment)
            self.angle_list = np.array(tmp_list)
    
    def update_raw_data(self, raw):
        """
        Laser is of type sensor_msgs.msg.LaserScan
        """
        self.raw_data = raw
        if len(self.angle_list) == 0:
            self.regenerate_angle_list()

class NavSensor:
    """
    TODO: Rewrite this for extra credit, args is dictionary contains additional data for each sensor
    this is made to be flexible so that gmapping can be implemented easily
    """
    id_count = 0
    def __init__(self, sensor_tf, args=None):
        self.tf = sensor_tf         # Transform converting a point at robot baselink to the position of the sensor
        self.state = False          # State of the sensor, True means hit something, False means not hitting anyting
        self.id = NavSensor.id_count
        self.seq = 0
        self.current_ray_length = 0
        self.STATE_RAY_LENGTH_ERROR = 5e-2
        NavSensor.id_count+=1

    def update(self, data):
        """
            data is of type LaserData or gmapping for extra credit
            TODO: Change this function to adapt gmapping
        """
        # Get sensor position
        pos = tfs.translation_from_matrix(self.tf)
        # Find angle between base_link Ox and the sensor
        angle = np.arctan2(pos[1],pos[0])
        angle = angle if angle >= 0 else (angle + 2 * math.pi)
        # Find the index of the ray this sensor is on
        index = int(angle/data.raw_data.angle_increment)
        # Get the sensor measurement
        self.current_ray_length = data.raw_data.ranges[index]
        self.state = True if self.current_ray_length <= (math.sqrt(pos[0]*pos[0] + pos[1]*pos[1]) + self.STATE_RAY_LENGTH_ERROR) else False

    def get_marker(self):
        self.seq+=1
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.header.seq = self.seq
        marker.ns = "nav_sensor"
        marker.id = self.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.tf[0,3]
        marker.pose.position.y = self.tf[1,3]
        marker.pose.position.z = self.tf[2,3]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0 if self.state else 0.0
        marker.color.g = 0.0 if self.state else 1.0
        marker.color.b = 0.0
        return marker
    
    def sensor_in_world_frame(self, world_frame_pos, world_frame_orien):
        """
        Take in a pose, describe the base_link frame in the world frame
        calculate and return a Pose() containing the position of the sensor in the world frame
        """
        world_T_base = tfs.quaternion_matrix([world_frame_orien.x , world_frame_orien.y, world_frame_orien.z, world_frame_orien.w])
        world_T_base[:3,3] = [world_frame_pos.x, world_frame_pos.y, world_frame_pos.z]
        sensor_in_world = np.dot(world_T_base, self.tf)
        sensor_in_world_orien = tfs.quaternion_from_matrix(sensor_in_world)
        sensor_in_world_position = tfs.translation_from_matrix(sensor_in_world)
        sensor_in_world_pose = Pose()
        sensor_in_world_pose.orientation.x = sensor_in_world_orien[0]
        sensor_in_world_pose.orientation.y = sensor_in_world_orien[1]
        sensor_in_world_pose.orientation.z = sensor_in_world_orien[2]
        sensor_in_world_pose.orientation.w = sensor_in_world_orien[3]
        sensor_in_world_pose.position.x = sensor_in_world_position[0]
        sensor_in_world_pose.position.y = sensor_in_world_position[1]
        sensor_in_world_pose.position.z = sensor_in_world_position[2]
        return sensor_in_world_pose

class NavSensorModule:
    def __init__(self):
        """ 
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
            x
            ^
            |
        y<--O is at center of the "Robot" above, coaxis with the wheel

        Burger dimension getting from 
        https://emanual.robotis.com/assets/images/platform/turtlebot3/hardware_setup/turtlebot3_dimension1.png
        """
        # List of sensor
        self.sensors = dict()
        self.sensors['R1'] = NavSensor(sensor_tf = tfs.translation_matrix([0.250, -0.092, 0.0]))
        self.sensors['R2'] = NavSensor(sensor_tf = tfs.translation_matrix([0.300, -0.092, 0.0]))
        self.sensors['L1'] = NavSensor(sensor_tf = tfs.translation_matrix([0.250, 0.092, 0.0]))
        self.sensors['L2'] = NavSensor(sensor_tf = tfs.translation_matrix([0.300, 0.092, 0.0]))
        self.APPROACHABLE_THRESHOLD = self.sensors['R2'].tf[0,3]
        # Publisher to visualize
        self.marker_pub = rospy.Publisher('tran0966/nav_sensors_array', MarkerArray)
    
    def update_sensor(self, data):
        """
            data is of type LaserData or gmapping for extra credit
        """
        # Give sensor new data and let them update their states
        for k in self.sensors:
            self.sensors[k].update(data)
        # After update, broadcast the state of the sensor to rviv to visualize
        self.broadcast_rviz_sensor_state()
    
    def broadcast_rviz_sensor_state(self):
        mk_array = MarkerArray()
        for k in self.sensors:
            mk_array.markers.append(self.sensors[k].get_marker())
        self.marker_pub.publish(mk_array)
    
    def __str__(self):
        debug_value = '\n'
        for k in self.sensors:
            debug_value += f'{k}:{self.sensors[k].state}, {self.sensors[k].current_ray_length}\n'
        return debug_value

    def __getitem__(self, item):
        return self.sensors[item]

def angle_between_vector(a,b):
    """
    The angle between two vector a and b
    going from a to b
    """
    return

def vector_length2d(v):
    return math.sqrt(v.x*v.x + v.y*v.y)

def angle_between_vector2d(a,b):
    return math.acos((a.x  * b.x + a.y  * b.y)/math.sqrt((a.x * a.x + a.y * a.y)*(b.x * b.x + b.y * b.y)))

def get_orientation_from_quarternion(q):
    rot_matrix = quaternion_matrix([q.x, q.y, q.z, q.w])[:-1,:-1]
    a = rot_matrix.dot(np.array([0,1,0]).reshape((-1,1)))           # This uses Oy, do not change
    p = Point()
    p.x = a[0]
    p.y = a[1]
    return p

class robotConsciousness:
    def __init__(self):
        # Location and motion
        self.orientation = Point()
        self.position = Point()
        self.motion = Twist()
        self.laser = LaserData()        # Laser scan handler will place a EulerLaser() here 
        self.goal = []                # Remember to set the to None when there is no goal
        self.loc_moc_has_been_read = True
        # Controller
        self.pid_vec_x = None
        self.pid_rot_z = None
        self.MAX_ROT_ERROR = 1e-1
        self.MAX_VEC_ERROR = 3e-2
        # Sensor Module
        self.nav_sensors = NavSensorModule()
        # Publisher
        # Register to publish cmd_vel to control motion
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Register to publish point_to_goal to show intention to goal
        self.intent_pub = rospy.Publisher('tran0966/point_to_goal', PoseStamped, queue_size=10)
        self.point_to_goal_init = PoseStamped()
        self.point_to_goal_init.header.frame_id = 'odom'
        self.point_to_goal_init.header.seq = 0
        # Serivces
        rospy.wait_for_service('/csci_5551/goal_reached')
        self.goal_reach_service = rospy.ServiceProxy('/csci_5551/goal_reached', Trigger)

    def update_location_motion(self, position, orientation, motion):
        # Update orientation
        self.orientation = orientation
        # Update location
        self.position = position
        # Update motion
        self.motion = motion
        # Mark as new
        self.loc_moc_has_been_read = False 
    
    def adjust_motion(self):
        if self.loc_moc_has_been_read:
            # Only process when there is new value
            return
        # Mark read
        self.loc_moc_has_been_read = True
        if len(self.goal) == 0:
            # Do not attempt to move when there is no goal
            return
        current_goal = self.get_current_goal()
        # Get new controlling values
        new_control = Twist()
        # First compute the goal orientation for the robot
        goal_orien = point_subtract(current_goal.position, self.position)
        self.publish_intention(goal_orien)
        # Compute the difference between current orientation and the goal (both angle and length)
        tmp = get_orientation_from_quarternion(self.orientation)
        diff_angle_qua = angle_between_vector2d(tmp, goal_orien) - math.pi/2
        dist_to_goal = vector_length2d(goal_orien)
        # Try to control the robot's orientation first
        new_control.angular.z = self.pid_rot_z(diff_angle_qua)
        if math.fabs(diff_angle_qua) >= self.MAX_ROT_ERROR:
            # Do not move forward if orientation does not line up in the same direction or when orientation is more than 180 degree apart (opposite)
            new_control.linear.x = 0
        else:
            if dist_to_goal <= self.nav_sensors.APPROACHABLE_THRESHOLD and not self.get_current_goal().is_inter:
                # If final goal is close enough turn off obstacle avoidance, go forward, even sensor will ring 
                if dist_to_goal <= self.MAX_VEC_ERROR:
                    # Stop when the goal reached
                    self.taget_reached_handler()
                else:
                    # Otherwise let speed be proportional to the distance to goal
                    # Move forward if orientation line up
                    new_control.linear.x = self.pid_vec_x(-dist_to_goal)
            # Otherwise object avoidance starts
            elif self.nav_sensors['L1'].state and self.nav_sensors['L2'].state:
                if self.get_current_goal().is_inter:
                    print('Obstacle detected, adding intermediate goal R2 to avoid')
                    # Replace current goal with a new intermediate goal
                    self.replace_current_goal(Goal(pose=self.nav_sensors['R2'].sensor_in_world_frame(self.position, self.orientation), intermediate=True))
                else:
                    self.add_goal(Goal(pose=self.nav_sensors['R2'].sensor_in_world_frame(self.position, self.orientation), intermediate=True))
            elif self.nav_sensors['R1'].state and self.nav_sensors['R2'].state:
                if self.get_current_goal().is_inter:
                    print('Obstacle detected, adding intermediate goal L2 to avoid')
                    # Replace current goal with a new intermediate goal
                    self.replace_current_goal(Goal(pose=self.nav_sensors['L2'].sensor_in_world_frame(self.position, self.orientation), intermediate=True))
                else:
                    self.add_goal(Goal(pose=self.nav_sensors['L2'].sensor_in_world_frame(self.position, self.orientation), intermediate=True))
            elif dist_to_goal <= self.MAX_VEC_ERROR:
                # Stop when the goal reached
                self.taget_reached_handler()
            else:
                # Otherwise let speed be proportional to the distance to goal
                # Move forward if orientation line up
                new_control.linear.x = self.pid_vec_x(-dist_to_goal)
        print(f'\nRobot z {self.orientation.z}\nGoal z {np.arctan2(goal_orien.y, goal_orien.x)}\nDiff angle {diff_angle_qua}\nDistance {dist_to_goal}')
        # Publish the new motion
        self.cmd_pub.publish(new_control)
    
    def taget_reached_handler(self):
        current_goal = self.get_current_goal()
        # Turning off all motion
        new_control = Twist()
        new_control.linear.x = 0
        new_control.angular.z = 0
        self.cmd_pub.publish(new_control)
        if current_goal.is_inter:
            # Intermediate goal reached!
            # TODO: Clean up this print?
            print('Intermediate goal target reached!')
        else:
            # Final goal reach!
            print('Final goal reached!')
            got_it = self.goal_reach_service(TriggerRequest())
            if got_it.success:
                print('Trigger sent and received successfully!')
        self.goal.pop(0)
    
    def publish_intention(self, point_to_goal):
        rs = self.point_to_goal_init
        # boiler 
        rs.header.seq = rs.header.seq + 1 
        rs.header.stamp = rospy.Time.now()
        # important value
        rs.pose.position = self.position
        orien_to_quar = quaternion_from_euler(0, 0, np.arctan2(point_to_goal.y, point_to_goal.x))
        rs.pose.orientation.x = orien_to_quar[0]
        rs.pose.orientation.y = orien_to_quar[1]
        rs.pose.orientation.z = orien_to_quar[2]
        rs.pose.orientation.w = orien_to_quar[3]
        self.intent_pub.publish(rs)
    
    def get_current_goal(self):
        return self.goal[0]
    

    def add_goal(self, goal):
        """
        Goal is of type Goal()
        """
        self.goal.insert(0, goal)
        self.pid_vec_x = PID(1, 0, 0, setpoint = 0, sample_time=None, output_limits=(-0.2, 0.2))
        self.pid_rot_z = PID(7, 0.001, 1, setpoint = 0, sample_time=None, output_limits=(-1.5, 1.5))
        print(f'New goal set {self.get_current_goal()}')
    
    def replace_current_goal(self, goal):
        self.goal.pop(0)
        self.add_goal(goal)

    def update_laser_data(self, data):
        """
        data is of type sensor_msgs.msg.LaserScan
        """
        self.laser.update_raw_data(data)

    def __str__(self):
        debug_value = f"""\n
        Position x,y,z: {self.position.x}, {self.position.y}, {self.position.z}
        Oerientation x,y,z: {self.orientation.x}, {self.orientation.y}, {self.orientation.z}
        """
        return debug_value

# Global vars
robot = None

def getEuler(orientation):
    (x_roll, y_pitch, z_yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    toPoint = Point()
    toPoint.x = x_roll
    toPoint.y = y_pitch
    toPoint.z = z_yaw
    return toPoint

def new_location_update_odom_handler(odom):
    global robot
    if robot == None:
        robot = robotConsciousness()
    # Update robot knowledgge about its whereabout
    robot.update_location_motion(
        odom.pose.pose.position,
        odom.pose.pose.orientation,
        odom.twist.twist
    )
    # Calculate the next move robot needs to takes
    robot.adjust_motion()


def laser_scan_feedback_handler(data):
    global robot
    if robot != None:
        # Update laser data
        robot.update_laser_data(data)
        # Then update nav sensors state
        robot.nav_sensors.update_sensor(robot.laser)
        
def goal_listener(data):
    global robot
    if robot != None:
        if len(robot.goal) == 0:
            robot.add_goal(Goal(pose=data.pose, intermediate=False))

if __name__=="__main__":
    # Creat navigation node
    rospy.init_node('navigation', anonymous=False)
    # Subscribe /odo meter topic
    rospy.Subscriber('odom', Odometry, new_location_update_odom_handler)
    # Subscribe /scan topic
    rospy.Subscriber('scan', LaserScan, laser_scan_feedback_handler)
    # Subscrive for goal topic
    rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_listener)
    # Obtain proxy for test note standi service
    rospy.wait_for_service('/csci_5551/goal_reached')
    # Create the conscousness for the robot
    robot = robotConsciousness()
    # Navigate
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print(robot.nav_sensors)
        r.sleep()
