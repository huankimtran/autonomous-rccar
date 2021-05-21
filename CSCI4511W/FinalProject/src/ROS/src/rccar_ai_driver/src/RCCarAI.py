# /RCCar/control/front takes an integer (backend allow double) indicating the steering angle level (between -100 and 100)
# /RCCar/control/rear takes an integer (backend allow double) indicating the throttling level (between -100 and 100)

import pathlib as PL
import keras
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Bool
import cv2
import os
import sys

# Importing architecture from remote AI python package, this path change is temporarily for this python session only
AI_MODULE_FOLDER = PL.Path(__file__).absolute().parents[4].joinpath('AI').absolute()
sys.path.append(str(AI_MODULE_FOLDER))
from Architecture import Regress32x128Color00PastVer0 as Network

camera_view = None
driver = None
bridge = CvBridge()

class RCCarConfig:
    def __init__(self):
        self.FRONT_WHEEL_CONTROL_TOPIC_NAME = '/RCCar/control/front'        
        self.REAR_WHEEL_CONTROL_TOPIC_NAME = '/RCCar/control/rear'
        self.CAMERA_CONTROL_TOPIC_NAME = '/RCCar/control/top_camera'
        self.CAMERA_IMG_TOPIC_NAME = '/RCCar/broadcast/top_camera/'
        # Change this to the directory of your network's saved weight (do not add / before the network_save)
        self.NETWORK_MODEL_SAVED_DIR = 'network_save/Regress32x128Color00PastVer1_2021_05_04-22_44_04'

class RCCarController:
    def __init__(self, steer_acceleration=10, throttle_acceleration=0.5):
        """
            steer_acceleration how many level will be added per key press to steer level
            throttle_acceleration how many level will be added per key press to throtle level
        """
        # Configure
        self.cfg = RCCarConfig()
        # Internal state
        self.steer_acc = steer_acceleration
        self.throttle_acc = throttle_acceleration
        self.steer_level = 0                # At anytime the value of this variable must be [-100, 100]
        self.throttle_level = 0             # At anytime the value of this variable must be [-100, 100]
        self.recorder_camera_status = False # The state of the top camera, off at first
        # Publisher to control the RC Car
        self.steer_controller = rospy.Publisher(self.cfg.FRONT_WHEEL_CONTROL_TOPIC_NAME, Float32, queue_size=10)
        self.throttle_controller = rospy.Publisher(self.cfg.REAR_WHEEL_CONTROL_TOPIC_NAME, Float32, queue_size=10)
        self.recorder_controller = rospy.Publisher(self.cfg.CAMERA_CONTROL_TOPIC_NAME, Bool, queue_size=10)

def on_new_img_arrive(data):
    global camera_view
    global driver
    global bridge
    # Get the data and convert to opencv format
    try:
        # Save the newly published img so that the while loop in clien_node can display it
        # Also convert from the ROS Image type to OpenCV image type and rescale to be usable
        # In the network
        camera_view = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)
    # If driver is set up then try to predict and control the car
    if driver:
        # Get predict value from the network
        values = driver.ai.predict(np.array([camera_view]))
        # Decode 
        steer, speed = values[0].tolist()
        # Send the values to control the car
        driver.rc_controller.steer_controller.publish(steer)
        driver.rc_controller.throttle_controller.publish(speed)
        # print out to see value
        print(f'Steer {steer}%, Speed {speed}%')

class AIDriver:
    def __init__(self):
        self.rc_config = RCCarConfig()
        self.rc_controller = RCCarController()
        self.ai = Network(weight_folder=str(AI_MODULE_FOLDER.joinpath(self.rc_config.NETWORK_MODEL_SAVED_DIR)))
        self.img_sub = rospy.Subscriber(
            name=self.rc_config.CAMERA_IMG_TOPIC_NAME,
            data_class=Image,
            callback=on_new_img_arrive)
        print('----------------READY----------------')

if __name__=="__main__":
    # Node initialization
    rospy.init_node('RCCarAI', anonymous=False)
    driver = AIDriver()
    # Constantly showing the received image
    while not rospy.is_shutdown():
        # Keep showing the new image if there is any changes
        if camera_view is not None:
            cv2.imshow('Viewport scene', camera_view)
            cv2.waitKey(1)
