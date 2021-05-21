#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/spinner.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include <thread>
#include <cmath>
#include <sys/stat.h>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#define IMG_FPS 5    // 10 frame per second
#define FRONT_WHEEL_STEER_LIMIT (M_PI*30.0/180.0)    // 30 degree in radian
#define ZERO_THRESHOLD -0.0001

/**
 * A plugin made based on rc_rear_wheel_rolling plugin
 * With extension to ros so user can control the rolling speed through ros topic
 * 
 * To run this, first build this folder by
 * create a folder name build, in the folder containing this file
 * cd build
 * cmake ../
 * make rc_key_stroke_listen
 * 
 * Then launch the world by
 * ../../Launch/RCCar/RCCar_key_stroke_listen.bash
 * 
 * Ros part explaination here by http://gazebosim.org/tutorials?tut=guided_i6
 * FYI - why do you need the thread? Because ros node needs computing power. You give ros node process power by through creating a thread
 * and process whatever ros node need to do in that thread
 * FYI - When is OnRosMsg callback called? From my understanding, whenver there is a message sent to the topic '.../vel_cmd', the 
 * rosmaster will create an instace of the OnRosMsg callback with the content of the message as the argument for the callback
 * and then send that callback instance to a queue. Ros node, when it get to run, will enqueue these callback instances one by one
 * and execute them by calling callAvailable. Actually, callAvailable will do all the enqueu and then execute the callback instance.
 */

namespace gazebo
{
  void toImageMsg(cv::Mat image, sensor_msgs::Image& ros_image)
  {
    ros_image.header = std_msgs::Header();
    ros_image.height = image.rows;
    ros_image.width = image.cols;
    ros_image.encoding = "bgr8";
    ros_image.is_bigendian = false;
    ros_image.step = image.cols * image.elemSize();
    size_t size = ros_image.step * image.rows;
    ros_image.data.resize(size);

    if (image.isContinuous())
    {
      memcpy((char*)(&ros_image.data[0]), image.data, size);
    }
    else
    {
      // Copy by row by row
      uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
      uchar* cv_data_ptr = image.data;
      for (int i = 0; i < image.rows; ++i)
      {
        memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
        ros_data_ptr += ros_image.step;
        cv_data_ptr += image.step;
      }
    }
  }

  class RCTeleopCamera : public ModelPlugin
  {

    // Gazebo stuffs
    // Pointer to the model, joints ,and sensor manager
    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr rLeft, rRight, fLeft, fRight;

    private: physics::WorldPtr world;

    private: sensors::CameraSensorPtr camera;
    
    private: transport::NodePtr gNode;
    private: transport::SubscriberPtr gSub;

    // Pointer to the update event connection
    private: physics::JointControllerPtr jointController;

    // Concurrent parameter for the joints
    private: double fTargetSteerAngle, rTargetVelocity, fSteerLimit, fSteerIncreament; 
    private: double fTargetSteerAngleLevel, rTargetVelocityLevel;
    
    // ROS stuffs

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSubRearWheelSpeed;
    private: ros::Subscriber rosSubFrontWheelAngle;
    private: ros::Subscriber rosSubTopCameraCapture;

    /// \brief ROS publisher used for publishing gazebo img to ros topic
    private: ros::Publisher rosPubCamera; 

    /// \brief A thread the keeps running the rosQueue
    private: ros::AsyncSpinner* spinner;

    // Ros timer to capture image
    // private: ros::Timer imgCaptureTimer;

    // msc.
    private: std::string imgSavePath;
    private: unsigned imgCounter;
    private: bool enableImageCapture, newImageAvailable;

    public:~RCTeleopCamera()
    {
      delete this->spinner;
    }

    public: static double throttleLevel2Speed(double throttle) {
      int vlSign;
      // No power, 0
      if(fabs(throttle) <= ZERO_THRESHOLD) {
        return 0.0;
      } else if (throttle > ZERO_THRESHOLD) {
        vlSign = 1;
      } else {
        vlSign = -1;
      }
      // Constraining on top limit
      if(throttle > 100) {
        throttle = 100.0;
      } else if( throttle < -100) {
        throttle = -100.0;
      }
      return vlSign * 45 * pow(1.03, fabs(throttle) - 100);
    }

    // Currently not used
    public: static double steerLevel2Angle(double steer) {
      int vlSign;
      // No power, 0
      if(fabs(steer) <= ZERO_THRESHOLD) {
        return 0.0;
      } else if (steer > ZERO_THRESHOLD) {
        vlSign = 1;
      } else {
        vlSign = -1;
      }
      // Constraining on top limit
      if(steer > 100) {
        steer = 100.0;
      } else if( steer < -100) {
        steer = -100.0;
      }
      return vlSign * 45 * pow(1.03, fabs(steer) - 100);
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Get the world
      this->world = this->model->GetWorld();

      // Get the sensor manager to get the camera attached to the model
      sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(this->model->SensorScopedName("top_camera")[0]);
      this->camera = std::static_pointer_cast<sensors::CameraSensor>(sensor);
      // Create a folder to save images
      time_t t = time(0);
      struct tm* now = gmtime(&t);
      std::ostringstream s;
      s << "./" << "Record_" << now->tm_mon + 1 << "_" << now->tm_mday << now->tm_hour << "_"  << now->tm_min <<"/";
      this->imgSavePath = s.str();
      // Create folder to save image
      mkdir(this->imgSavePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      this->imgCounter = 0;

      // Node for listen to image
      this->gNode.reset(new transport::Node());
      this->gNode->Init();
      this->gSub = this->gNode->Subscribe(this->camera->Topic(), &RCTeleopCamera::GazeboTopCameraMsgHandler, this);

      // Getting the joints of two rear wheels
      this->rLeft = this->model->GetJoint("REVOLUTE_CHASSIS_LEFT_REAR_WHEEL");
      this->rRight = this->model->GetJoint("REVOLUTE_CHASSIS_RIGHT_REAR_WHEEL");

      // Getting the joints of two front wheels
      this->fLeft = this->model->GetJoint("REVOLUTE_CHASSIS_LEFT_FRONT_WHEEL");
      this->fRight = this->model->GetJoint("REVOLUTE_CHASSIS_RIGHT_FRONT_WHEEL");

      // Setting initial value
      this->fTargetSteerAngle = 0.0;
      this->rTargetVelocity = 0.0;
      this->rTargetVelocityLevel = 0;

      // Using PID on the front wheel to set the position, setPosition does not work
      double fPID_P, fPID_I, fPID_D;
      fPID_P = 100;
      fPID_I = 0;
      fPID_D = 5;
      this->jointController.reset(new physics::JointController(this->model));
      // Position pid for front wheels
      this->jointController->AddJoint(this->fLeft);
      this->jointController->SetPositionPID(this->fLeft->GetScopedName(), common::PID(fPID_P, fPID_I, fPID_D));
      this->jointController->SetPositionTarget(this->fLeft->GetScopedName(), this->fTargetSteerAngle);
      this->jointController->AddJoint(this->fRight);
      this->jointController->SetPositionPID(this->fRight->GetScopedName(), common::PID(fPID_P, fPID_I, fPID_D));
      this->jointController->SetPositionTarget(this->fRight->GetScopedName(), this->fTargetSteerAngle);

      // Using PID on the rear wheel to set PID
      double rPID_P, rPID_I, rPID_D;
      rPID_P = 2;
      rPID_I = 0;
      rPID_D = 0;
      // Velocity pid for rear wheels
      this->jointController->AddJoint(this->rLeft);
      this->jointController->SetVelocityPID(this->rLeft->GetScopedName(), common::PID(rPID_P, rPID_I, rPID_D));
      this->jointController->SetVelocityTarget(this->rLeft->GetScopedName(), rTargetVelocity);
      this->jointController->AddJoint(this->rRight);
      this->jointController->SetVelocityPID(this->rRight->GetScopedName(), common::PID(rPID_P, rPID_I, rPID_D));
      this->jointController->SetVelocityTarget(this->rRight->GetScopedName(), rTargetVelocity);

      // Setting joint limit
      this->fSteerLimit = FRONT_WHEEL_STEER_LIMIT;
      this->fSteerIncreament = this->fSteerLimit/100;
      this->fLeft->SetLowerLimit(0, -this->fSteerLimit);
      this->fLeft->SetUpperLimit(0, this->fSteerLimit);
      this->fRight->SetLowerLimit(0, -this->fSteerLimit);
      this->fRight->SetUpperLimit(0, this->fSteerLimit);

      // Config joints, without this, setting velocity won't work
      // However, if we set this, pid control won't work, so let comment them out
      // this->fLeft->SetParam("fmax", 0, 10000.0);
      // this->fRight->SetParam("fmax", 0, 10000.0);
      // this->rLeft->SetParam("fmax", 0, 10000.0);
      // this->rRight->SetParam("fmax", 0, 10000.0);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RCTeleopCamera::OnUpdate, this));

      // Using ROS for communication and as a C++ to Python bridge
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "rc_gazebo_bridge", ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      /// Initialize node namespace
      this->rosNode.reset(new ros::NodeHandle("rc_gazebo_bridge"));

      // Set timer to capture image periodically
      // this->imgCaptureTimer = this->rosNode->createTimer(ros::Duration(1.0/IMG_FPS), &RCTeleopCamera::captureImage,this);
      this->enableImageCapture = false;
      this->newImageAvailable = false;


      // Create a named topic, and subscribe to it.

      // One to control the rear wheel speed
      ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/control/rear",
            1,
            boost::bind(&RCTeleopCamera::OnRearWheelCmdMsg, this, _1),
            ros::VoidPtr(), ros::getGlobalCallbackQueue());
      this->rosSubRearWheelSpeed = this->rosNode->subscribe(so1);

      // One to control the front wheel steering angle
      ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/control/front",
            1,
            boost::bind(&RCTeleopCamera::OnFrontWheelCmdMsg, this, _1),
            ros::VoidPtr(), ros::getGlobalCallbackQueue());
      this->rosSubFrontWheelAngle = this->rosNode->subscribe(so2);

      // One to toggle top camera capture on/off
      ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Bool>(
            "/" + this->model->GetName() + "/control/top_camera",
            1,
            boost::bind(&RCTeleopCamera::OnTopCameraCmdMsg, this, _1),
            ros::VoidPtr(), ros::getGlobalCallbackQueue());
      this->rosSubTopCameraCapture = this->rosNode->subscribe(so3);

      // One publisher to publish the images received to ROS topic
      this->rosPubCamera = this->rosNode->advertise<sensor_msgs::Image>(
        "/" + this->model->GetName() + "/broadcast/top_camera/", 
        10);

      // Spin up the queue to processa message and timer events
      this->spinner = new ros::AsyncSpinner(0);
      this->spinner->start();
    }

    /// \brief Handle an incoming message from ROS
    public: void OnTopCameraCmdMsg(const std_msgs::BoolConstPtr &_msg)
    {
      this->enableImageCapture = _msg->data;
      gzdbg << "Top camera capture is now " << (this->enableImageCapture ? "ON" : "OFF") << std::endl;
    }

    /// \param[in] _msg A float value that is used to set the velocity of the rear wheel
    public: void OnRearWheelCmdMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      double recvData = _msg->data; 
      // Setting Angular velocity on the two rear
      if(fabs(recvData) < ZERO_THRESHOLD)
      {   
        // This means 0 <=> brake
        this->rTargetVelocity = 0;
        this->rTargetVelocityLevel = 0;
      }
      else
      {
        if(recvData > ZERO_THRESHOLD)
        {
          // Positive value
          // So keep it limit to at most 100
          this->rTargetVelocityLevel = std::min(100.0, recvData);
        }
        else
        {
          // Negative value
          // So keep it limit to at least -100
          this->rTargetVelocityLevel = std::max(-100.0, recvData);
        }
      }
      // Convert the throttle level to velocity using the throttle function
      this->rTargetVelocity =  RCTeleopCamera::throttleLevel2Speed(this->rTargetVelocityLevel);
      // Set the rear wheel to rotate at the new speed
      this->jointController->SetVelocityTarget(this->rLeft->GetScopedName(), this->rTargetVelocity);
      this->jointController->SetVelocityTarget(this->rRight->GetScopedName(), this->rTargetVelocity);
    }

    // private: void saveFrame()
    // {
    //   if(!this->world->IsPaused() && this->newImageAvailable)
    //   {
    //     this->newImageAvailable = false;
    //     // Only save image when world is not paused
    //     std::ostringstream s;
    //     s << this->imgSavePath<< this->imgCounter++ << "_" << this->fTargetSteerAngleLevel <<".jpg";
    //     this->camera->SaveFrame(s.str());
    //   }
    // }

    // private: void captureImage(const ros::TimerEvent& event)
    // {
    //   if(this->enableImageCapture)
    //   {
    //     this->saveFrame();
    //   }
    // }

    /// \param[in] _msg A float value that is used to set the angle
    /// of the front wheel.
    public: void OnFrontWheelCmdMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      double recvData = _msg->data; 
      // _msg is the level from -100.0 to 100.0 to steer
      if(fabs(recvData) < ZERO_THRESHOLD)
      {
        // Setting Angular velocity on the two rear
       this->fTargetSteerAngleLevel = 0.0;
      }
      else
      {
        if(this->fTargetSteerAngleLevel > ZERO_THRESHOLD)
        {
          // Positive value
          // Make sure it does not exceed 100
          this->fTargetSteerAngleLevel = std::min(100.0, recvData);
        }
        else
        {
          // Negative value
          // Make sure it does not goes below -100
          this->fTargetSteerAngleLevel = std::max(-100.0, recvData);
        }
      }
      this->fTargetSteerAngle = this->fSteerIncreament * -this->fTargetSteerAngleLevel;
      this->jointController->SetPositionTarget(this->fLeft->GetScopedName(), this->fTargetSteerAngle);
      this->jointController->SetPositionTarget(this->fRight->GetScopedName(), this->fTargetSteerAngle);
      
      // // Save the image and the target angle
      // if(this->enableImageCapture)
      // {
      //   this->saveFrame();
      // }
    }

    public: void GazeboTopCameraMsgHandler(ConstImageStampedPtr& msg)
    {
      if(!this->world->IsPaused())
      {
        // Getting data and save data by opencv
        int width;
        int height;
        char *data;

        width = (int) msg->image().width();
        height = (int) msg->image().height();
        //+1 for null terminate
        data = new char[msg->image().data().length() + 1];

        memcpy(data, msg->image().data().c_str(), msg->image().data().length());
        //gazebo output rgb data
        //PixelFormat.R8G8B8
        cv::Mat image(height, width, CV_8UC3, data);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Save when toggle switch is on
        if(this->enableImageCapture)
        {
          // Build saving path
          std::stringstream s;
          s << std::fixed << std::setprecision(2) << this->imgSavePath << this->imgCounter++ << "_" << this->fTargetSteerAngleLevel << "_" << this->rTargetVelocityLevel <<".jpg";
          // Save image
          cv::imwrite(s.str(), image);
        }

        // Publish the image to ros topic
        // First convert the cv::Mat to Ros Image type
        sensor_msgs::Image msg;
        toImageMsg(image, msg);
        // Then publish the image
        this->rosPubCamera.publish(msg);

        // Remove allocated data
        delete data;  // DO NOT FORGET TO DELETE THIS, 
                      // ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
        
      }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      this->jointController->Update();
      this->newImageAvailable = true;
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RCTeleopCamera)
}
