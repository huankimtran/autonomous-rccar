#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "cmath"

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
  class RCKeyListen : public ModelPlugin
  {

    // Gazebo stuffs
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr rLeft, rRight, fLeft, fRight;

    private: physics::JointControllerPtr jointController;

    // Concurrent parameter for the joints
    private: double fTargetSteerAngle, rTargetVelocity, fSteerLimit; 

    // ROS stuffs

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSubRearWheelSpeed;
    private: ros::Subscriber rosSubFrontWheelAngle;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Getting the joints of two rear wheels
      this->rLeft = this->model->GetJoint("REVOLUTE_CHASSIS_LEFT_REAR_WHEEL");
      this->rRight = this->model->GetJoint("REVOLUTE_CHASSIS_RIGHT_REAR_WHEEL");

      // Getting the joints of two front wheels
      this->fLeft = this->model->GetJoint("REVOLUTE_CHASSIS_LEFT_FRONT_WHEEL");
      this->fRight = this->model->GetJoint("REVOLUTE_CHASSIS_RIGHT_FRONT_WHEEL");

      // Setting initial value
      this->fTargetSteerAngle = 0.0;
      this->rTargetVelocity = 0.0;

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
      this->fSteerLimit = M_PI*30.0/180.0;    // 30 degree in radian
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
          std::bind(&RCKeyListen::OnUpdate, this));

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

      // Create a named topic, and subscribe to it.

      // One to control the rear wheel speed
      ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/control/rear",
            1,
            boost::bind(&RCKeyListen::OnRearWheelCmdMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubRearWheelSpeed = this->rosNode->subscribe(so1);

      // One to control the front wheel steering angle
      ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/control/front",
            1,
            boost::bind(&RCKeyListen::OnFrontWheelCmdMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubFrontWheelAngle = this->rosNode->subscribe(so2);

      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&RCKeyListen::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS

    /// \param[in] _msg A float value that is used to set the velocity of the rear wheel
    public: void OnRearWheelCmdMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      // Setting Angular velocity on the two rear
      if(fabs(_msg->data) < 1e-3)
      {
        // This means break
        this->rTargetVelocity = 0;
      }
      else
      {
        this->rTargetVelocity += _msg->data;
      }
      this->jointController->SetVelocityTarget(this->rLeft->GetScopedName(), this->rTargetVelocity);
      this->jointController->SetVelocityTarget(this->rRight->GetScopedName(), this->rTargetVelocity);
      gzdbg << this->rTargetVelocity << std::endl;
    }

    /// \param[in] _msg A float value that is used to set the angle
    /// of the front wheel.
    public: void OnFrontWheelCmdMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      // Setting Angular velocity on the two rear
      this->fTargetSteerAngle -= _msg->data;
      if(fabs(this->fTargetSteerAngle) > fabs(this->fSteerLimit))
      {
        this->fTargetSteerAngle = this->fTargetSteerAngle > 0 ? this->fSteerLimit : -this->fSteerLimit;
      }
      this->jointController->SetPositionTarget(this->fLeft->GetScopedName(), this->fTargetSteerAngle);
      this->jointController->SetPositionTarget(this->fRight->GetScopedName(), this->fTargetSteerAngle);
      
      gzdbg << this->fTargetSteerAngle<< std::endl;
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      this->jointController->Update();
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RCKeyListen)
}
