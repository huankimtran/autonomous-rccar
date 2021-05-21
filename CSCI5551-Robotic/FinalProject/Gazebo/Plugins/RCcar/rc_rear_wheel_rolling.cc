#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "cmath"
/**
 * A plugin to make the rear wheels of RCCar model rotates. This plugin is included 
 * in the RCCar_Wheel_Rolling model and can be run by launching RCcarWorld_wheel_rolling.xml world
 */

/*
 * 
 * To run this, first build this folder by
 * create a folder name build, in the folder containing this file
 * cd build
 * cmake ../
 * make
 * 
 * Then launch the world by
 * ../../Launch/RCCar/RCCar_Wheel_rolling.bash
 */ 

namespace gazebo
{
  class RCWheelRolling : public ModelPlugin
  {

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RCWheelRolling::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // gzdbg << "Hello" <<std::endl;
      physics::JointPtr rLeft, rRight;
      // Getting the joints of two rear wheels
      rLeft = this->model->GetJoint("REVOLUTE_CHASSIS_LEFT_REAR_WHEEL");
      rRight = this->model->GetJoint("REVOLUTE_CHASSIS_RIGHT_REAR_WHEEL");
      // Setting Angular velocity on the two rear
      rLeft->SetVelocity(0, 2*M_PI);      // The first parameter of this function is the joint index. This is useful when dealing with joints that has 2 axis like revolute2. For revolute2, there are 2 axis, so you have to specify which axis you are trying to set the velocity. In that case, the index can be 0 or 1. Normally, for 1-axis joints, just use 0 
      rRight->SetVelocity(0, 2*M_PI);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RCWheelRolling)
}
