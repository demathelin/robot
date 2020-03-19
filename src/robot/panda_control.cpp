// Copyright (c) 2017 Franka Emika GmbH0
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <velocity_qp/robot/panda_control.h>
#include <cmath>
#include <memory>
#include <chrono> 
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


using namespace std;

namespace velocity_qp{


bool PandaController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {    

    //--------------------------------------
    // LOAD ROBOT
    //--------------------------------------
    string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
        return false;
    }   
    if (!node_handle.getParam("control_level", control_level)) {
        ROS_ERROR_STREAM("Could not read parameter control_level");
        return false;
    }   
    
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
        << joint_names.size() << " instead of 7 names!");
        return false;
    }
   
    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>( state_interface->getHandle("panda_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Exception getting state handle: " << e.what());
            return false;
    }

    if (control_level == "velocity")
    {
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
                "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
                return false;
        }
         velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }
    }
    else if ( control_level == "torque")
    {
        auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM("control_level must be either velocitiy or torque");
        return false;
    }
    
    Eigen::VectorXd q_init, qd_init;
    
    q_init.resize(7);
    qd_init.resize(7);
  
    franka::RobotState robot_state = state_handle_->getRobotState(); 
    //Get robot current state
    for (int i =0 ; i<7 ; i++)
    {
        q_init(i) = robot_state.q[i];
        qd_init(i) = robot_state.dq[i];
    }
    
    qp.Init(node_handle, q_init, qd_init);
  
    
    return true;
}

void PandaController::starting(const ros::Time&)
{
    ROS_WARN_STREAM("Starting QP Controller on the real Panda");
}


void PandaController::update(const ros::Time&, const ros::Duration& period) {    

    //--------------------------------------
    // ROBOT STATE
    //--------------------------------------
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState(); 
    
    Eigen::VectorXd q_,qd_;
    q_.resize(7);
    qd_.resize(7);
    //Get robot current state
    for (int i =0 ; i<7 ; i++)
    {
        q_(i) = robot_state.q[i];
        qd_(i) = robot_state.dq[i];
    }
    
    Eigen::VectorXd joint_command_;
    joint_command_.resize(7);
    // joint_command_ = qp.update(q_,qd_,period);
    Eigen::VectorXd gravity_comp;
    gravity_comp.resize(7);
    joint_command_ = qp.update(q_,qd_,period);


    if (control_level == "velocity")
    {
        for (size_t i = 0; i < 7; ++i)
        {
            velocity_joint_handles_[i].setCommand(joint_command_(i));
        }
    }
    else if (control_level == "torque")
    {
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(joint_command_(i));
        }
    }        
    
}



}  // namespace velocity_qp

PLUGINLIB_EXPORT_CLASS(velocity_qp::PandaController,
controller_interface::ControllerBase)
