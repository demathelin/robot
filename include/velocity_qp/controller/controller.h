/*
 * Copyright 2019 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <velocity_qp/paramConfig.h>

#include <qpOASES.hpp>
#include <boost/scoped_ptr.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

#include <velocity_qp/PandaRunMsg.h>

#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

namespace Controller {
class Controller{
public:
        bool init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init);
        std::tuple<Eigen::VectorXd, Eigen::VectorXd> update(Eigen::VectorXd q, Eigen::VectorXd qd, const ros::Duration& period);

private:
    
    template<class T>
    bool getRosParam(const std::string& param_name, T& param_data)
    {
        if(!ros::param::get(param_name,param_data))
        {
            ROS_FATAL_STREAM(" Problem loading parameters " << param_name << ". Check the Yaml config file. Killing ros");
            ros::shutdown();
        }
        else
            ROS_INFO_STREAM(param_name << " : " << param_data);
        return true;
    }
   
    bool getRosParam(const std::string& param_name, Eigen::VectorXd& param_data)
{
    std::vector<double> std_param;
    if(!ros::param::get(param_name,std_param))
    {
        ROS_FATAL_STREAM(" Problem loading parameters " << param_name << ". Check the Yaml config file. Killing ros");
        ros::shutdown();
    }
    else
    {
        if(std_param.size() == param_data.size())
        {
            for(int i = 0; i < param_data.size() ; ++i )
                param_data(i) = std_param[i];
            ROS_INFO_STREAM(param_name << " : [" << param_data.transpose() << "]");
            return true;
        }
        else
        {
            ROS_FATAL_STREAM("Wrong matrix size for param " << param_name << ". Check the Yaml config file. Killing ros" );
            ros::shutdown();
        }
    }
}
    /**
    * @brief Initializes publishers
    */
    void init_publishers(ros::NodeHandle& node_handle);
    
    /**
    * @brief Loads parameters from yaml file
    */
    void load_parameters();
    
    /**
    * @brief Callback routine to update human-robot distance
    */
    void distance_callback(const std_msgs::Float32::ConstPtr& msg);
    
    /**
    * @brief Callback routine for the dynamic reconfigure GUI
    */
    void paramCallback(velocity_qp::paramConfig& config, uint32_t level);
    
    /**
    * @brief Loads Panda robot
    */
    bool load_robot(ros::NodeHandle& node_handle);
    
    /**
    * @brief Publishes values
    */
    void do_publishing();
    
    /**
    * @brief Build the trajectory
    */
    void BuildTrajectory(KDL::Frame X_curr_);
    
    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<velocity_qp::paramConfig>> dynamic_server_params_;
    ros::NodeHandle dynamic_reconfigure_params_node_;
    
    // Publishers
    geometry_msgs::Pose X_curr_msg_,X_traj_msg_,fake_human_pose_msg_;
    geometry_msgs::Twist X_err_msg_,Xd_msg_,Xdd_msg_,xd_des_msg_;  
    realtime_tools::RealtimePublisher<geometry_msgs::PoseArray> pose_array_publisher;
    realtime_tools::RealtimePublisher<nav_msgs::Path> path_publisher;
    realtime_tools::RealtimePublisher<velocity_qp::PandaRunMsg> panda_rundata_publisher;
    
    // Subscribers
    ros::Subscriber human_workspace_dist_sub;
    
    boost::shared_ptr<TRAC_IK::TRAC_IK> ik_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> chainjacsolver_;
    
    /**
    * @brief The dynamic solver.
    */
    boost::scoped_ptr<KDL::ChainDynParam> dynModelSolver_;
    
    /**
    * @brief The forward kinematic solver for position
    */
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;

    /**
    * @brief The forward kinematic solver for velocity
    */
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fksolvervel_;

    
    
    KDL::Chain chain;
    KDL::JntArray ll, ul, q_kdl,qd_kdl,coriolis_kdl,gravity_kdl, tau_kdl;
    KDL::Twist Jdqd_kdl;
    KDL::Jacobian J_;
    KDL::JntSpaceInertiaMatrix M_;    
    KDL::Frame X_curr_,X_last_, X_traj_,X_traj_next;
    KDL::FrameVel Xd_curr_;
    KDL::Twist Xd_traj_, Xdd_traj_, X_err_, Xd_;
    KDL::JntArrayVel q_in;
    Eigen::VectorXd p_gains_, i_gains_, d_gains_, torque_max_, jnt_vel_max_, p_gains_qd_, damping_weight_, joint_velocity_out_;
    Eigen::Matrix<double,6,1>  xd_des_, xd_curr_, x_curr_;
    double regularisation_weight_,transition_gain_;
    int dof;
    std::string root_link_,tip_link_;
    
    
    Eigen::Matrix<double,6,1> x_err;
    Eigen::Matrix <double,6,7> J;
    Eigen::Matrix <double,7,7> M;

    // Kinetic energy variables
    Eigen::Matrix<double,6,6> Lambda_;
    double m_u;
    Eigen::Matrix<double,3,1> u;
    KDL::Vector dir_;
    double ec_lim;

    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::VectorXd g_;
    Eigen::VectorXd lb_, ub_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd lbA_, ubA_, qd_min_, qd_max_,qdd_max_, q_mean_;
    Eigen::VectorXd nonLinearTerms_;
    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_;
    int number_of_constraints_;
    int number_of_variables;
    
    // Trajectory variables
    bool publish_traj,play_traj_;
    double t_traj_curr,init_dur;
    KDL::Trajectory_Composite* ctraject;
    KDL::Path_RoundedComposite* path;
    KDL::VelocityProfile* velpref;

    // Human detection
    double distance_to_contact;
    
    ros::Duration elapsed_time_;
};
}
#endif // CONTROLLER_HPP
