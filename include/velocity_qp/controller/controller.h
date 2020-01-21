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

/**
 * \file controller.h
 * \brief Main header
 * \author Lucas Joseph
 * \version 0.1
 * \date 21/01/2020
 *
 * Header for the control of the velocity qp
 *
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
        /**
        * \fn bool Init
        * \brief Initializes the controller
        * \param ros::NodeHandle& node_handle a ros node handle
        * \param Eigen::VectorXd q_init the robot initial joint position
        * \param Eigen::VectorXd qd_init the robot initial joint velocity
        * \return true if the controller is correctly initialized
        */
        bool Init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init);
    
        /**
        * \fn std::tuple<Eigen::VectorXd, Eigen::VectorXd> update
        * \brief Update the controller to get the new desired joint velocity to send to the robot.
        * \param Eigen::VectorXd q the current joint position of the robot
        * \param Eigen::VectorXd qd the current joint velocity of the robot
        * \param const ros::Duration& period the refresh rate of the control
        * \return A std::tupe with the desired joint velocity and information on the qp solver
        */
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
    * \fn void init_publishers
    * \brief Initializes publishers
    * \param ros::NodeHandle& node_handle a ros node handle
    */
    void init_publishers(ros::NodeHandle& node_handle);
    
    /**
    * \fn void load_parameters
    * \brief Loads parameters from yaml file
    */
    void load_parameters();
    
    /**
    * \fn void distance_callback
    * \brief Callback routine to update human-robot distance
    * \param const std_msgs::Float32::ConstPtr& msg a float32 message representing the human-robot distance
    */
    void distance_callback(const std_msgs::Float32::ConstPtr& msg);
    
    /**
    * \fn  void paramCallback
    * \brief Callback routine for the dynamic reconfigure GUI
    * \param velocity_qp::paramConfig& config the new robot config
    * \param uint32_t level not used
    */
    void paramCallback(velocity_qp::paramConfig& config, uint32_t level);
    
    /**
    * \fn bool load_robot 
    * \brief Loads Panda robot
    * \param ros::NodeHandle& node_handle a ros node handle
    * \return true if the robot is correctly loaded
    */
    bool load_robot(ros::NodeHandle& node_handle);
    
    /**
    * \fn void do_publishing
    * \brief Publishes values
    * \return publish messages
    */
    void do_publishing();
    
    /**
    * \fn void BuildTrajectory
    * \brief Build the trajectory
    * \param KDL::Frame X_curr_ the current pose of the robot
    */
    void BuildTrajectory(KDL::Frame X_curr_);
    
    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<velocity_qp::paramConfig>> dynamic_server_params_; //dynamic reconfigure node
    ros::NodeHandle dynamic_reconfigure_params_node_; //dynamic reconfigure node
    
    // Publishers
    geometry_msgs::Pose X_curr_msg_
                        ,X_traj_msg_
                        ,fake_human_pose_msg_
                        ;
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

    
    
    KDL::Chain chain; /*!< @brief KDL chain corresponding to the robot */  
    KDL::JntArray ll; /*!< @brief Joint lower limits vector*/  
    KDL::JntArray ul; /*!< @brief Joint upper limits vector*/   
    KDL::JntArray gravity_kdl; /*!< @brief KDL gravity vector  */

    KDL::Jacobian J_; /*!< @brief KDL gravity vector  */

    KDL::JntSpaceInertiaMatrix M_;  /*!< @brief KDL inertia matrix in joint space */

    KDL::Frame X_curr_; /*!< @brief KDL current Cartesian pose of the tip_link */
    KDL::Frame X_traj_; /*!< @brief KDL desired Cartesian pose of the tip_link */
    KDL::Frame X_traj_next; /*!< @brief Previous KDL desired Cartesian pose of the tip_link */
    
    KDL::FrameVel Xd_curr_; /*!< @brief KDL current Cartesian twist of the tip_link */

    KDL::Twist Xd_traj_;  /*!< @brief KDL desired Cartesian twist of the tip_link */
    KDL::Twist Xdd_traj_; /*!< @brief KDL desired Cartesian acceleration of the tip_link */
    KDL::Twist X_err_; /*!< @brief KDL desired Cartesian error between the desired and current pose */

    KDL::JntArrayVel q_in; /*!< @brief KDL joint position of the robot */
    Eigen::VectorXd p_gains_; /*!< @brief Proportional gains of the PID controller */ 
    Eigen::VectorXd i_gains_; /*!< @brief Derivative gains of the PID controller */
    Eigen::VectorXd d_gains_; /*!< @brief Integral gains of the PID controller */
    Eigen::VectorXd torque_max_; /*!< @brief Maximum allowable torque */
    Eigen::VectorXd jnt_vel_max_; /*!< @brief Maximum allowable joint velocity */
    Eigen::VectorXd p_gains_qd_; /*!< @brief Proportional gains of the regularisation controller */
    Eigen::VectorXd joint_velocity_out_; /*!< @brief Results of the QP optimization */

    Eigen::Matrix<double,6,1> xd_des_; /*!< @brief Desired robot twist of the robot tip_link */
    Eigen::Matrix<double,6,1> xd_curr_; /*!< @brief Current robot twist of the robot tip_link */
    Eigen::Matrix<double,6,1> x_curr_; /*!< @brief Current robot pose of the robot tip_link */

    double regularisation_weight_; /*!< @brief Regularisation weight */
    int dof; /*!< @brief Number of degrees of freedom of the robot */

    std::string root_link_; /*!< @brief base link of the KDL chain */
    std::string tip_link_; /*!< @brief tip link of the KDL chain (usually the end effector*/

    Eigen::Matrix<double,6,1> x_err; /*!< @brief desired Cartesian error between the desired and current pose in Eigen */  
    Eigen::Matrix <double,6,7> J; /*!< @brief Jacobian in Eigen */  
    Eigen::Matrix <double,7,7> M; /*!< @brief Inertia matrix in joint space in Eigen */  

    // Kinetic energy variables
    Eigen::Matrix<double,6,6> Lambda_; /*!< @brief Inertia Matrix in operation space in Eigen */  
    double m_u; /*!< @brief Projected mass in the direction of motion u */  
    Eigen::Matrix<double,3,1> u; /*!< @brief Direction of motion in Eigen*/  
    KDL::Vector dir_; /*!< @brief Direction of motion in KDL*/
    double ec_lim; /*!< @brief Maximum allowed kinetic energy*/

    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_; /*!< @brief Hessian matrix of the QP*/
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_; /*!< @brief Constraint matrix of the QP */

    Eigen::VectorXd g_; /*!< @brief Gradient vector of the QP */
    Eigen::VectorXd lb_; /*!< @brief Lower bound vector of the QP */
    Eigen::VectorXd ub_; /*!< @brief Upper bound vector of the QP */
    Eigen::VectorXd lbA_; /*!< @brief Constraint lower bound vector of the QP */
    Eigen::VectorXd ubA_; /*!< @brief Constraint upper bound vector of the QP */
    Eigen::VectorXd qd_min_; /*!< @brief Minimum joint velocity limit vector*/
    Eigen::VectorXd qd_max_; /*!< @brief Maximum joint velocity limit vector*/
    Eigen::VectorXd qdd_max_; /*!< @brief Maximum joint acceleration limit vector*/
    Eigen::VectorXd q_mean_; /*!< @brief Mean joint position of the robot (for the regularization task*/

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_; /*!< @brief QP solver point*/
    int number_of_constraints_; /*!< @brief Number of constraints of the QP problem*/
    int number_of_variables; /*!< @brief Number of optimization variables of the QP problem*/
    
    // Trajectory variables
    bool publish_traj; /*!< @brief Trajectory is published if true*/
    bool play_traj_; /*!< @brief Trajectory is played if true*/
    double t_traj_curr; /*!< @brief Current time along the trajectory*/
    double init_dur; /*!< @brief Time required to reach the begin of the trajectory from the robot initial position*/
    KDL::Trajectory_Composite* ctraject; /*!< @brief KDL composite trajectory object */
    KDL::Path_RoundedComposite* path; /*!< @brief KDL rounded composite path object */
    KDL::VelocityProfile* velpref; /*!< @brief KDL velocity profile object */

    // Human detection
    double distance_to_contact; /*!< @brief distance between the human and the workspace */
    
    ros::Duration elapsed_time_; /*!< @brief Time elapsed */
};
}
#endif // CONTROLLER_HPP
