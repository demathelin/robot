/*
 * Copyright 2020 <copyright holder> <email>
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

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/package.h>
#include <Eigen/Dense>

#include <velocity_qp/UI.h>
#include <velocity_qp/PandaRunMsg.h>

#include <qp_solver/qp_solver.hpp>
#include <qpOASES.hpp>
#include <boost/scoped_ptr.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <panda_traj/panda_traj.hpp>


namespace Controller {
    /**
     * \class The controller class
    */
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
    bool init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init);

    /**
    * \fn Eigen::VectorXd update
    * \brief Update the controller to get the new desired joint velocity to send to the robot.
    * \param Eigen::VectorXd q the current joint position of the robot
    * \param Eigen::VectorXd qd the current joint velocity of the robot
    * \param const ros::Duration& period the refresh rate of the control
    * \return A Eigen::VectorXd with the desired joint velocity
    */
    Eigen::VectorXd update(Eigen::VectorXd q, Eigen::VectorXd qd, const ros::Duration& period);

private:
    
template <class T>
  bool getRosParam(const std::string& param_name, T& param_data, bool verbose = false)
  {
    if (!ros::param::get(param_name, param_data))
    {
      ROS_FATAL_STREAM(" Problem loading parameters " << param_name << ". Check the Yaml config file. Killing ros");
      ros::shutdown();
    }
    else
    {
      if (verbose)
        ROS_INFO_STREAM(param_name << " : " << param_data);
    }
    return true;
  }

  bool getRosParam(const std::string& param_name, Eigen::VectorXd& param_data, bool verbose = false)
  {
    std::vector<double> std_param;
    if (!ros::param::get(param_name, std_param))
    {
      ROS_FATAL_STREAM(" Problem loading parameters " << param_name << ". Check the Yaml config file. Killing ros");
      ros::shutdown();
    }
    else
    {
      if (std_param.size() == param_data.size())
      {
        for (int i = 0; i < param_data.size(); ++i)
          param_data(i) = std_param[i];
        if (verbose)
          ROS_INFO_STREAM(param_name << " : " << param_data.transpose());
        return true;
      }
      else
      {
        ROS_FATAL_STREAM("Wrong matrix size for param " << param_name << ". Check the Yaml config file. Killing ros");
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
    void BuildTrajectory(Eigen::Affine3d oMtip);

    /**
    * @brief Publish the trajectory
    */
    void publishTrajectory();

    /**
    * @brief ros service to interact with the robot
    */
    bool updateUI(velocity_qp::UI::Request& req, velocity_qp::UI::Response& resp);

    /**
    * @brief ros service to update the trajectory
    */
    bool updateTrajectory(panda_traj::UpdateTrajectory::Request &req, panda_traj::UpdateTrajectory::Response &resp);
    
    
    // Publishers
    geometry_msgs::Pose X_curr_msg
                        ,X_traj_msg;
    geometry_msgs::Twist err_msg;  
    realtime_tools::RealtimePublisher<geometry_msgs::PoseArray> pose_array_publisher;
    realtime_tools::RealtimePublisher<nav_msgs::Path> path_publisher;
    realtime_tools::RealtimePublisher<velocity_qp::PandaRunMsg> panda_rundata_publisher;
    realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pose_curr_publisher, pose_des_publisher;
    
    ros::ServiceServer updateUI_service,
                       updateTraj_service;
    
    std::shared_ptr<pinocchio::Model> model;
    std::shared_ptr<pinocchio::Data> data;
    Eigen::Matrix<double,6,1> err;

    pinocchio::Data::Matrix6x J;

    KDL::JntArrayVel q_in; /*!< @brief KDL joint position of the robot */
    Eigen::VectorXd p_gains; /*!< @brief Proportional gains of the PID controller */ 
    Eigen::VectorXd p_gains_qd; /*!< @brief Proportional gains of the regularisation controller */
    Eigen::VectorXd joint_velocity_out; /*!< @brief Results of the QP optimization */

    Eigen::Matrix<double,6,1> xd_des; /*!< @brief Desired robot twist of the robot tip_link */

    double regularisation_weight; /*!< @brief Regularisation weight */

    std::string tip_link; /*!< @brief tip link of the KDL chain (usually the end effector*/
    Eigen::Affine3d oMtip; 

    Eigen::VectorXd q_mean; /*!< @brief Mean joint position of the robot (for the regularization task*/

    QPSolver qp_solver;
    QPSolver::qpProblem qp;

    int number_of_constraints; /*!< @brief Number of constraints of the QP problem*/
    int number_of_variables; /*!< @brief Number of optimization variables of the QP problem*/
    
    // Trajectory variables
    TrajectoryGenerator trajectory; /*!< @brief TrajectoryGenerator object */
    panda_traj::TrajProperties traj_properties; /*!< @brief Properties of the trajectory */
};
}
#endif // CONTROLLER_HPP
