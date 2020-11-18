#include "velocity_qp/controller/controller.h"

using namespace KDL;
using namespace std;

namespace Controller{
bool Controller::Init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init) 
{
    ROS_WARN("Using velocity_qp controller !!! ");

    //--------------------------------------
    // INITIALIZE PUBLISHERS
    //--------------------------------------
    init_publishers(node_handle);
    
    //--------------------------------------
    // LOAD PARAMETERS
    //--------------------------------------
    load_parameters();
    
    //--------------------------------------
    // LOAD ROBOT
    //--------------------------------------
    if (!load_robot(node_handle))
        return false;
    
    //--------------------------------------
    // INITIALIZE VARIABLES
    //--------------------------------------  
    J_.resize(chain.getNrOfJoints());
    M_.resize(chain.getNrOfJoints());

    H_.resize(number_of_variables,number_of_variables);
    g_.resize(number_of_variables);
    lb_.resize(number_of_variables);
    ub_.resize(number_of_variables);
    A_.resize(number_of_constraints_,number_of_variables);
    lbA_.resize(number_of_constraints_);
    ubA_.resize(number_of_constraints_);
    joint_velocity_out_.resize(dof);
    
    //--------------------------------------
    // QPOASES
    //--------------------------------------
    qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints_,qpOASES::HST_POSDEF));

    // QPOases options
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    // options.setToDefault();
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    qpoases_solver_->setOptions( options );
    qpoases_solver_->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none
    
    //--------------------------------------
    // INITIALIZE RBOT STATE
    //--------------------------------------
    q_in.q.data = q_init;
    q_in.qdot.data = qd_init;
    fksolver_->JntToCart(q_in.q,X_curr_);
    
    //--------------------------------------
    // BUILD TRAJECTORY
    //--------------------------------------
    std::string panda_traj_path = ros::package::getPath("panda_traj");
    std::string trajectory_file = panda_traj_path+"/trajectories/go_to_point.csv";
    std::string csv_file_name = trajectory_file;
    trajectory.Load(csv_file_name);
    trajectory.Build(X_curr_, true);
    ROS_DEBUG_STREAM(" Trajectory computed ");

    return true;
}

Eigen::VectorXd Controller::update(Eigen::VectorXd q, Eigen::VectorXd qd, const ros::Duration& period)
{
    double time_dt = period.toSec();
    
    //Get robot current state
    q_in.q.data = q;
    q_in.qdot.data = qd;

    // Update the model
    chainjacsolver_->JntToJac(q_in.q,J_);
    fksolver_->JntToCart(q_in.q,X_curr_);

    //Update the trajectory
    trajectory.updateTrajectory(traj_properties_, time_dt);
    if (traj_properties_.move_)
        traj_properties_.move_ = false;
    X_traj_ = trajectory.Pos();
    Xd_traj_ = trajectory.Vel();

    // Proportionnal controller 
    X_err_ = diff( X_curr_ , X_traj_ ); 
    tf::twistKDLToEigen(X_err_,x_err);
    xd_des_ = p_gains_.cwiseProduct(x_err) + Xd_traj_;
        

    // Formulate QP problem such that
    // joint_velocity_out_ = argmin 1/2 qd^T H_ qd + qd^T g_
    //                         s.t     lbA_ < A_ qd << ubA_
    //                                     lb_ < qd < ub_ 

    J = J_.data;
    M = M_.data;
    
    H_ =  2.0 * regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
    g_ = -2.0 * regularisation_weight_ * p_gains_qd_.cwiseProduct((q_mean_ - q));

    H_ +=  2.0 *  J.transpose() * J;
    g_ += -2.0 *  J.transpose() * xd_des_;
    
    double horizon_dt = 15 * time_dt;
    
    ub_ = qd_max_;
    lb_ = qd_min_;
  
    A_.block(0,0,7,7) = horizon_dt * Eigen::MatrixXd::Identity(7,7);    
    ubA_.segment(0,7) = ul.data - q;
    lbA_.segment(0,7) = ll.data - q;
    
    // number of allowed compute steps
    int nWSR = 1e6;

    // Let's compute !
    qpOASES::returnValue ret;
    static bool qpoases_initialized = false;

    if(!qpoases_initialized){
        // Initialise the problem, once it has found a solution, we can hotstart
        ret = qpoases_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        // Keep init if it didn't work
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = true;
    }
    else{
        // Otherwise let's reuse the previous solution to find a solution faster
        ret = qpoases_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        if(ret != qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = false;
    }
    
    // Zero velocity if no solution found
    joint_velocity_out_.setZero();

    // If successful_return get the primal solution
    if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_solver_->getPrimalSolution(joint_velocity_out_.data());
    else
        ROS_WARN_STREAM("QPOases failed! Sending zero velocity");
    
    // Publish some messages
    do_publishing();

    return joint_velocity_out_;
}

void Controller::BuildTrajectory(KDL::Frame X_curr_)
{
    trajectory.Build(X_curr_, false);
    publishTrajectory();
}

void Controller::publishTrajectory()
{
    panda_traj::PublishTraj publish_traj_;
    publish_traj_ = trajectory.publishTrajectory();

    nav_msgs::Path path_ros;
    path_ros.poses = publish_traj_.path_ros_.poses;
    path_ros.header.frame_id = root_link_;
    path_ros.header.stamp = ros::Time::now();
    geometry_msgs::PoseArray pose_array;
    pose_array.poses = publish_traj_.pose_array_.poses;
    pose_array.header.frame_id = root_link_;
    pose_array.header.stamp = ros::Time::now();

    if (pose_array_publisher.trylock())
    {
        pose_array_publisher.msg_.header.stamp = ros::Time::now();
        pose_array_publisher.msg_.header.frame_id = root_link_;
        pose_array_publisher.msg_ = pose_array;
        pose_array_publisher.unlockAndPublish();
    }
    if (path_publisher.trylock())
    {
        path_publisher.msg_.header.stamp = ros::Time::now();
        path_publisher.msg_.header.frame_id = root_link_;
        path_publisher.msg_ = path_ros;
        path_publisher.unlockAndPublish();
    }
    ROS_INFO_STREAM(" Trajectory published ");
}

void Controller::init_publishers(ros::NodeHandle& node_handle){
    //Realtime safe publishers
    pose_array_publisher.init(node_handle, "Pose_array", 1);
    path_publisher.init(node_handle, "Ros_Path", 1);    
    panda_rundata_publisher.init(node_handle, "panda_rundata", 1);
}

void Controller::load_parameters(){    
    ROS_INFO_STREAM ( "------------- Loading parameters -------------" );
    qd_min_.resize(7);
    getRosParam("/velocity_qp/qd_min_",qd_min_);
    qd_max_.resize(7);
    getRosParam("/velocity_qp/qd_max_",qd_max_);
    p_gains_.resize(6);
    getRosParam("/velocity_qp/p_gains_",p_gains_);
    d_gains_.resize(6);
    getRosParam("/velocity_qp/d_gains_",d_gains_);
    p_gains_qd_.resize(7);
    getRosParam("/velocity_qp/p_gains_qd_",p_gains_qd_);
    q_mean_.resize(7);
    getRosParam("/velocity_qp/q_mean_",q_mean_);
    getRosParam("/velocity_qp/regularisation_weight_",regularisation_weight_);
    getRosParam("/velocity_qp/root_link_",root_link_);
    getRosParam("/velocity_qp/tip_link_",tip_link_);
    ROS_INFO_STREAM ( "------------- Parameters Loaded -------------" );
} 

bool Controller::load_robot(ros::NodeHandle& node_handle)
{
    updateUI_service = node_handle.advertiseService("updateUI", &Controller::updateUI, this);
    updateTraj_service = node_handle.advertiseService("updateTrajectory", &Controller::updateTrajectory, this); 

    // get robot descritpion
    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    std::string urdf_param;
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    double eps = 1e-5;
    // Initialize the KDL Chain
    ik_solver.reset(new TRAC_IK::TRAC_IK(root_link_, tip_link_, urdf_param, timeout, eps));
    bool valid = ik_solver->getKDLChain(chain);

    if (!valid) {
        ROS_ERROR_STREAM("There was no valid KDL chain found");
        return false;
    }
    // Get the limits from the urdf
    valid = ik_solver->getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR_STREAM("There were no valid KDL joint limits found");
        return false;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    if(chain.getNrOfSegments() == 0)
        ROS_WARN("KDL chain empty !");

    ROS_INFO_STREAM("  Chain has "<<chain.getNrOfJoints()<<" joints");
    ROS_INFO_STREAM("  Chain has "<<chain.getNrOfSegments()<<" segments");

    for(unsigned int i=0; i<chain.getNrOfSegments(); ++i)
        ROS_INFO_STREAM("    "<<chain.getSegment(i).getName());

    // Initialize the various solvers 
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain)); // Forward kinematic solver for geometric purpose
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(chain));// Forward kinematic solver for kinematic purpose
    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(chain));  // To get the jacobian

    dof = chain.getNrOfJoints();
    number_of_variables = dof;
    number_of_constraints_ = dof;

    ROS_INFO_STREAM ( "Number of variables : " << number_of_variables );
    ROS_INFO_STREAM ( "Number of constraints : " << number_of_constraints_);
    
    return true;
}

void Controller::do_publishing()
{
    // Publishing
    tf::poseKDLToMsg(X_curr_, X_curr_msg_);
    tf::poseKDLToMsg(X_traj_, X_traj_msg_);
    tf::twistKDLToMsg(X_err_, X_err_msg_);

    // Publish custom message define in the msg folder
    if (panda_rundata_publisher.trylock())
    {
        panda_rundata_publisher.msg_.header.stamp = ros::Time::now();
        panda_rundata_publisher.msg_.header.frame_id = root_link_;
        panda_rundata_publisher.msg_.X_err = X_err_msg_;
        panda_rundata_publisher.msg_.play_traj_ = traj_properties_.play_traj_;
        panda_rundata_publisher.msg_.tune_gains_ = traj_properties_.gain_tunning_ ;
        panda_rundata_publisher.unlockAndPublish();
    }

    //Publish robot current pose
    if (pose_curr_publisher.trylock())
    {
        tf::poseKDLToMsg(X_curr_, X_curr_msg_);
        geometry_msgs::PoseStamped X_curr_stamp;
        pose_curr_publisher.msg_.header.stamp = ros::Time::now();
        pose_curr_publisher.msg_.header.frame_id = root_link_;
        pose_curr_publisher.msg_.pose = X_curr_msg_;
        pose_curr_publisher.unlockAndPublish();
    }

    //Publish robot desired pose
    if (pose_des_publisher.trylock())
    {
        tf::poseKDLToMsg(X_traj_, X_traj_msg_);
        geometry_msgs::PoseStamped X_des_stamp;
        pose_des_publisher.msg_.header.stamp = ros::Time::now();
        pose_des_publisher.msg_.header.frame_id = root_link_;
        pose_des_publisher.msg_.pose = X_traj_msg_;
        pose_des_publisher.unlockAndPublish();
    }
}

// Ros service to interact with the code
bool Controller::updateUI(velocity_qp::UI::Request &req, velocity_qp::UI::Response &resp)
{
    traj_properties_.play_traj_ = req.play_traj;
    if (req.publish_traj)
        publishTrajectory();
    if (req.build_traj)
        BuildTrajectory(X_curr_);

    if (req.exit_)
    {
        ros::shutdown();
        exit(0);
    } 
    resp.result = true;

    return true;
}

// Ros service to update the trajectory
bool Controller::updateTrajectory(panda_traj::UpdateTrajectory::Request& req, panda_traj::UpdateTrajectory::Response& resp){

  trajectory.Load(req.csv_traj_path);
  trajectory.Build(X_curr_, req.verbose);
  publishTrajectory();
  std::cout << "Received waypoint computing traj and publishing" << std::endl;

  return true;
}

}
