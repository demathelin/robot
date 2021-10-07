#include "velocity_qp/controller/controller.h"


namespace Controller{
bool Controller::init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init) 
{
    ROS_DEBUG_STREAM("Using velocity_qp controller !");

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
    J.resize(6,model->nv);
    J.setZero();

    qp = qp_solver.configureQP(number_of_variables, number_of_constraints);

    //--------------------------------------
    // INITIALIZE RBOT STATE
    //--------------------------------------
    // First calls the forwardKinematics on the model, then computes the placement of each frame.
    pinocchio::framesForwardKinematics(*model,*data,q_init);
    oMtip = data->oMf[model->getFrameId(tip_link)].toHomogeneousMatrix();

    //--------------------------------------
    // BUILD TRAJECTORY
    //--------------------------------------
    std::string panda_traj_path = ros::package::getPath("panda_traj");
    std::string trajectory_file = panda_traj_path+"/trajectories/go_to_point.csv";
    trajectory.Load(trajectory_file);
    trajectory.Build(oMtip, true);
   
    ROS_DEBUG_STREAM(" Trajectory computed ");

    return true;
}

Eigen::VectorXd Controller::update(Eigen::VectorXd q, Eigen::VectorXd qd, const ros::Duration& period)
{
    double time_dt = period.toSec();

    // First calls the forwardKinematics on the model, then computes the placement of each frame.
    pinocchio::framesForwardKinematics(*model,*data,q);

    // Compute jacobian
    pinocchio::computeJointJacobians(*model,*data,q);
    pinocchio::getFrameJacobian(*model, *data, model->getFrameId(tip_link), pinocchio::ReferenceFrame::LOCAL, J);
    
    //Update the trajectory
    trajectory.updateTrajectory(traj_properties, time_dt);

    pinocchio::SE3 oMdes(trajectory.Pos().matrix());
    
    // Compute error
    const pinocchio::SE3 tipMdes = data->oMf[model->getFrameId(tip_link)].actInv(oMdes);
    err = pinocchio::log6(tipMdes).toVector();

    // Proportional controller with feedforward
    xd_des = p_gains.cwiseProduct(err) + trajectory.Vel();
        
    // Formulate QP problem such that
    // joint_velocity_out = argmin 1/2 qd^T H_ qd + qd^T g_
    //                         s.t     lbA_ < A_ qd << ubA_
    //                                     lb_ < qd < ub_ 

    // Regularisation task
    qp.hessian =  2.0 * regularisation_weight * Eigen::MatrixXd::Identity(model->nv,model->nv);
    qp.gradient = -2.0 * regularisation_weight * p_gains_qd.cwiseProduct((q_mean- q));

    // Main task
    qp.hessian +=  2.0 *  J.transpose() * J;
    qp.gradient += -2.0 *  J.transpose() * xd_des;
    
    // Bounds
    qp.ub = model->velocityLimit;
    qp.lb = -model->velocityLimit;
  
    // Constraints
    double horizon_dt = 15 * time_dt;
    qp.a_constraints.block(0,0,7,7) = horizon_dt * Eigen::MatrixXd::Identity(7,7);    
    qp.ub_a.segment(0,7) = model->upperPositionLimit - q;
    qp.lb_a.segment(0,7) = model->lowerPositionLimit - q;
    

    // Publish some messages
    do_publishing();

    return qp_solver.SolveQP(qp);
}

void Controller::BuildTrajectory(Eigen::Affine3d oMtip)
{
    trajectory.Build(oMtip, false);
    publishTrajectory();
}

void Controller::publishTrajectory()
{
    panda_traj::PublishTraj publish_traj;
    publish_traj = trajectory.publishTrajectory();

    nav_msgs::Path path_ros;
    path_ros.poses = publish_traj.path_ros_.poses;
    path_ros.header.frame_id = "world";
    path_ros.header.stamp = ros::Time::now();
    geometry_msgs::PoseArray pose_array;
    pose_array.poses = publish_traj.pose_array_.poses;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();

    if (pose_array_publisher.trylock())
    {
        pose_array_publisher.msg_.header.stamp = ros::Time::now();
        pose_array_publisher.msg_.header.frame_id = "world";
        pose_array_publisher.msg_ = pose_array;
        pose_array_publisher.unlockAndPublish();
    }
    if (path_publisher.trylock())
    {
        path_publisher.msg_.header.stamp = ros::Time::now();
        path_publisher.msg_.header.frame_id = "world";
        path_publisher.msg_ = path_ros;
        path_publisher.unlockAndPublish();
    }
    ROS_INFO_STREAM(" Trajectory published ");
}

void Controller::init_publishers(ros::NodeHandle& node_handle){
    //Realtime safe publishers
    pose_array_publisher.init(node_handle, "Pose_array", 1);
    pose_curr_publisher.init(node_handle, "X_curr", 1);
    pose_des_publisher.init(node_handle, "X_traj", 1);
    path_publisher.init(node_handle, "Ros_Path", 1);    
    panda_rundata_publisher.init(node_handle, "panda_rundata", 1);
}

void Controller::load_parameters(){    
    ROS_INFO_STREAM ( "------------- Loading parameters -------------" );
    p_gains.resize(6);
    getRosParam("/velocity_qp/p_gains",p_gains,true);
    p_gains_qd.resize(7);
    getRosParam("/velocity_qp/p_gains_qd",p_gains_qd,true);
    q_mean.resize(7);
    getRosParam("/velocity_qp/q_mean",q_mean,true);
    getRosParam("/velocity_qp/regularisation_weight",regularisation_weight,true);
    getRosParam("/velocity_qp/tip_link",tip_link,true);
    ROS_INFO_STREAM ( "------------- Parameters Loaded -------------" );
} 

bool Controller::load_robot(ros::NodeHandle& node_handle)
{
    updateUI_service = node_handle.advertiseService("updateUI", &Controller::updateUI, this);
    updateTraj_service = node_handle.advertiseService("updateTrajectory", &Controller::updateTrajectory, this); 

    // get robot descritpion
    std::string urdf_param;
    getRosParam("/robot_description", urdf_param);
    
    // Load the urdf model
    model.reset(new pinocchio::Model);
    pinocchio::urdf::buildModelFromXML(urdf_param,*model,false);
    data.reset(new pinocchio::Data(*model));

    // Initialize the various solvers 
    number_of_variables = model->nv;
    number_of_constraints = model->nv;
    
    return true;
}

void Controller::do_publishing()
{
    // Publishing
    Eigen::Affine3d xcurr,xdes;
    xcurr =  data->oMf[model->getFrameId(tip_link)].toHomogeneousMatrix();
    tf::poseEigenToMsg(xcurr,X_curr_msg);
    
    pinocchio::SE3 oMdes(trajectory.Pos().matrix());
    xdes = oMdes.toHomogeneousMatrix();

    tf::poseEigenToMsg(xdes,X_traj_msg);
    tf::twistEigenToMsg(err, err_msg);

    // Publish custom message define in the msg folder
    if (panda_rundata_publisher.trylock())
    {
        panda_rundata_publisher.msg_.header.stamp = ros::Time::now();
        panda_rundata_publisher.msg_.header.frame_id = "world";
        panda_rundata_publisher.msg_.X_err = err_msg;
        panda_rundata_publisher.msg_.play_traj_ = traj_properties.play_traj_;
        panda_rundata_publisher.msg_.tune_gains_ = traj_properties.gain_tunning_ ;
        panda_rundata_publisher.unlockAndPublish();
    }

    //Publish robot current pose
    if (pose_curr_publisher.trylock())
    {
        geometry_msgs::PoseStamped X_curr_stamp;
        pose_curr_publisher.msg_.header.stamp = ros::Time::now();
        pose_curr_publisher.msg_.header.frame_id = "world";
        pose_curr_publisher.msg_.pose = X_curr_msg;
        pose_curr_publisher.unlockAndPublish();
    }

    //Publish robot desired pose
    if (pose_des_publisher.trylock())
    {
        geometry_msgs::PoseStamped X_des_stamp;
        pose_des_publisher.msg_.header.stamp = ros::Time::now();
        pose_des_publisher.msg_.header.frame_id = "world";
        pose_des_publisher.msg_.pose = X_traj_msg;
        pose_des_publisher.unlockAndPublish();
    }
}

// Ros service to interact with the code
bool Controller::updateUI(velocity_qp::UI::Request &req, velocity_qp::UI::Response &resp)
{
    traj_properties.play_traj_ = req.play_traj;
    if (req.publish_traj)
        publishTrajectory();
    if (req.build_traj)
    {
        Eigen::Affine3d X_curr(data->oMf[model->getFrameId(tip_link)].toHomogeneousMatrix()); 
        BuildTrajectory(X_curr);
    }
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

  Eigen::Affine3d X_curr(data->oMf[model->getFrameId(tip_link)].toHomogeneousMatrix()); 
  trajectory.Build(X_curr, req.verbose);
  publishTrajectory();
  std::cout << "Received waypoint computing traj and publishing" << std::endl;

  return true;
}

}
